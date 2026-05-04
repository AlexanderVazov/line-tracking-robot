/*
 * ESP32-CAM Line Detector with LED Feedback
 * Hardware: AI-Thinker ESP32-CAM
 *
 * Behaviour:
 *   - LED flickers (~4 Hz) when any line distinguishable from its background is detected
 *   - LED stays steady when no line is present
 *   - Automatically adjusts LED brightness so the scene is well-lit for detection
 *
 * No motors used. GPIO 4 drives the onboard flash LED exclusively.
 *
 * Detection works for dark-on-light AND light-on-dark lines by measuring
 * per-row deviation from the row mean rather than a fixed threshold.
 *
 * LEDC channel assignments:
 *   Channel 0 → LED (GPIO 4)
 *   Channel 2 → Camera XCLK  (reserved by esp_camera)
 */

#include "esp_camera.h"

// ── Camera pin map (AI-Thinker) ───────────────────────────────────────────────
#define PWDN_GPIO_NUM   32
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM    0
#define SIOD_GPIO_NUM   26
#define SIOC_GPIO_NUM   27
#define Y9_GPIO_NUM     35
#define Y8_GPIO_NUM     34
#define Y7_GPIO_NUM     39
#define Y6_GPIO_NUM     36
#define Y5_GPIO_NUM     21
#define Y4_GPIO_NUM     19
#define Y3_GPIO_NUM     18
#define Y2_GPIO_NUM      5
#define VSYNC_GPIO_NUM  25
#define HREF_GPIO_NUM   23
#define PCLK_GPIO_NUM   22

// ── LED ───────────────────────────────────────────────────────────────────────
#define LED_PIN      4
#define LED_CHAN     0       // LEDC channel — must not be 2 (reserved by camera)
#define LED_PWM_FREQ 5000   // 5 kHz carrier; visual flicker is done in software
#define LED_PWM_RES  8      // 8-bit duty → 0–255

// ── Image ─────────────────────────────────────────────────────────────────────
#define IMG_WIDTH   160
#define IMG_HEIGHT  120

// ── Line detection ────────────────────────────────────────────────────────────
// A row "has a line" when some pixels deviate significantly from that row's mean,
// and the number of such pixels fits within a plausible line width.
#define CONTRAST_THRESHOLD  40    // |pixel - rowMean| must exceed this
#define MIN_LINE_PX          4    // Narrowest acceptable line (pixels)
#define MAX_LINE_PX         80    // Widest acceptable line — avoids false triggers
                                  // from globally bright or dark scenes (half of 160)
#define MIN_LINE_ROWS        8    // How many rows must agree before reporting a line

// ── Auto-brightness ───────────────────────────────────────────────────────────
#define TARGET_MEAN     120   // Desired average pixel value (0–255)
#define BRIGHTNESS_MARGIN 20  // Dead-band around target — no adjustment in this range
#define BRIGHTNESS_STEP   8   // Duty-cycle change per adjustment tick
#define MIN_LED_DUTY     15   // Never go fully off while scanning (need some light)
#define MAX_LED_DUTY    255

// ── Flicker timing ────────────────────────────────────────────────────────────
#define FLICKER_HALF_PERIOD_MS  125   // Toggle every 125 ms → ~4 Hz visible flicker

// ─────────────────────────────────────────────────────────────────────────────

static int           ledDuty      = 80;    // Start at medium brightness
static bool          ledOn        = true;
static unsigned long lastToggle   = 0;

void setLED(int duty) {
    ledcWrite(LED_PIN, constrain(duty, 0, 255));
}

// ─────────────────────────────────────────────────────────────────────────────

bool setupCamera() {
    camera_config_t cfg;
    cfg.ledc_channel  = LEDC_CHANNEL_2;  // Do not change — must not clash with LED channel 0
    cfg.ledc_timer    = LEDC_TIMER_2;
    cfg.pin_d0        = Y2_GPIO_NUM;
    cfg.pin_d1        = Y3_GPIO_NUM;
    cfg.pin_d2        = Y4_GPIO_NUM;
    cfg.pin_d3        = Y5_GPIO_NUM;
    cfg.pin_d4        = Y6_GPIO_NUM;
    cfg.pin_d5        = Y7_GPIO_NUM;
    cfg.pin_d6        = Y8_GPIO_NUM;
    cfg.pin_d7        = Y9_GPIO_NUM;
    cfg.pin_xclk      = XCLK_GPIO_NUM;
    cfg.pin_pclk      = PCLK_GPIO_NUM;
    cfg.pin_vsync     = VSYNC_GPIO_NUM;
    cfg.pin_href      = HREF_GPIO_NUM;
    cfg.pin_sscb_sda  = SIOD_GPIO_NUM;
    cfg.pin_sscb_scl  = SIOC_GPIO_NUM;
    cfg.pin_pwdn      = PWDN_GPIO_NUM;
    cfg.pin_reset     = RESET_GPIO_NUM;
    cfg.xclk_freq_hz  = 20000000;
    cfg.pixel_format  = PIXFORMAT_GRAYSCALE;
    cfg.frame_size    = FRAMESIZE_QQVGA;  // 160×120
    cfg.jpeg_quality  = 12;
    cfg.fb_count      = 2;

    if (esp_camera_init(&cfg) != ESP_OK) {
        Serial.println("Camera init failed");
        return false;
    }
    Serial.println("Camera OK");
    return true;
}

// ── Image analysis ────────────────────────────────────────────────────────────

// Returns mean grayscale value of the whole frame (used for brightness control).
int frameMean(const uint8_t* pixels) {
    long sum = 0;
    for (int i = 0; i < IMG_WIDTH * IMG_HEIGHT; i++) sum += pixels[i];
    return (int)(sum / (IMG_WIDTH * IMG_HEIGHT));
}

/*
 * Returns true if a line distinguishable from its background is found.
 *
 * For each row: count pixels whose value deviates from the row mean by more
 * than CONTRAST_THRESHOLD (catches both dark-on-light and light-on-dark lines).
 * If the count falls within [MIN_LINE_PX, MAX_LINE_PX] the row contains a line.
 * Report a line when at least MIN_LINE_ROWS agree.
 */
bool detectLine(const uint8_t* pixels) {
    int lineRows = 0;

    for (int row = 0; row < IMG_HEIGHT; row++) {
        const int offset = row * IMG_WIDTH;

        // Row mean
        long rowSum = 0;
        for (int x = 0; x < IMG_WIDTH; x++) rowSum += pixels[offset + x];
        int rowMean = (int)(rowSum / IMG_WIDTH);

        // Count pixels that deviate significantly from the row mean
        int devCount = 0;
        for (int x = 0; x < IMG_WIDTH; x++) {
            if (abs((int)pixels[offset + x] - rowMean) > CONTRAST_THRESHOLD) devCount++;
        }

        if (devCount >= MIN_LINE_PX && devCount <= MAX_LINE_PX) lineRows++;
    }

    return lineRows >= MIN_LINE_ROWS;
}

/*
 * Nudges ledDuty up or down based on how bright the frame looks.
 * Only called when the LED is at a stable level (not flickering) so
 * the measurement reflects actual scene brightness, not a dark flicker phase.
 */
void adjustBrightness(int mean) {
    if (mean < TARGET_MEAN - BRIGHTNESS_MARGIN) {
        ledDuty = min(ledDuty + BRIGHTNESS_STEP, MAX_LED_DUTY);
    } else if (mean > TARGET_MEAN + BRIGHTNESS_MARGIN) {
        ledDuty = max(ledDuty - BRIGHTNESS_STEP, MIN_LED_DUTY);
    }
}

// ─────────────────────────────────────────────────────────────────────────────

static bool cameraReady = false;

void setup() {
    Serial.begin(115200);
    delay(500);

    // LED must be configured before camera so it can claim channel 0 first
    ledcAttachChannel(LED_PIN, LED_PWM_FREQ, LED_PWM_RES, LED_CHAN);
    setLED(ledDuty);

    for (int attempt = 1; attempt <= 5 && !cameraReady; attempt++) {
        Serial.printf("Camera init attempt %d/5\n", attempt);
        cameraReady = setupCamera();
        if (!cameraReady) { esp_camera_deinit(); delay(500); }
    }

    if (!cameraReady) {
        Serial.println("Camera failed — check wiring");
        // Rapid blink signals hardware error
        while (true) {
            setLED(255); delay(80);
            setLED(0);   delay(80);
        }
    }

    Serial.println("Ready — watching for lines");
}

void loop() {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("Frame capture failed");
        delay(30);
        return;
    }

    int  mean      = frameMean(fb->buf);
    bool lineFound = detectLine(fb->buf);
    esp_camera_fb_return(fb);

    if (lineFound) {
        // Flicker LED at ~4 Hz; freeze brightness so we don't adjust during flicker
        unsigned long now = millis();
        if (now - lastToggle >= FLICKER_HALF_PERIOD_MS) {
            ledOn = !ledOn;
            setLED(ledOn ? ledDuty : 0);
            lastToggle = now;
        }
        Serial.printf("LINE  mean=%3d  led=%3d\n", mean, ledDuty);
    } else {
        // Steady LED — safe to measure brightness and adjust
        ledOn = true;
        adjustBrightness(mean);
        setLED(ledDuty);
        Serial.printf("none  mean=%3d  led=%3d\n", mean, ledDuty);
    }

    delay(15);
}
