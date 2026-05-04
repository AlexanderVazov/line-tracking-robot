/*
 * ESP32-CAM Line Following Robot
 * Hardware: AI-Thinker ESP32-CAM + L298N Motor Driver
 *
 * L298N Wiring:
 *   ENA  -> GPIO 2   (Left motor PWM)
 *   IN1  -> GPIO 14  (Left motor forward)
 *   IN2  -> GPIO 15  (Left motor reverse)
 *   ENB  -> GPIO 3   (Right motor PWM)
 *   IN3  -> GPIO 13  (Right motor forward)
 *   IN4  -> GPIO 12  (Right motor reverse)
 *
 * Notes:
 *   - GPIO 3 is used for ENB; GPIO 4 (flash LED) is now free
 *   - GPIO 3 is U0RXD — Serial monitor will not work while ENB is connected
 *   - Camera uses LEDC_CHANNEL_2 / LEDC_TIMER_2 for XCLK
 *   - Motors use LEDC channels 0 and 1 explicitly to avoid conflict
 *   - Power motors from separate supply; share GND with ESP32-CAM
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

// ── Motor driver pins ─────────────────────────────────────────────────────────
#define ENA_PIN  2    // Left  motor PWM
#define IN1_PIN  14   // Left  motor +
#define IN2_PIN  15   // Left  motor -
#define ENB_PIN  3    // Right motor PWM (U0RXD — Serial monitor unavailable when in use)
#define IN3_PIN  13   // Right motor +
#define IN4_PIN  12   // Right motor -

// Explicit LEDC channels — camera uses channel 2, motors use 0 and 1
#define ENA_CHAN    0
#define ENB_CHAN    1
#define PWM_FREQ 1000
#define PWM_RES     8

// ── Tuning ────────────────────────────────────────────────────────────────────
#define BASE_SPEED      160
#define MAX_SPEED       220
#define MIN_SPEED        60
#define KP              1.5f

#define IMG_WIDTH   160
#define IMG_HEIGHT  120

// Contrast-based detection — works for dark-on-light and light-on-dark lines
#define CONTRAST_THRESHOLD  25   // |pixel - rowMean| must exceed this to count
#define MIN_LINE_PX          4   // Narrowest plausible line (pixels)
#define MAX_LINE_PX        120   // Widest plausible line (75 % of frame width)

#define LOST_TIMEOUT_MS  800

// ─────────────────────────────────────────────────────────────────────────────

bool setupCamera() {
    camera_config_t cfg;
    cfg.ledc_channel  = LEDC_CHANNEL_2;  // Must not clash with motor channels 0,1
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
    cfg.frame_size    = FRAMESIZE_QQVGA;   // 160×120
    cfg.jpeg_quality  = 12;
    cfg.fb_count      = 2;                 // Double buffer for smoother capture

    if (esp_camera_init(&cfg) != ESP_OK) {
        Serial.println("Camera init failed");
        return false;
    }
    Serial.println("Camera OK");
    return true;
}

void setupMotors() {
    // Explicitly assign channels so they never collide with the camera's channel 2
    ledcAttachChannel(ENA_PIN, PWM_FREQ, PWM_RES, ENA_CHAN);
    ledcAttachChannel(ENB_PIN, PWM_FREQ, PWM_RES, ENB_CHAN);

    pinMode(IN1_PIN, OUTPUT);
    pinMode(IN2_PIN, OUTPUT);
    pinMode(IN3_PIN, OUTPUT);
    pinMode(IN4_PIN, OUTPUT);

    stopMotors();
}

void driveLeft(int speed) {
    if (speed >= 0) { digitalWrite(IN1_PIN, HIGH); digitalWrite(IN2_PIN, LOW); }
    else            { digitalWrite(IN1_PIN, LOW);  digitalWrite(IN2_PIN, HIGH); speed = -speed; }
    ledcWrite(ENA_PIN, constrain(speed, 0, 255));
}

void driveRight(int speed) {
    if (speed >= 0) { digitalWrite(IN3_PIN, HIGH); digitalWrite(IN4_PIN, LOW); }
    else            { digitalWrite(IN3_PIN, LOW);  digitalWrite(IN4_PIN, HIGH); speed = -speed; }
    ledcWrite(ENB_PIN, constrain(speed, 0, 255));
}

void stopMotors() {
    digitalWrite(IN1_PIN, LOW); digitalWrite(IN2_PIN, LOW);
    digitalWrite(IN3_PIN, LOW); digitalWrite(IN4_PIN, LOW);
    ledcWrite(ENA_PIN, 0);
    ledcWrite(ENB_PIN, 0);
}

/*
 * Scan a single row using a minority-wins strategy.
 *
 * Why minority-wins: a line is always narrower than the background it sits on.
 * We split deviating pixels into two groups — those darker than the row mean
 * and those brighter — then use whichever group has fewer members.  This works
 * for both dark-on-light and light-on-dark lines and doesn't break when the
 * tape is wide (in the old approach a wide line pulled the row mean toward
 * itself, making BOTH the line and the background deviate, blowing past the
 * max-pixel guard and returning -1).
 *
 * Returns the weighted centroid X of the line pixels, or -1 if no line found.
 */
int scanRow(const uint8_t* pixels, int row) {
    const int offset = row * IMG_WIDTH;

    long rowSum = 0;
    for (int x = 0; x < IMG_WIDTH; x++) rowSum += pixels[offset + x];
    int rowMean = (int)(rowSum / IMG_WIDTH);

    long darkWsum = 0,  brightWsum = 0;
    int  darkCnt  = 0,  brightCnt  = 0;

    for (int x = 0; x < IMG_WIDTH; x++) {
        int diff = (int)pixels[offset + x] - rowMean;
        if (diff < -CONTRAST_THRESHOLD) { darkWsum  += x; darkCnt++;   }
        else if (diff >  CONTRAST_THRESHOLD) { brightWsum += x; brightCnt++; }
    }

    bool darkOk   = (darkCnt   >= MIN_LINE_PX && darkCnt   <= MAX_LINE_PX);
    bool brightOk = (brightCnt >= MIN_LINE_PX && brightCnt <= MAX_LINE_PX);

    int  cnt;
    long wsum;
    if (darkOk && brightOk) {
        // Both qualify — the line is whichever is narrower
        if (darkCnt <= brightCnt) { cnt = darkCnt;   wsum = darkWsum;   }
        else                      { cnt = brightCnt; wsum = brightWsum; }
    } else if (darkOk)   { cnt = darkCnt;   wsum = darkWsum;   }
    else if (brightOk)   { cnt = brightCnt; wsum = brightWsum; }
    else                 { return -1; }

    return (int)(wsum / cnt);
}

/*
 * Scan 15 rows from bottom to top (every 5 rows across the lower two-thirds
 * of the frame).  Returns the first successful hit, or -1.
 * Bottom-first means we react to the line as early as possible before a curve.
 */
int detectLine(const uint8_t* pixels) {
    for (int row = 110; row >= 40; row -= 5) {
        int x = scanRow(pixels, row);
        if (x >= 0) return x;
    }
    return -1;
}

// ── State for "line lost" recovery ───────────────────────────────────────────
static int           lastError = 0;
static bool          lineLost  = false;
static unsigned long lostAt    = 0;

static bool cameraReady = false;

void setup() {
    Serial.begin(115200);
    delay(500);

    setupMotors();

    // Retry camera init up to 5 times before giving up
    for (int attempt = 1; attempt <= 5 && !cameraReady; attempt++) {
        Serial.printf("Camera init attempt %d/5\n", attempt);
        cameraReady = setupCamera();
        if (!cameraReady) {
            esp_camera_deinit();
            delay(500);
        }
    }

    if (!cameraReady) {
        Serial.println("Camera failed — check wiring. Motors disabled.");
    } else {
        delay(1000);
        Serial.println("Starting line follower");
    }
}

void loop() {
    if (!cameraReady) {
        delay(1000);
        return;
    }

    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("Frame capture failed");
        stopMotors();
        delay(30);
        return;
    }

    int lineX = detectLine(fb->buf);
    esp_camera_fb_return(fb);

    if (lineX < 0) {
        if (!lineLost) {
            lineLost = true;
            lostAt   = millis();
        }

        if (millis() - lostAt > LOST_TIMEOUT_MS) {
            stopMotors();
            Serial.println("Line lost — stopped");
        } else {
            int turnSpeed = BASE_SPEED / 2;
            if (lastError >= 0) { driveLeft(turnSpeed);  driveRight(-turnSpeed); }
            else                { driveLeft(-turnSpeed); driveRight(turnSpeed);  }
            Serial.println("Searching");
        }
        delay(20);
        return;
    }

    lineLost = false;

    int error      = (IMG_WIDTH / 2) - lineX;  // Negated: camera is mounted upside-down
    lastError      = error;
    int correction = (int)(KP * error);
    int leftSpeed  = constrain(BASE_SPEED + correction, MIN_SPEED, MAX_SPEED);
    int rightSpeed = constrain(BASE_SPEED - correction, MIN_SPEED, MAX_SPEED);

    driveLeft(leftSpeed);
    driveRight(rightSpeed);

    Serial.printf("lineX=%3d  err=%4d  L=%3d  R=%3d\n", lineX, error, leftSpeed, rightSpeed);

    delay(10);
}
