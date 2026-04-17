/*
 * ESP32-CAM Line Following Robot
 * Hardware: AI-Thinker ESP32-CAM + L298N Motor Driver
 *
 * L298N Wiring:
 *   ENA  -> GPIO 2   (Left motor PWM)
 *   IN1  -> GPIO 14  (Left motor forward)
 *   IN2  -> GPIO 15  (Left motor reverse)
 *   ENB  -> GPIO 4   (Right motor PWM)
 *   IN3  -> GPIO 13  (Right motor forward)
 *   IN4  -> GPIO 12  (Right motor reverse)
 *
 * Notes:
 *   - GPIO 0 must be floating/HIGH during normal boot (not used for motors)
 *   - GPIO 4 doubles as flash LED — it will flicker slightly during PWM
 *   - If PSRAM is present, avoid GPIO 16
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
#define ENB_PIN  4    // Right motor PWM
#define IN3_PIN  13   // Right motor +
#define IN4_PIN  12   // Right motor -

// ── PWM ───────────────────────────────────────────────────────────────────────
#define PWM_FREQ    1000   // Hz
#define PWM_RES     8      // bits → 0-255

// ── Tuning ────────────────────────────────────────────────────────────────────
#define BASE_SPEED      170   // Straight-line speed (0-255)
#define MAX_SPEED       220   // Speed cap to prevent slipping
#define MIN_SPEED        60   // Minimum speed on a tight curve
#define KP              1.8f  // Proportional gain  (increase → snappier turns)

#define IMG_WIDTH       160
#define IMG_HEIGHT      120
#define SCAN_ROW        90    // Pixel row to analyse (0 = top, 119 = bottom)
#define BLACK_THRESHOLD 80    // Pixels below this value are counted as black
#define MIN_BLACK_PX     8    // Fewer black pixels → treat as "no line"

// ─────────────────────────────────────────────────────────────────────────────

void setupCamera() {
    camera_config_t cfg;
    cfg.ledc_channel = LEDC_CHANNEL_2;   // Avoid conflict with motor PWM ch 0-1
    cfg.ledc_timer   = LEDC_TIMER_2;
    cfg.pin_d0       = Y2_GPIO_NUM;
    cfg.pin_d1       = Y3_GPIO_NUM;
    cfg.pin_d2       = Y4_GPIO_NUM;
    cfg.pin_d3       = Y5_GPIO_NUM;
    cfg.pin_d4       = Y6_GPIO_NUM;
    cfg.pin_d5       = Y7_GPIO_NUM;
    cfg.pin_d6       = Y8_GPIO_NUM;
    cfg.pin_d7       = Y9_GPIO_NUM;
    cfg.pin_xclk     = XCLK_GPIO_NUM;
    cfg.pin_pclk     = PCLK_GPIO_NUM;
    cfg.pin_vsync    = VSYNC_GPIO_NUM;
    cfg.pin_href     = HREF_GPIO_NUM;
    cfg.pin_sscb_sda = SIOD_GPIO_NUM;
    cfg.pin_sscb_scl = SIOC_GPIO_NUM;
    cfg.pin_pwdn     = PWDN_GPIO_NUM;
    cfg.pin_reset    = RESET_GPIO_NUM;
    cfg.xclk_freq_hz = 20000000;
    cfg.pixel_format = PIXFORMAT_GRAYSCALE;  // Fastest, no colour needed
    cfg.frame_size   = FRAMESIZE_QQVGA;      // 160×120
    cfg.jpeg_quality = 12;
    cfg.fb_count     = 1;

    if (esp_camera_init(&cfg) != ESP_OK) {
        Serial.println("Camera init failed — halting");
        while (true) delay(1000);
    }
    Serial.println("Camera OK");
}

void setupMotors() {
    ledcAttach(ENA_PIN, PWM_FREQ, PWM_RES);
    ledcAttach(ENB_PIN, PWM_FREQ, PWM_RES);

    pinMode(IN1_PIN, OUTPUT);
    pinMode(IN2_PIN, OUTPUT);
    pinMode(IN3_PIN, OUTPUT);
    pinMode(IN4_PIN, OUTPUT);

    stopMotors();
}

// speed: -255 … +255  (negative = reverse)
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
 * Scan SCAN_ROW of the grayscale frame.
 * Returns the weighted centroid X of all black pixels, or -1 if no line found.
 */
int detectLine(const uint8_t* pixels) {
    long wsum = 0;
    int  cnt  = 0;
    const int offset = SCAN_ROW * IMG_WIDTH;

    for (int x = 0; x < IMG_WIDTH; x++) {
        if (pixels[offset + x] < BLACK_THRESHOLD) {
            wsum += x;
            cnt++;
        }
    }

    return (cnt >= MIN_BLACK_PX) ? (int)(wsum / cnt) : -1;
}

// ── State for "line lost" recovery ───────────────────────────────────────────
static int  lastError      = 0;   // Remember which side the line was on
static bool lineLost       = false;
static unsigned long lostAt = 0;

#define LOST_TIMEOUT_MS  800   // Stop searching after this long

void setup() {
    Serial.begin(115200);
    setupMotors();
    setupCamera();

    // Brief pause so motors don't jolt at power-on
    delay(2000);
    Serial.println("Starting line follower");
}

void loop() {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("Frame capture failed");
        stopMotors();
        delay(50);
        return;
    }

    int lineX = detectLine(fb->buf);
    esp_camera_fb_return(fb);

    if (lineX < 0) {
        // ── Line lost ──────────────────────────────────────────────────────
        if (!lineLost) {
            lineLost = true;
            lostAt   = millis();
        }

        if (millis() - lostAt > LOST_TIMEOUT_MS) {
            stopMotors();
            Serial.println("Line lost — stopped");
        } else {
            // Spin toward the side the line was last seen on
            int turnSpeed = BASE_SPEED / 2;
            if (lastError >= 0) { driveLeft(turnSpeed);  driveRight(-turnSpeed); }  // Turn right
            else                { driveLeft(-turnSpeed); driveRight(turnSpeed);  }  // Turn left
            Serial.println("Searching…");
        }
        return;
    }

    lineLost  = false;

    // Error: positive → line is to the right, negative → line is to the left
    int error = lineX - (IMG_WIDTH / 2);
    lastError = error;

    int correction = (int)(KP * error);
    int leftSpeed  = constrain(BASE_SPEED + correction, MIN_SPEED, MAX_SPEED);
    int rightSpeed = constrain(BASE_SPEED - correction, MIN_SPEED, MAX_SPEED);

    driveLeft(leftSpeed);
    driveRight(rightSpeed);

    Serial.printf("lineX=%3d  err=%4d  L=%3d  R=%3d\n",
                  lineX, error, leftSpeed, rightSpeed);
}
