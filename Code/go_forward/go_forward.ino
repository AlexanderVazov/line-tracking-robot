// ESP32-CAM Motor Test — drives straight forward at full BASE_SPEED

#define ENA_PIN  2    // Left  motor PWM
#define IN1_PIN  14   // Left  motor +
#define IN2_PIN  15   // Left  motor -
#define ENB_PIN  3    // Right motor PWM (U0RXD — Serial monitor unavailable when in use)
#define IN3_PIN  13   // Right motor +
#define IN4_PIN  12   // Right motor -

#define ENA_CHAN    0
#define ENB_CHAN    1
#define PWM_FREQ    1000
#define PWM_RES     8
#define BASE_SPEED  170

void setup() {

    // Explicit channels prevent auto-assignment from mapping both pins to the same channel
    ledcAttachChannel(ENA_PIN, PWM_FREQ, PWM_RES, ENA_CHAN);
    ledcAttachChannel(ENB_PIN, PWM_FREQ, PWM_RES, ENB_CHAN);

    pinMode(IN1_PIN, OUTPUT);
    pinMode(IN2_PIN, OUTPUT);
    pinMode(IN3_PIN, OUTPUT);
    pinMode(IN4_PIN, OUTPUT);

    delay(2000);

    digitalWrite(IN1_PIN, HIGH); digitalWrite(IN2_PIN, LOW);
    digitalWrite(IN3_PIN, HIGH); digitalWrite(IN4_PIN, LOW);
    ledcWrite(ENA_PIN, BASE_SPEED);
    ledcWrite(ENB_PIN, BASE_SPEED);
}

void loop() {}
