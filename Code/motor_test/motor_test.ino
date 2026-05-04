// ESP32-CAM Motor Test — forward, backward, spin left, spin right
// Flash LED (GPIO4) blinks to indicate each phase:
//   1 blink = forward, 2 = backward, 3 = spin left, 4 = spin right

#define LEFT_PWM   2
#define LEFT_FWD  14
#define LEFT_REV  15
#define RIGHT_PWM  3
#define RIGHT_FWD 13
#define RIGHT_REV 12
#define FLASH_LED  4

#define LEFT_CH   0
#define RIGHT_CH  1
#define FREQ    1000
#define RES        8
#define SPD      180

void setLeft(int spd) {
    if      (spd > 0) { digitalWrite(LEFT_FWD, HIGH); digitalWrite(LEFT_REV, LOW);  }
    else if (spd < 0) { digitalWrite(LEFT_FWD, LOW);  digitalWrite(LEFT_REV, HIGH); spd = -spd; }
    else              { digitalWrite(LEFT_FWD, LOW);   digitalWrite(LEFT_REV, LOW);  }
    ledcWrite(LEFT_PWM, constrain(spd, 0, 255));
}

void setRight(int spd) {
    if      (spd > 0) { digitalWrite(RIGHT_FWD, HIGH); digitalWrite(RIGHT_REV, LOW);  }
    else if (spd < 0) { digitalWrite(RIGHT_FWD, LOW);  digitalWrite(RIGHT_REV, HIGH); spd = -spd; }
    else              { digitalWrite(RIGHT_FWD, LOW);   digitalWrite(RIGHT_REV, LOW);  }
    ledcWrite(RIGHT_PWM, constrain(spd, 0, 255));
}

void stopAll() {
    setLeft(0);
    setRight(0);
}

void blink(int n) {
    for (int i = 0; i < n; i++) {
        digitalWrite(FLASH_LED, HIGH); delay(100);
        digitalWrite(FLASH_LED, LOW);  delay(100);
    }
}

void setup() {
    // Init then immediately end Serial so UART0 releases GPIO3 (RIGHT_PWM).
    // Without this the framework holds GPIO3 as U0RXD and LEDC cannot attach to it.
    Serial.begin(115200);
    delay(50);
    Serial.end();

    ledcAttachChannel(LEFT_PWM,  FREQ, RES, LEFT_CH);
    ledcAttachChannel(RIGHT_PWM, FREQ, RES, RIGHT_CH);

    pinMode(LEFT_FWD,  OUTPUT);
    pinMode(LEFT_REV,  OUTPUT);
    pinMode(RIGHT_FWD, OUTPUT);
    pinMode(RIGHT_REV, OUTPUT);
    pinMode(FLASH_LED, OUTPUT);

    stopAll();
    delay(2000);
}

void loop() {
    blink(1);
    setLeft(SPD); setRight(SPD);   // forward
    delay(1500);
    stopAll(); delay(500);

    blink(2);
    setLeft(-SPD); setRight(-SPD); // backward
    delay(1500);
    stopAll(); delay(500);

    blink(3);
    setLeft(-SPD); setRight(SPD);  // spin left
    delay(1000);
    stopAll(); delay(500);

    blink(4);
    setLeft(SPD); setRight(-SPD);  // spin right
    delay(1000);
    stopAll(); delay(1000);
}
