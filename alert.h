// ============================================================
// alert.h — Buzzer Alert
// Vibration motor support added in Phase 3
// ============================================================
#pragma once
#include <Arduino.h>

static int _buzz;

static void alert_init(int buzz_pin) {
    _buzz = buzz_pin;
    pinMode(_buzz, OUTPUT);
    digitalWrite(_buzz, LOW);

    // Startup test beep — confirms buzzer is wired correctly
    digitalWrite(_buzz, HIGH); delay(100);
    digitalWrite(_buzz, LOW);  delay(100);
    digitalWrite(_buzz, HIGH); delay(100);
    digitalWrite(_buzz, LOW);
    Serial.println("  Buzzer test OK");
}

// 3 fast beeps — fall just confirmed
static void alert_fall_detected() {
    for (int i = 0; i < 3; i++) {
        digitalWrite(_buzz, HIGH); delay(300);
        digitalWrite(_buzz, LOW);  delay(150);
    }
}

// 1 short beep — every second during countdown
static void alert_pulse() {
    digitalWrite(_buzz, HIGH); delay(80);
    digitalWrite(_buzz, LOW);
}

// 2 short beeps — user cancelled
static void alert_cancel() {
    digitalWrite(_buzz, HIGH); delay(60);
    digitalWrite(_buzz, LOW);  delay(80);
    digitalWrite(_buzz, HIGH); delay(60);
    digitalWrite(_buzz, LOW);
}

// SOS pattern — emergency triggered after 15s
static void alert_emergency() {
    int dot = 150, dash = 400, gap = 100, letter = 300;
    // S ...
    for (int i = 0; i < 3; i++) {
        digitalWrite(_buzz, HIGH); delay(dot);
        digitalWrite(_buzz, LOW);  delay(gap);
    }
    delay(letter);
    // O ---
    for (int i = 0; i < 3; i++) {
        digitalWrite(_buzz, HIGH); delay(dash);
        digitalWrite(_buzz, LOW);  delay(gap);
    }
    delay(letter);
    // S ...
    for (int i = 0; i < 3; i++) {
        digitalWrite(_buzz, HIGH); delay(dot);
        digitalWrite(_buzz, LOW);  delay(gap);
    }
    // Phase 3: add WiFi/BLE emergency notification here
}
