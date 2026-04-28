// ============================================================
// fall_detection.ino
// ESP32-C3 + MPU6500 + Buzzer + Cancel Button + Slide Switch
// Vibration motor wired but disabled until Phase 3
// ============================================================

#include <Wire.h>
#include "mpu6500.h"
#include "preprocessing.h"
#include "inference.h"
#include "alert.h"

// ── Pins ─────────────────────────────────────────────────────
#define PIN_SDA         8    // MPU6500 SDA
#define PIN_SCL         9    // MPU6500 SCL
#define PIN_BUZZER      5    // SmartElex buzzer IN
#define PIN_CANCEL      4    // 12mm round cancel button
#define PIN_SLIDE_SW    2    // SPDT slide switch NO contact
// #define PIN_MOTOR    6    // Vibration motor — enable Phase 3

// ── Constants ────────────────────────────────────────────────
#define SAMPLE_MS       10      // 100 Hz
#define WINDOW_SIZE     300     // 3 seconds
#define STEP_SIZE       150     // inference every 1.5s
#define CONSEC_NEEDED   3       // 3 consecutive falls → alert
#define ALERT_CANCEL_MS 15000   // 15s cancel window

// ── IMU circular buffer ──────────────────────────────────────
float    imu_buf[WINDOW_SIZE][8];
int      buf_head   = 0;
int      fill_count = 0;

// ── State ────────────────────────────────────────────────────
int      fall_streak  = 0;
bool     alert_active = false;
uint32_t alert_start  = 0;
int      step_ctr     = 0;

// ─────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(800);

    Serial.println("\n==============================");
    Serial.println("   Fall Detection System");
    Serial.println("==============================");

    // I2C
    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(400000);

    // Slide switch check — if OFF do not proceed
    pinMode(PIN_SLIDE_SW, INPUT_PULLUP);
    if (digitalRead(PIN_SLIDE_SW) == HIGH) {
        Serial.println("⚠ Slide switch is OFF — flip to ON");
        while (digitalRead(PIN_SLIDE_SW) == HIGH) delay(200);
        Serial.println("  Switch ON — starting...");
    }

    // MPU6500
    if (!mpu_init()) {
        Serial.println("❌ MPU6500 not found!");
        Serial.println("   Check: SDA=GPIO8 SCL=GPIO9 VCC=3.3V AD0=GND");
        while (true) delay(1000);
    }
    Serial.println("✅ MPU6500 ready @ 100Hz");

    // TFLite model
    if (!inference_init()) {
        Serial.println("❌ Model load failed");
        Serial.println("   Check model.h is in sketch folder");
        Serial.println("   Try increasing ARENA_KB in inference.h");
        while (true) delay(1000);
    }
    Serial.println("✅ TFLite model loaded");

    // Alert + cancel button
    alert_init(PIN_BUZZER);
    pinMode(PIN_CANCEL, INPUT_PULLUP);  // active LOW

    Serial.println("✅ System ready — monitoring...\n");
    Serial.println("prob    | pred | streak | state");
    Serial.println("--------|------|--------|------");
}

// ─────────────────────────────────────────────────────────────
void loop() {
    uint32_t t0 = millis();

    // ── Slide switch OFF → pause system ─────────────────────
    if (digitalRead(PIN_SLIDE_SW) == HIGH) {
        Serial.println("[paused — slide switch OFF]");
        delay(500);
        return;
    }

    // ── 1. Read IMU ─────────────────────────────────────────
    float ax, ay, az, gx, gy, gz;
    mpu_read(&ax, &ay, &az, &gx, &gy, &gz);

    // ── 2. Magnitude features ────────────────────────────────
    float acc_mag  = sqrtf(ax*ax + ay*ay + az*az);
    float gyro_mag = sqrtf(gx*gx + gy*gy + gz*gz);

    // ── 3. Fill circular buffer ──────────────────────────────
    imu_buf[buf_head][0] = ax;
    imu_buf[buf_head][1] = ay;
    imu_buf[buf_head][2] = az;
    imu_buf[buf_head][3] = gx;
    imu_buf[buf_head][4] = gy;
    imu_buf[buf_head][5] = gz;
    imu_buf[buf_head][6] = acc_mag;
    imu_buf[buf_head][7] = gyro_mag;
    buf_head = (buf_head + 1) % WINDOW_SIZE;
    if (fill_count < WINDOW_SIZE) fill_count++;

    // ── 4. Run inference every STEP_SIZE samples ─────────────
    step_ctr++;
    if (fill_count >= WINDOW_SIZE && step_ctr >= STEP_SIZE) {
        step_ctr = 0;

        // Unroll circular buffer oldest → newest
        float window[WINDOW_SIZE][8];
        for (int i = 0; i < WINDOW_SIZE; i++) {
            int src = (buf_head + i) % WINDOW_SIZE;
            for (int ch = 0; ch < 8; ch++)
                window[i][ch] = imu_buf[src][ch];
        }

        // Filter + normalize in-place
        preprocess_window(window);

        // Run model
        float prob = run_inference(window);
        int   pred = (prob >= 0.57f) ? 1 : 0;

        if (pred == 1) fall_streak++;
        else           fall_streak = 0;

        Serial.printf("%.4f  |  %d   |   %d    | %s\n",
            prob, pred, fall_streak,
            alert_active ? "ALERT" : "OK");

        // Trigger alert after 3 consecutive falls
        if (fall_streak >= CONSEC_NEEDED && !alert_active) {
            Serial.println("\n🚨 FALL DETECTED");
            alert_active = true;
            alert_start  = millis();
            fall_streak  = 0;
            alert_fall_detected();
        }
    }

    // ── 5. Alert countdown + cancel ──────────────────────────
    if (alert_active) {
        uint32_t elapsed   = millis() - alert_start;
        uint32_t remaining = (ALERT_CANCEL_MS - elapsed) / 1000 + 1;

        // Cancel button pressed (active LOW)
        if (digitalRead(PIN_CANCEL) == LOW) {
            Serial.println("✅ Alert cancelled\n");
            alert_active = false;
            alert_cancel();
            delay(500);  // debounce
        }
        // 15s expired → emergency
        else if (elapsed >= ALERT_CANCEL_MS) {
            Serial.println("📞 EMERGENCY TRIGGERED\n");
            alert_active = false;
            alert_emergency();
        }
        // Countdown pulse
        else {
            static uint32_t last_pulse = 0;
            if (millis() - last_pulse >= 1000) {
                last_pulse = millis();
                Serial.printf("   Press cancel button — %d s remaining\n",
                              remaining);
                alert_pulse();
            }
        }
    }

    // ── 6. Maintain 100 Hz ───────────────────────────────────
    uint32_t spent = millis() - t0;
    if (spent < SAMPLE_MS) delay(SAMPLE_MS - spent);
}
