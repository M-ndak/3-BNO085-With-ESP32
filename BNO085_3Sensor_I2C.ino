/*
 * ESP32 (30-pin WROOM-32) + 3× BNO085 via I²C
 * Library: SparkFun_BNO08x_Arduino_Library (patched)
 *
 * ── WHY I²C INSTEAD OF SPI ────────────────────────────────────────────────────
 * I²C needs only 2 shared wires (SDA/SCL) plus one RST per sensor.
 * All 3 sensors get different addresses by wiring the ADR pin differently,
 * so no chip-select lines are needed and INT can be shared or omitted entirely
 * (we poll instead — see timing discussion below).
 *
 * ── I²C ADDRESS SELECTION ────────────────────────────────────────────────────
 * The BNO085 breakout has an ADR solder jumper (or pad):
 *   ADR = 0 (pad open / GND)  → 0x4B
 *   ADR = 1 (pad bridged/3V3) → 0x4A
 * ── PIN WIRING ────────────────────────────────────────────────────────────────
 *
 *  ┌─────────────────────────────────────────────────────────────┐
 *  │  ESP32 30-pin WROOM-32                                      │
 *  │                                                             │
 *  │  3V3  ──────┬────────────────────────────┐                 │
 *  │             │      BNO085 #0              │  BNO085 #1      │
 *  │             │  VDD ─────────────          │  VDD ─────────  │
 *  │  GND  ──────┼──  GND ─────────────────── │  GND ─────────  │
 *  │             │  SDA ──── GPIO21 (Wire SDA) │  SDA ── GPIO21  │
 *  │             │  SCL ──── GPIO22 (Wire SCL) │  SCL ── GPIO22  │
 *  │             │  ADR ──── GND  → addr 0x4B │  ADR ── 3V3     │
 *  │             │  RST ──── GPIO4             │  RST ── GPIO2   │
 *  │             │  INT  (not connected)        │  INT  (NC)      │
 *  │             │  PS0 ──── GND  (I²C mode)   │  PS0 ── GND     │
 *  │             │  PS1 ──── GND  (I²C mode)   │  PS1 ── GND     │
 *  │             │                              └─────────────    │
 *  │             │      BNO085 #2  (on Wire1)                    │
 *  │             └──  VDD ─────────────                          │
 *  │  GND  ───────── GND ─────────────                          │
 *  │  GPIO33 ──────── SDA (Wire1)                               │
 *  │  GPIO32 ──────── SCL (Wire1)                               │
 *  │                 ADR ──── GND  → addr 0x4B  (different bus) │
 *  │  GPIO25 ──────── RST                                        │
 *  │                 INT  (not connected)                        │
 *  │                 PS0 ──── GND                                │
 *  │                 PS1 ──── GND                                │
 *  └─────────────────────────────────────────────────────────────┘
 *  PS0 and PS1 (protocol select):
 *    Both LOW  → I²C mode  (what we want here)
 *    Both HIGH → SPI mode
 *    Most SparkFun boards set PS0/PS1 LOW by default — check your board.
 *    If your board has solder bridges for PS0/PS1, ensure both are bridged to GND.
 *
 * ── IMPORTANT: no 5V anywhere near BNO085 ────────────────────────────────────
 *  The BNO085 is a 3.3V-only device. All signals (SDA, SCL, RST, INT) must be
 *  3.3V. The ESP32 GPIOs are 3.3V so this is fine as long as you do not
 *  accidentally connect any 5V Arduino pin or external signal.
 */

#include <Arduino.h>
#include <Wire.h>
#include "SparkFun_BNO08x_Arduino_Library.h"

// ── I²C bus configuration ────────────────────────────────────────────────────
// Wire  handles IMU 0 and IMU 1 (different I²C addresses on the same bus)
// Wire1 handles IMU 2 (separate bus, address reused as 0x4A)
#define I2C_SDA0    21       // Wire SDA
#define I2C_SCL0    22       // Wire SCL
#define I2C_SDA1    33       // Wire1 SDA
#define I2C_SCL1    32       // Wire1 SCL
#define I2C_FREQ    100000UL // 100 kHz 

// ── Sensor I²C addresses ─────────────────────────────────────────────────────
// IMU 0: ADR pin → GND  → 0x4A  on Wire
// IMU 1: ADR pin → 3V3  → 0x4B  on Wire
// IMU 2: ADR pin → GND  → 0x4A  on Wire1 (separate bus, address reuse is fine)
static const uint8_t I2C_ADDR[3] = { 0x4B, 0x4A, 0x4B };

// ── RST pins (one per sensor, INT not connected — polling mode) ───────────────
static const int8_t RST_PIN[3]   = { -1, -1, -1 };

// ── Timing constants ──────────────────────────────────────────────────────────
#define NUM_IMUS        3
#define REPORT_MS       10       // 10ms = 100Hz
#define RETRY_MS        8000UL   // retry a dead sensor every 8s
#define INIT_GAP_MS     500      // gap between sequential sensor inits

// ── Sensor objects ────────────────────────────────────────────────────────────
BNO08x        imu[NUM_IMUS];
bool          imuOk[NUM_IMUS]     = { false, false, false };
unsigned long lastRetry[NUM_IMUS] = { 0, 0, 0 };

// ── Helper: which Wire bus does sensor i use? ─────────────────────────────────
static TwoWire &busFor(int i) {
    return (i < 2) ? Wire : Wire1;
}

// ── Initialise one sensor ─────────────────────────────────────────────────────
bool initSensor(int idx) {
    Serial.printf("[IMU %d] Init addr=0x%02X RST=%d bus=%s ... ",
                  idx, I2C_ADDR[idx], RST_PIN[idx],
                  (idx < 2) ? "Wire" : "Wire1");

    if (!imu[idx].begin(I2C_ADDR[idx],   // I²C address
                        busFor(idx),       // TwoWire instance
                        -1,               // INT pin: -1 = polling mode
                        RST_PIN[idx])) {  // RST pin
        Serial.println("FAILED (begin)");
        return false;
    }

    bool ok = true;

    if (!imu[idx].enableRotationVector(REPORT_MS)) {
        Serial.printf("[IMU %d] enableRotationVector FAILED\n", idx);
        ok = false;
    }
    if (!imu[idx].enableAccelerometer(REPORT_MS)) {
        Serial.printf("[IMU %d] enableAccelerometer FAILED (non-fatal)\n", idx);
    }
    if (!imu[idx].enableGyro(REPORT_MS)) {
        Serial.printf("[IMU %d] enableGyro FAILED (non-fatal)\n", idx);
    }

    if (ok) Serial.println("OK");
    return ok;
}

// ── setup() ───────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("\n=== ESP32 + 3× BNO085 I²C ===\n");

    // ── Drive all RST pins HIGH before starting I²C ───────────────────────────
    // This ensures no sensor is held in reset while the bus is being configured.
    // The library's begin() will also do this, but doing it here first means
    // the sensors are already running when Wire.begin() is called.
    for (int i = 0; i < NUM_IMUS; i++) {
        pinMode(RST_PIN[i], OUTPUT);
        digitalWrite(RST_PIN[i], HIGH);
    }

    // ── Start I²C buses ───────────────────────────────────────────────────────
    Wire.begin(I2C_SDA0, I2C_SCL0, I2C_FREQ);
    Wire1.begin(I2C_SDA1, I2C_SCL1, I2C_FREQ);

    // Raise the clock-stretch timeout on both buses.
    // The BNO085 can stretch the I²C clock during sensor fusion updates.
    // The ESP32 default timeout (~50ms) is sometimes too short, causing the
    // driver to reset the bus mid-packet. 200ms is safe for all BNO085 modes.
    Wire.setTimeOut(200);
    Wire1.setTimeOut(200);

    // ── Wait for all sensors to fully boot from power-on ─────────────────────
    // BNO085 needs ≥300ms from VDD stable to first I²C response.
    // We use 2000ms to cover slow 3V3 rail rise and simultaneous 3-sensor boot.
    Serial.println("Waiting 2000ms for sensors to boot...");
    delay(2000);

    // ── Init sensors one at a time with a gap between each ───────────────────
    // Sequential init prevents RST pulses from overlapping on the shared bus,
    // which would cause one sensor coming out of reset to glitch the SDA line
    // and corrupt the other sensor's active transaction.
    for (int i = 0; i < NUM_IMUS; i++) {
        imuOk[i] = initSensor(i);
        if (i < NUM_IMUS - 1) {
            Serial.printf("  (waiting %dms before next sensor)\n", INIT_GAP_MS);
            delay(INIT_GAP_MS);
        }
    }

    int alive = 0;
    for (int i = 0; i < NUM_IMUS; i++) alive += imuOk[i];
    Serial.printf("\n%d / %d IMUs online\n\n", alive, NUM_IMUS);
}

// ── loop() ────────────────────────────────────────────────────────────────────
void loop() {
    unsigned long now = millis();

    for (int i = 0; i < NUM_IMUS; i++) {

        // Retry offline sensors periodically
        if (!imuOk[i]) {
            if (now - lastRetry[i] >= RETRY_MS) {
                lastRetry[i] = now;
                Serial.printf("[IMU %d] Retrying...\n", i);
                delay(100);
                imuOk[i] = initSensor(i);
            }
            Serial.printf("IMU%d:OFFLINE\t\t\t", i);
            continue;
        }

        // Poll the sensor for a new event
        if (imu[i].getSensorEvent()) {

            // Sensor rebooted — re-enable all reports
            if (imu[i].wasReset()) {
                Serial.printf("[IMU %d] Reset detected — re-enabling reports\n", i);
                imu[i].enableRotationVector(REPORT_MS);
                imu[i].enableAccelerometer(REPORT_MS);
                imu[i].enableGyro(REPORT_MS);
            }

            switch (imu[i].getSensorEventID()) {

                case SENSOR_REPORTID_ROTATION_VECTOR:
                    Serial.printf("IMU%d  roll:%7.2f  pitch:%7.2f  yaw:%7.2f\t",
                        i,
                        imu[i].getRoll()  * RAD_TO_DEG,
                        imu[i].getPitch() * RAD_TO_DEG,
                        imu[i].getYaw()   * RAD_TO_DEG);
                    break;

                case SENSOR_REPORTID_ACCELEROMETER:
                    Serial.printf("IMU%d  ax:%6.2f  ay:%6.2f  az:%6.2f\t",
                        i,
                        imu[i].getAccelX(),
                        imu[i].getAccelY(),
                        imu[i].getAccelZ());
                    break;

                case SENSOR_REPORTID_GYROSCOPE_CALIBRATED:
                    Serial.printf("IMU%d  gx:%6.3f  gy:%6.3f  gz:%6.3f\t",
                        i,
                        imu[i].getGyroX(),
                        imu[i].getGyroY(),
                        imu[i].getGyroZ());
                    break;

                default:
                    break;
            }
        } else {
            Serial.printf("IMU%d:(waiting)\t\t\t", i);
        }
    }

    Serial.println();
    delay(10); // 10ms polling loop — matches the 10ms report interval 
