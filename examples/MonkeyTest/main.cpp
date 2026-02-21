#include <Arduino.h>
#include "DcMotorCore.h"

DcMotorCore motor;

const uint8_t PWM_PIN    = 22;
const uint8_t PHASE_PIN  = 23;
const uint8_t SLEEP_PIN  = 33;
const uint8_t DECAY_PIN  = 32;

enum class TestMode : uint8_t {
    Stability = 0,
    AggressiveRamp,
    SafetyLocks,
    PowerCycle
};

TestMode currentMode = TestMode::Stability;
uint32_t modeStartedAt = 0;
uint32_t lastActionAt = 0;
uint32_t modeDurationMs = 12000;

const char* modeName(TestMode mode) {
    switch (mode) {
        case TestMode::Stability: return "STABILITY";
        case TestMode::AggressiveRamp: return "AGGRESSIVE_RAMP";
        case TestMode::SafetyLocks: return "SAFETY_LOCKS";
        case TestMode::PowerCycle: return "POWER_CYCLE";
        default: return "UNKNOWN";
    }
}

void switchToMode(TestMode mode) {
    currentMode = mode;
    modeStartedAt = millis();
    lastActionAt = 0;
    Serial.println("\n================================================");
    Serial.print("[MODE] "); Serial.println(modeName(mode));
    Serial.println("================================================");
}

void nextMode() {
    uint8_t modeIndex = (uint8_t)currentMode;
    modeIndex = (modeIndex + 1) % 4;
    switchToMode((TestMode)modeIndex);
}

void runStabilityMode() {
    if (millis() - lastActionAt < 350) return;
    lastActionAt = millis();

    float target = (float)random(-1000, 1001) / 10.0f;
    bool ok = motor.runAtSpeed(target);

    Serial.print("[STAB] target="); Serial.print(target, 1);
    Serial.print(" | ok="); Serial.print(ok ? "YES" : "NO");
    Serial.print(" | read="); Serial.println(motor.getSpeed(), 1);
}

void runAggressiveRampMode() {
    if (millis() - lastActionAt < 1400) return;
    lastActionAt = millis();

    float target = (float)random(-1000, 1001) / 10.0f;
    uint32_t accel = (uint32_t)random(5, 90);
    bool rampOk = motor.accelToSpeed(target, accel);

    Serial.print("[RAMP] target="); Serial.print(target, 1);
    Serial.print(" | accel="); Serial.print(accel);
    Serial.print(" | rampOk="); Serial.println(rampOk ? "YES" : "NO");

    if (random(0, 100) < 40) {
        float interruptTarget = (float)random(-1000, 1001) / 10.0f;
        bool interruptOk = motor.runAtSpeed(interruptTarget);
        Serial.print("[RAMP-INT] target="); Serial.print(interruptTarget, 1);
        Serial.print(" | ok="); Serial.println(interruptOk ? "YES" : "NO");
    }
}

void runSafetyLocksMode() {
    if (millis() - lastActionAt < 1800) return;
    lastActionAt = millis();

    bool marginLocked = !motor.setMargin(0.0f, 100.0f);
    bool freqLocked = !motor.setPwmFreq(20000);
    bool enableLocked = !motor.setEnablePin(18, ActiveLevel::ActiveHigh);
    bool sleepLocked = !motor.setSleepPin(19, ActiveLevel::ActiveHigh);
    bool decayLocked = !motor.setDecayPin(21, DecayMode::SlowDecay, DecayMode::FastDecay);

    Serial.print("[LOCK] margin="); Serial.print(marginLocked ? "OK" : "KO");
    Serial.print(" | freq="); Serial.print(freqLocked ? "OK" : "KO");
    Serial.print(" | enable="); Serial.print(enableLocked ? "OK" : "KO");
    Serial.print(" | sleep="); Serial.print(sleepLocked ? "OK" : "KO");
    Serial.print(" | decay="); Serial.println(decayLocked ? "OK" : "KO");
}

void runPowerCycleMode() {
    if (millis() - lastActionAt < 2200) return;
    lastActionAt = millis();

    uint8_t action = (uint8_t)random(0, 5);
    switch (action) {
        case 0:
            Serial.println("[PWR] sleep()");
            motor.sleep();
            break;
        case 1:
            Serial.println("[PWR] wakeup()");
            motor.wakeup();
            break;
        case 2:
            Serial.println("[PWR] disable()");
            motor.disable();
            break;
        case 3:
            Serial.println("[PWR] enable()");
            motor.enable();
            break;
        default: {
            float target = (float)random(-1000, 1001) / 10.0f;
            Serial.print("[PWR] runAtSpeed("); Serial.print(target, 1); Serial.println(")");
            motor.runAtSpeed(target);
            break;
        }
    }
}

void runCurrentMode() {
    switch (currentMode) {
        case TestMode::Stability:
            runStabilityMode();
            break;
        case TestMode::AggressiveRamp:
            runAggressiveRampMode();
            break;
        case TestMode::SafetyLocks:
            runSafetyLocksMode();
            break;
        case TestMode::PowerCycle:
            runPowerCycleMode();
            break;
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial);
    Serial.println("\n===========================================");
    Serial.println("   CRASH TEST : MODE PHASE/ENABLE (PH/EN)  ");
    Serial.println("===========================================");

    motor.setEnablePin(PHASE_PIN, ActiveLevel::ActiveHigh);
    motor.setSleepPin(SLEEP_PIN, ActiveLevel::ActiveHigh);
    motor.setDecayPin(DECAY_PIN, DecayMode::SlowDecay, DecayMode::FastDecay);
    motor.setMargin(15.0f, 98.0f);

    if (!motor.attach(PWM_PIN, std::nullopt)) {
        Serial.println("!!! Erreur fatale : Attach impossible.");
        while (1);
    }

    Serial.println("Hardware configuré. Statut : STANDBY (50% PWM)");
    motor.enable();
    randomSeed(analogRead(0));

    switchToMode(TestMode::Stability);
}

void loop() {
    runCurrentMode();

    if (millis() - modeStartedAt > modeDurationMs) {
        nextMode();
    }

    delay(20);
}
