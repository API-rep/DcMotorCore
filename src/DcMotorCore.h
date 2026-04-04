/******************************************************************************
 * @file DcMotorCore.h
 * @brief Universal DC motor controller interface using PWM.
 * 
 * Provides a high-level abstraction for H-Bridge motor drivers, supporting 
 * Speed/Dir and Phase/Enable control modes. While optimized for hardware 
 * PWM (LEDC), the architecture is designed for cross-platform compatibility.
 * 
 * ****************************************************************************/

#pragma once

#include <Arduino.h>
#include <optional>
#include <memory>
#include <PwmControl.h>
#include <pin_defs.h> // Shared definitions library

// =============================================================================
// 1. ARCHITECTURE AND DEBUG CONFIGURATION
// =============================================================================

	// --- 1. Architecture Guard ---
	// Currently restricted to Esp32 due to LEDC driver dependency.
#if !defined(ARDUINO_ARCH_ESP32) && !defined(ESP32)
	#error "DcMotorCore currently requires Esp32 LEDC hardware PWM."
#endif

	// --- 2. Debug Macros ---
// #define DC_MOTOR_CORE_DEBUG
#ifdef DC_MOTOR_CORE_DEBUG
	#define DPRINT(...)    Serial.print(__VA_ARGS__)
	#define DPRINTLN(...)  Serial.println(__VA_ARGS__)
#else
	#define DPRINT(...)
	#define DPRINTLN(...)
#endif

// =============================================================================
// 2. CONSTANTS AND ENUMERATIONS
// =============================================================================

	/**
	 * @brief Motor driver control interface type
	 */
enum class DriverControlMode : uint8_t {
	SpeedDir = 0,    // PWM on Speed, Logic level on Dir
	PhaseEnable = 1  // PWM on Enable, Logic level on Phase
};

// =============================================================================
// 3. DC_MOTOR_CORE CLASS DEFINITION
// =============================================================================

class DcMotorCore {
public:
	static constexpr uint32_t DefaultPwmFreq = 400;    ///< Default PWM frequency in Hz.
	static constexpr float    MinSpeed       = 0.0f;   ///< Minimum (zero) speed, in percent.
	static constexpr float    MaxSpeed       = 100.0f; ///< Maximum speed, in percent.

	DcMotorCore();
	~DcMotorCore();

	// --- 1. Hardware Assignment ---
	bool useTimer(int8_t timer);                                               ///< Hint the broker to prefer this LEDC timer; must be called before attach().
	bool useChannel(int8_t channel);                                           ///< Hint the broker to prefer this LEDC channel; must be called before attach().
	bool setPwmFreq(uint32_t frequency);                                       ///< Set PWM frequency in Hz; must be called before attach().
	bool attach(uint8_t pwmPin, std::optional<int8_t> dirPin = std::nullopt);  ///< Attach motor to hardware; allocates LEDC resource via PwmBroker.

	// --- 2. Driver Configuration ---
	bool setEnablePin(uint8_t enablePin, ActiveLevel mode);                        ///< Configure the enable pin and its active level; must be before attach().
	bool setDecayPin(uint8_t decayPin, DecayMode lowState, DecayMode highState);   ///< Configure the decay pin and its low/high state mapping; must be before attach().
	bool setSleepPin(uint8_t sleepPin, ActiveLevel mode);                          ///< Configure the sleep pin and its active level; must be before attach().
	bool setMargin(float minMargin = MinSpeed, float maxMargin = MaxSpeed);        ///< Set speed dead-zone margins in percent; must be before attach().
	void setInverted(bool invert);                                                 ///< Invert rotation direction relative to CEI convention; must be before attach().

	// --- 3. Motion Control ---
	bool runAtSpeed(float speed);                  ///< Set motor speed immediately (-100.0 to 100.0).
	bool accelToSpeed(float speed, uint32_t accel); ///< Ramp motor speed using hardware PWM fade.
	void stop();                                   ///< Stop the motor immediately (sets speed to 0).

	// --- 4. Power and State Management ---
	bool enable();                  ///< Assert the enable pin (driver active).
	bool disable();                 ///< De-assert the enable pin (driver inactive).
	bool sleep();                   ///< Assert the sleep pin (driver in low-power state).
	bool wakeup();                  ///< De-assert the sleep pin (driver active again).
	bool decayMode(DecayMode mode); ///< Set the decay mode via the decay pin.

	// --- 5. Telemetry and Getters ---
	float      getSpeed();         ///< Return current speed with direction (+CW / -CCW).
	bool       isMoving();         ///< Return true if current speed is non-zero.
	bool       isSleeping();       ///< Return true if the sleep pin is asserted.
	DecayMode  getDecayMode();     ///< Return the current decay mode from the physical pin state.
	int8_t     getPwmTimer();      ///< Return the LEDC timer index, or -1 if not attached.
	uint32_t   getPwmFreq();       ///< Return PWM frequency in Hz, or 0xFFFFFFFF if not attached.
	uint32_t   getMaxDutyVal();    ///< Return maximum PWM duty cycle value based on resolution.
	bool       isAttached() const { return _pwmControl != nullptr; } ///< Return true if the PWM resource is acquired and active.

private:
	// --- 1. Instance Hardware Config ---
	std::unique_ptr<PwmControl> _pwmControl = nullptr;
	uint32_t 							 _pwmFreq = DefaultPwmFreq;
	std::optional<int8_t>  _dirPin;
	std::optional<int8_t>  _enablePin;
	std::optional<int8_t>  _decayPin;
	std::optional<int8_t>  _sleepPin;

	// --- 2. Active Levels (Polarity) ---
	ActiveLevel _enablePinMode    = ActiveLevel::ActiveHigh;
	ActiveLevel _sleepPinMode     = ActiveLevel::ActiveHigh;
	DecayMode   _decayLowPinMode  = DecayMode::Unset; // Ce qui active le freinage
	DecayMode   _decayHighPinMode = DecayMode::Unset;  // Ce qui active la roue libre

	// --- 3. Motion Properties ---
	bool 				_isInverted 		= false;    // Default: CEI standard (Positive = CW)
	float       _minMargin      = MinSpeed;
	float       _maxMargin      = MaxSpeed;
	bool        _marginAreSet   = false;
	uint32_t    _accel_factor   = 0;
	

	// --- 4. Internal PWM Properties ---
	int8_t      _pwmTimer       = -1;
	int8_t      _pwmChannel     = -1;
	uint32_t    _pwmMaxDuty     = 0;

	// --- 5. Internal Helpers ---
	bool     accelIsValid(uint32_t accel);
	bool     speedIsValid(float speed);
	uint32_t speedToDuty(float speed);
	float    dutyToSpeed(uint32_t duty);
	float    speedInMargin(float speed);
	float    revertMargedSpeed(float speed);
	bool     dirPinFromSpeed(float speed);

	// --- 6. Input parameters validations ---
	bool isSafeOutput(uint8_t pin);
	bool isValidActiveLevel(ActiveLevel mode);
};

// EOF DcMotorCore.h
