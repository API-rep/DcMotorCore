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
#include <driver/ledc.h>
#include <pin_defs.h> // Shared definitions library

// =============================================================================
// 1. ARCHITECTURE AND DEBUG CONFIGURATION
// =============================================================================

	// --- 1. Architecture Guard ---
	// Currently restricted to Esp32 due to LEDC driver dependency.
#if !defined(Esp32)
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

	// --- 1. Functional Constants ---
static constexpr uint32_t DefaultPwmFreq = 400;
static constexpr float    MinSpeed       = 0.0f;
static constexpr float    MaxSpeed       = 100.0f;


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
	DcMotorCore();

	// --- 1. Hardware Assignment ---
	bool useTimer(int8_t timer);
	bool useChannel(int8_t channel);
	bool setPwmFreq(uint32_t frequency);
	
	bool attach(uint8_t pwmPin, std::optional<int8_t> dirPin = std::nullopt);

	// --- 2. Driver Configuration ---
	bool setEnablePin(uint8_t enablePin, ActiveLevel mode);
	bool setDecayPin(uint8_t decayPin, DecayMode lowState, DecayMode highState);
	bool setSleepPin(uint8_t sleepPin, ActiveLevel mode);
	bool setMargin(float minMargin = MinSpeed, float maxMargin = MaxSpeed);
	void setInverted(bool invert);

	// --- 3. Motion Control ---
	bool runAtSpeed(float speed);
	bool accelToSpeed(float speed, uint32_t accel);
	void stop();
	
	// --- 4. Power and State Management ---
	bool enable();
	bool disable();
	bool sleep();
	bool wakeup();
	bool decayMode(DecayMode mode);

	// --- 5. Telemetry and Getters ---
	float      getSpeed();
	bool       isMoving();
	bool       isSleeping();
	DecayMode  getDecayMode();
	int8_t     getPwmTimer();
	uint32_t   getPwmFreq();
	uint32_t   getMaxDutyVal();

private:
	// --- 1. Static Resource Management (Architecture specific) ---
	static ledc_timer_config_t* _timers_config[LEDC_TIMER_MAX];
	static uint8_t              _clients_for_timer[LEDC_TIMER_MAX];
	static int8_t               _default_timer;
	static bool                 _pwm_channel_used[LEDC_CHANNEL_MAX];

	// --- 2. Instance Hardware Config ---
	ledc_channel_config_t* _ledc_channel_config = nullptr;
	uint32_t 							 _pwmFreq = DefaultPwmFreq;
	std::optional<int8_t>  _dirPin;
	std::optional<int8_t>  _enablePin;
	std::optional<int8_t>  _decayPin;
	std::optional<int8_t>  _sleepPin;

	// --- 3. Active Levels (Polarity) ---
	ActiveLevel _enablePinMode    = ActiveLevel::ActiveHigh;
	ActiveLevel _sleepPinMode     = ActiveLevel::ActiveHigh;
	DecayMode   _decayLowPinMode  = DecayMode::Unset; // Ce qui active le freinage
	DecayMode   _decayHighPinMode = DecayMode::Unset;  // Ce qui active la roue libre

	// --- 4. Motion Properties ---
	bool 				_isInverted 		= false;    // Default: CEI standard (Positive = CW)
	float       _minMargin      = MinSpeed;
	float       _maxMargin      = MaxSpeed;
	bool        _marginAreSet   = false;
	uint32_t    _accel_factor   = 0;
	

	// --- 5. Internal PWM Properties ---
	int8_t      _pwmTimer       = -1;
	int8_t      _pwmChannel     = -1;
	uint32_t    _pwmMaxDuty     = 0;

	// --- 6. Internal Helpers ---
	bool     accelIsValid(uint32_t accel);
	bool     speedIsValid(float speed);
	uint32_t speedToDuty(float speed);
	float    dutyToSpeed(uint32_t duty);
	float    speedInMargin(float speed);
	float    revertMargedSpeed(float speed);
	bool     dirPinFromSpeed(float speed);
	inline bool isAttached() const { return _ledc_channel_config != nullptr; }

	// --- 7. Input parameters validations ---
	bool isSafeOutput(uint8_t pin);
	bool isValidActiveLevel(ActiveLevel mode);

	static int  espSilentLog(const char* string, va_list args);
	static int  _logHasOccured;
};

// EOF DcMotorCore.h
