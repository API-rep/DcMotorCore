/******************************************************************************
 * @file DcMotorCore.cpp
 * @brief Implementation of the universal DC motor controller.
 ******************************************************************************/
#include "DcMotorCore.h"
#include <PwmBroker.h>

#include <Arduino.h>
#include <driver/ledc.h>

// =============================================================================
// 1. CONSTRUCTOR & INITIALIZATION
// =============================================================================

/**
 * @brief cMotorCore default constructor
 * @details Initalize motor contoller and default (safe) atributes values
 */

DcMotorCore::DcMotorCore()
{

}

DcMotorCore::~DcMotorCore() {
	if (isAttached()) {
		stop();
		_pwmControl.reset();
		_pwmTimer = -1;
		_pwmChannel = -1;
		_pwmMaxDuty = 0;
	}
}


// =============================================================================
// 2. HARDWARE ASSIGNMENT
// =============================================================================

/**
 * @brief Provide a specific PWM timer for PWM signal
 * 
 * @details Use to set a specific timer for PWM signal. Can be used once and before
 *          attach() (will be ignore otherwise). Timer of another motor can be use.
 *          Use sourceMotor.getPwmTimer() to get it.
 * 
 * @param	timer Timer to use for PWM signal (ledc : High speed timer 0-3)
 */
bool DcMotorCore::useTimer(int8_t timer) {
		// --- 1. Range validation ---
	if (timer < 0 || timer >= LEDC_TIMER_MAX) {
		DPRINTLN("DcMotorCore: Invalid timer index provided.");
		return false; // SUCRÉ: EXIT_FAILURE remplacé par false
	}

		// --- 2. Assignment ---
	_pwmTimer = timer;
	
	return true; // SUCRÉ: EXIT_SUCCESS remplacé par true
}



/**
 * @brief Provide a specific channel for PWM signal.
 * 
 * @details Used to set a specific channel. Must be called before attach(). 
 *          Useful to resolve conflicts with other libraries using LEDC.
 * 
 * @param channel Channel to use for PWM (LEDC: 0-15).
 */
bool DcMotorCore::useChannel(int8_t channel) {
		// --- 1. Range validation ---
	if (channel < 0 || channel >= LEDC_CHANNEL_MAX) {
		DPRINTLN("DcMotorCore: Invalid channel index provided.");
		return false;
	}

		// --- 2. Assignment ---
	_pwmChannel = channel;
	
	return true;
}



bool DcMotorCore::setPwmFreq(uint32_t frequency) {
		// --- 1. Lock check ---
	if (isAttached() ) {
		DPRINTLN("DcMotorCore Error: Use setPwmFreq() before attach() only. Command ignored.");
		return false;
	}

		// --- 2. Temporary storage or direct update into DefaultPwmFreq ---
	_pwmFreq = frequency; 
	return true;
}



/**
 * @brief Attach motor to pins and configure hardware PWM.
 * 
 * @param pwmPin Pin connected to motor driver PWM input.
 * @param dirPin Pin connected to motor driver direction pin (std::optional).
 * @return true if configuration success, false otherwise.
 */
bool DcMotorCore::attach(uint8_t pwmPin, std::optional<int8_t> dirPin) {
		// --- 1. Lock check ---
	if (isAttached()) return false;

		// --- 2. Interface Management ---
	_dirPin = dirPin;

	if (!_dirPin.has_value()) {
			// CASE 1: PH/EN (Locked Anti-Phase)
		DPRINTLN("DcMotorCore: PH/EN mode (Locked Anti-Phase) selected.");
	} 
	else if (_dirPin.value() == -1) {
			// CASE 2: Unidirectional
		DPRINTLN("DcMotorCore: Unidirectional mode selected.");
	} 
	else {
			// CASE 3: Standard SPEED/DIR
		if (!isSafeOutput(_dirPin.value())) return false;
		pinMode(_dirPin.value(), OUTPUT);
		DPRINT("DcMotorCore: Speed/Dir mode on GPIO "); DPRINTLN(_dirPin.value());
	}

		// --- 3. PWM Pin Validation ---
	if (!isSafeOutput(pwmPin)) return false;

		// --- 4. Resource Allocation through Broker ---
	DPRINTLN("DcMotorCore: Requesting PWM resource from Broker...");
	_pwmControl = PwmBroker::getInstance().requestResource(pwmPin, _pwmFreq, PwmModeRequest::LowSpeed);

	if (!_pwmControl) {
		DPRINTLN("    -> Error: Broker could not allocate PWM resource.");
		return false;
	}

	_pwmChannel = _pwmControl->getChannel();
	_pwmTimer = _pwmControl->getTimer();
	_pwmFreq = _pwmControl->getFrequency();
	_pwmMaxDuty = _pwmControl->getMaxDuty();

	if (_pwmChannel < 0 || _pwmTimer < 0 || _pwmMaxDuty == 0) {
		DPRINTLN("    -> Error: Broker returned an invalid PWM handle.");
		_pwmControl.reset();
		return false;
	}

		// --- 6. Finalize Instance ---
	ledc_fade_func_install(0);

	uint32_t neutralDuty = speedToDuty(MinSpeed); // MinSpeed est 0.0f
  ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, (ledc_channel_t)_pwmChannel, neutralDuty, 0);

  DPRINTLN("DcMotorCore: PWM initialized to neutral state.");

	wakeup(); 

	DPRINTLN("DcMotorCore: PWM configuration success.");
	
	return true;
}




/**
 * @brief Enable pin setup
 * 
 * @details Configure the motor driver enable pin and its active level (active high or active low).
 * 
 * @param	enablePin Pin connected to motor driver enable input
 * @param	mode Pin mode (active high or active low)
 */

bool DcMotorCore::setEnablePin(uint8_t enablePin, ActiveLevel mode) {
		DPRINTLN("DcMotorCore: Enable pin configuration:");

		// --- 1. Lock check ---
	if (isAttached()) {
		DPRINTLN("DcMotorCore Error: Cannot change Enable pin after attach().");
		return false;
	}

		// --- 2. Input parameters validation ---
	if (!isSafeOutput(enablePin) || !isValidActiveLevel(mode)) {
		DPRINTLN("    -> Invalid Enable Pin or ActiveLevel mode. Configuration aborted.");
		return false;
	}

		// --- 3. Configuration ---
	_enablePin = enablePin; // Automatically wrapped into std::optional
	_enablePinMode = mode;

	pinMode(_enablePin.value(), OUTPUT);
	DPRINT("    -> Enable pin attached to pin "); DPRINTLN(_enablePin.value());

		// --- 4. Initial state: Set to disabled for safety ---
	return disable();
}



/**
 * @brief Decay pin setup
 * 
 * @details Configure the motor driver decay pin and its active levels for HIGH and LOW states.
 * 
 * @param decayPin Pin connected to motor driver decay input
 * @param lowState Decay mode mode when pin is LOW
 * @param highState Decay mode when pin is HIGH
 */
bool DcMotorCore::setDecayPin(uint8_t decayPin, DecayMode lowState, DecayMode highState) {
		DPRINTLN("DcMotorCore: Decay pin configuration:");

		// --- 1. Lock check ---
	if (isAttached()) {
		DPRINTLN("DcMotorCore Error: Cannot change Decay pin after attach().");
		return false;
	}

		// --- 2. Safety check ---
	if (!isSafeOutput(decayPin)) {
		DPRINTLN("    -> Invalid Decay Pin. Configuration aborted.");
		return false;
	}

		// --- 3. Configuration and Assignment ---
	_decayPin = decayPin; // Automatically wrapped into std::optional
	_decayLowPinMode = lowState;
	_decayHighPinMode = highState;

	pinMode(_decayPin.value(), OUTPUT);
	DPRINT("    -> Decay pin attached to pin "); DPRINTLN(_decayPin.value());

		// --- 4. Initial state: Apply mode defined for the LOW state ---
	return decayMode(_decayLowPinMode);
}



/**
 * @brief Sleep pin setup
 * 
 * @details Configure the motor driver sleep pin and its active level (active high or active low).
 * 
 * @param	sleepPin Pin connected to motor driver sleep input
 * @param	mode Pin mode (active high or active low)
 */
bool DcMotorCore::setSleepPin(uint8_t sleepPin, ActiveLevel mode) {
		DPRINTLN("DcMotorCore: Sleep pin configuration:");

		// --- 1. Lock check ---
	if (isAttached()) {
		DPRINTLN("DcMotorCore Error: Cannot change Sleep pin after attach().");
		return false;
	}

		// --- 2. Input parameters validation ---
	if (!isSafeOutput(sleepPin) || !isValidActiveLevel(mode)) {
		DPRINTLN("    -> Invalid Sleep Pin or ActiveLevel mode. Configuration aborted.");
		return false;
	}

		// --- 3. Configuration ---
	_sleepPin = sleepPin; // Automatically wrapped into std::optional
	_sleepPinMode = mode;

	pinMode(_sleepPin.value(), OUTPUT);
	DPRINT("    -> Sleep pin attached to pin "); DPRINTLN(_sleepPin.value());

		// --- 4. Initial state: Force sleep for maximum safety ---
	return sleep();
}


/**
 * @brief Set motor margin
 * 
 * @details Configure the minimum and maximum speed limits for the motor to compensate dead zone.
 * 
 * @param	minMargin Minimum speed at which the motor starts
 * @param	maxMargin Maximum motor speed limit (in both directions)
 */

bool DcMotorCore::setMargin(float minMargin, float maxMargin) {
		DPRINTLN("DcMotorCore: Setting up margins...");

		// --- 1. Lock check ---
	if (isAttached()) {
		DPRINTLN("DcMotorCore Error: Cannot change margins after attach().");
		return false;
	}
	
		// --- 2. Validation with safety gap ---
	const float safetyGap = 10.0f;
	
	if (minMargin >= MinSpeed && 
	    maxMargin <= MaxSpeed && 
	    (maxMargin - minMargin) >= safetyGap) {

		_minMargin = minMargin;
		_maxMargin = maxMargin;

		// --- 3. Smart flag switching ---
		// If margins are at absolute limits, we disable mapping logic to save CPU
		_marginAreSet = !(_minMargin == MinSpeed && _maxMargin == MaxSpeed);

		DPRINT("    -> Margins updated. Active: "); 
		DPRINTLN(_marginAreSet ? "YES" : "NO (Default)");
		
		return true;
	}

		DPRINTLN("    -> Error: Invalid margin range or gap too small.");
	return false;
}



/**
 * @brief Invert the motor rotation logic relative to the CEI standard.
 */
void DcMotorCore::setInverted(bool invert) {
		// --- 1. Lock check ---
	if (isAttached()) {
		DPRINTLN("DcMotorCore Error: Cannot invert rotation logic after attach().");
		return;
	}

		// --- 2. Assignment ---
	_isInverted = invert;
	DPRINT("DcMotorCore: Rotation logic inverted: "); 
	DPRINTLN(_isInverted ? "YES" : "NO");
}




/**
 * @brief Set motor speed in percent (-100.0 to 100.0).
 * 
 * @param speed Target speed value.
 * @return true if speed was set, false if value was invalid or hardware error.
 */
bool DcMotorCore::runAtSpeed(float speed) {
		DPRINTLN("DcMotorCore: Setting motor speed.");

		// --- 1. Logical Inversion ---
	if (_isInverted) speed = -speed; 

		// --- 2. Validation ---
	if (speedIsValid(speed)) {
		
			// --- 3. Margin Mapping ---
			// Internal helper automatically handles the _marginAreSet logic
		float mappedSpeed = speedInMargin(speed);

			// --- 4. Direction management ---
			// Sets the physical DIR pin state if defined
		dirPinFromSpeed(mappedSpeed);

			// --- 5. PWM Duty Cycle calculation ---
			// Handles both standard and Locked Anti-Phase (PH/EN) modes
		uint32_t duty = speedToDuty(mappedSpeed);

			// --- 6. Hardware Update ---
			if (_pwmControl && _pwmControl->setDuty(duty)) {
			return true;
		}
	}

		DPRINTLN("  DcMotorCore: Speed setting aborted.");
	return false;
}



/**
 * @brief Set motor speed with hardware-assisted acceleration (Fade).
 * 
 * @details Gradually change the motor speed to the target value using harware PWM fading.
 *          Acceleration time is determined by the accel parameter in ...
 */
bool DcMotorCore::accelToSpeed(float targetSpeed, uint32_t accel) {
		DPRINTLN("DcMotorCore: Setting speed with acceleration.");

		// --- 1. Logical Inversion ---
	if (_isInverted) targetSpeed = -targetSpeed;

		// --- 1. Parameters validation ---
	if (!speedIsValid(targetSpeed) || !accelIsValid(accel)) {
		DPRINTLN("    -> Acceleration setup aborted: invalid parameters.");
		return false; 
	}


		// --- 2. Target speed preparation ---
	float mappedTarget = speedInMargin(targetSpeed);
	dirPinFromSpeed(mappedTarget);
	uint32_t targetDuty = speedToDuty(mappedTarget);


		// --- 3. Hardware state acquisition and Fade Calculation ---
		// 3.1 Capture current duty BEFORE stopping the fade
	uint32_t currentDuty = ledc_get_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)_pwmChannel);
	
		// 3.2 Stop any ongoing fade by forcing current duty immediately
	ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, (ledc_channel_t)_pwmChannel, currentDuty, 0);

		// 3.3 Calculate the absolute difference (delta)
	uint32_t deltaDuty = (currentDuty > targetDuty) ? (currentDuty - targetDuty) : (targetDuty - currentDuty);

		// 3.4 Calculate the fade time based on the acceleration factor
	uint32_t fadeTimeMs = (uint32_t)((deltaDuty * accel * 100.0f) / _pwmMaxDuty);


		// --- 4. Hardware execution ---
	esp_err_t result = ledc_set_fade_time_and_start(
		LEDC_LOW_SPEED_MODE, 
		(ledc_channel_t)_pwmChannel, 
		targetDuty, 
		fadeTimeMs, 
		LEDC_FADE_NO_WAIT
	);

	if (result == ESP_OK) {
		DPRINT("    -> Acceleration started: "); DPRINT(fadeTimeMs); DPRINTLN(" ms transition.");
		return true;
	}

	DPRINTLN("    -> Hardware error during fade initialization.");
	return false;
}



/**
 * @brief Stop motor rotation immediately.
 */

void DcMotorCore::stop() {
		DPRINTLN("DcMotorCore: Motor stopped.");

		// --- 1. Interruption of any ongoing hardware fade ---
	uint32_t currentDuty = ledc_get_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)_pwmChannel);
	ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, (ledc_channel_t)_pwmChannel, currentDuty, 0);

		// --- 2. Force speed to minimum ---
	runAtSpeed(MinSpeed);
}



/**
 * @brief Set decay mode based on hardware mapping.
 * 
 * @details This function sets the decay mode by writing to the physical pin according to the mapping defined in setDecayPin().
 *          It abstracts the user from the actual pin state required for each mode, allowing them to simply request the desired decay behavior.
 * @param mode Desired decay mode (SlowDecay for braking, FastDecay for coasting ...).
 * @return true if the decay mode was successfully set, false if there was an error (e.g., pin not defined or mode not supported by hardware).
 */

bool DcMotorCore::decayMode(DecayMode mode) {
		// --- 1. Validation ---
		// Check if pin is defined and if requested mode is functional
	if (!_decayPin.has_value() || mode == DecayMode::Unset) {
		DPRINTLN("DcMotorCore: Decay command ignored (pin not set or Unset mode).");
		return false;
	}

		// --- 2. Map intent to physical state ---
		// Compare requested mode with our defined truth table for LOW and HIGH states
	if (mode == _decayLowPinMode) {
		digitalWrite(_decayPin.value(), LOW);
	} 
	else if (mode == _decayHighPinMode) {
		digitalWrite(_decayPin.value(), HIGH);
	} 
	else {
		DPRINTLN("DcMotorCore: Requested mode not supported by this hardware.");
		return false;
	}

	DPRINT("DcMotorCore: Decay mode set to "); 
	DPRINTLN((mode == DecayMode::SlowDecay) ? "SLOW (Brake)" : "FAST (Coast)");
	
	return true;
}




/**
 * @brief Get the current decay mode by reading the physical pin state.
 */
DecayMode DcMotorCore::getDecayMode() {
		// --- 1. Safety check ---
	if (!_decayPin.has_value()) {
		return DecayMode::Unset;
	}

		// --- 2. Read physical pin and match with the mapping ---
	if (digitalRead(_decayPin.value()) == LOW) {
		return _decayLowPinMode;
	} 
	
	else {
		return _decayHighPinMode;
	}
}



/**
 * @brief Enable the motor driver output.
 */
bool DcMotorCore::enable() {
		// --- 1. Safety check ---
	if (!_enablePin.has_value()) {
		DPRINTLN("DcMotorCore: Enable pin not set, command ignored.");
		return false;
	}

		// --- 2. Set pin state based on ActiveLevel configuration ---
	digitalWrite(_enablePin.value(), (_enablePinMode == ActiveLevel::ActiveHigh) ? HIGH : LOW);
	DPRINTLN("DcMotorCore: Driver enabled.");

	return true;
}



/**
 * @brief Disable the motor driver output.
 */
bool DcMotorCore::disable() {
		// --- 1. Safety check ---
	if (!_enablePin.has_value()) {
		DPRINTLN("DcMotorCore: Disable pin not set, command ignored.");
		return false;
	}
	
		// --- 2. Set pin state to inactive ---
	digitalWrite(_enablePin.value(), (_enablePinMode == ActiveLevel::ActiveHigh) ? LOW : HIGH);
	DPRINTLN("DcMotorCore: Driver disabled.");

	return true;
}



/**
 * @brief Put the motor driver into sleep mode.
 */
bool DcMotorCore::sleep() {
		// --- 1. Safety check ---
	if (!_sleepPin.has_value()) {
		DPRINTLN("DcMotorCore: Sleep pin not set, command ignored.");
		return false;
	}

		// --- 2. Set pin state to sleep ---
	digitalWrite(_sleepPin.value(), (_sleepPinMode == ActiveLevel::ActiveHigh) ? LOW : HIGH);
	DPRINTLN("DcMotorCore: Driver is now sleeping.");

	return true;
}




/**
 * @brief Wakeup the motor driver from sleep mode.
 */
bool DcMotorCore::wakeup() {
		// --- 1. Safety check ---
	if (!_sleepPin.has_value()) {
		DPRINTLN("DcMotorCore: Wakeup pin not set, command ignored.");
		return false;
	}

		// --- 2. Set pin state to active ---
	digitalWrite(_sleepPin.value(), (_sleepPinMode == ActiveLevel::ActiveHigh) ? HIGH : LOW);
	DPRINTLN("DcMotorCore: Driver is now awake.");

	return true;
}



/**
 * @brief Return the current speed with direction (+ for CW, - for CCW).
 */
float DcMotorCore::getSpeed() {
		// --- 1. Safety check ---
	if (!isAttached() || _pwmTimer == -1) {
		DPRINTLN("DcMotorCore Error: getSpeed() called before attach() or timer init.");
		return 0.0f;
	}

		// --- 2. Get raw speed from hardware duty cycle ---
		// Direct hardware query to get current PWM duty
	uint32_t currentDuty = _pwmControl->getDuty();
	float speed = dutyToSpeed(currentDuty);
	
	DPRINT("DcMotorCore: Raw speed from duty cycle: "); DPRINTLN(speed);

		// --- 3. Revert margin mapping ---
		// Handles dead-zone compensation logic internally
	speed = revertMargedSpeed(speed);

		// --- 4. Determine direction and apply inversion flag ---
		// For PH/EN mode, dutyToSpeed already provides the signed speed
	if (!_dirPin.has_value() || _dirPin.value() == -1) {
		return _isInverted ? -speed : speed;
	}

		// For Speed/Dir mode, we map the physical pin state (HIGH = Positive)
	float finalSpeed = (digitalRead(_dirPin.value()) == HIGH) ? speed : -speed;

		// Apply the logical inversion flag (CEI standard vs Reversed)
	return _isInverted ? -finalSpeed : finalSpeed;
}



/**
 * @brief Check if the motor is currently moving.
 * 
 * @return true if current speed is different from MinSpeed.
 */
bool DcMotorCore::isMoving() {
	// --- 1. Velocity check ---
	// Since getSpeed() returns a float, we compare against our MinSpeed constant
	if (getSpeed() != MinSpeed) {
		return true;
	}

	// --- 2. Static state ---
	return false;
}



/**
 * @brief Check if the motor driver is currently in sleep mode.
 * 
 * @return true if sleep pin is active, false if not defined or inactive.
 */
bool DcMotorCore::isSleeping() {
		// --- 1. Safety check ---
	if (!_sleepPin.has_value()) {
		return false;
	}

		// --- 2. Logic check ---
	uint8_t sleepState = (_sleepPinMode == ActiveLevel::ActiveHigh) ? LOW : HIGH;

	return (digitalRead(_sleepPin.value()) == sleepState);
}



/**
 * @brief Return the PWM timer index (0-3) used by the motor.
 * 
 * @return Timer index or -1 if not yet configured.
 */
int8_t DcMotorCore::getPwmTimer() {
		// --- 1. Check hardware configuration state ---
	if (isAttached()) {
		return _pwmTimer;
	}

		// --- 2. Return -1 if not configured ---
	return -1;
}



/**
 * @brief Return the frequency of the PWM signal in Hz.
 * 
 * @return Frequency or 0xFFFFFFFF if not configured.
 */
uint32_t DcMotorCore::getPwmFreq() {
		// --- 1. Check if PWM is configured ---
	if (isAttached()) {
		return _pwmControl->getFrequency();
	}

		// --- 2. Return error code if not configured ---
	return 0xFFFFFFFF;
}




/**
 * @brief Return the maximum value of PWM duty cycle based on resolution.
 */
uint32_t DcMotorCore::getMaxDutyVal() {
		// --- 1. Check if PWM is configured ---
	if (isAttached()) {
		return _pwmControl->getMaxDuty();
	}

		// --- 2. Return error code if not configured ---
	return 0xFFFFFFFF;
}



/**
 * @brief Verify if the acceleration factor will cause a mathematical overflow.
 */
bool DcMotorCore::accelIsValid(uint32_t accel) {
		DPRINTLN("DcMotorCore: Checking acceleration factor.");

		// --- 1. Constant for uint32_t maximum value ---
	const uint32_t uint32Max = std::numeric_limits<uint32_t>::max();

		// --- 2. Overflow protection check ---
		// Formula check: (max_duty * accel * 100) must be within uint32_t range
	if ((_pwmMaxDuty * accel * 100.0f) < (float)uint32Max) {
		return true;
	}

		DPRINT("    -> Error: Acceleration factor too large. Maximum allowed: "); 
		DPRINTLN(uint32Max / (100 * _pwmMaxDuty));

	return false;
}



/**
 * @brief Verify if the provided speed is within authorized bounds.
 */
bool DcMotorCore::speedIsValid(float speed) {
		// --- 1. General range validation (-100.0 to 100.0) ---
	if (speed < -MaxSpeed || speed > MaxSpeed) {
		DPRINT("DcMotorCore: Speed out of bounds: "); DPRINTLN(speed);
		return false;
	}

		// --- 2. Unidirectional safety check ---
		// If _dirPin is explicitly set to -1, we only allow positive speeds or zero.
	if (_dirPin.has_value() && _dirPin.value() == -1 && speed < MinSpeed) {
		DPRINTLN("DcMotorCore: Negative speed rejected for unidirectional motor.");
		return false;
	}

		// --- 3. Speed is valid for current hardware configuration ---
	return true;
}




/**
 * @brief Convert normalized speed (-100.0 to 100.0) to PWM duty cycle.
 */
uint32_t DcMotorCore::speedToDuty(float speed) {
		// --- Case 1: Phase/Enable mode (Locked Anti-Phase) ---
		// Logic: No direction pin defined. 0% speed is mapped to 50% duty cycle.
		// Formula: duty = max_duty * ((speed + 100) / 200)
	if (!_dirPin.has_value()) {
		uint32_t duty = (uint32_t)round(_pwmMaxDuty * ((speed + MaxSpeed) / (MaxSpeed * 2.0f)));
		
		DPRINT("DcMotorCore: PH/EN duty calculation: "); DPRINTLN(duty);
		return duty;
	}

		// --- Case 2 & 3: Unidirectional or Speed/Dir mode ---
		// Logic: Direction is handled by a pin or ignored. PWM represents magnitude.
		// Formula: duty = max_duty * (abs(speed) / 100)
	float absSpeed = (speed < 0.0f) ? -speed : speed;
	uint32_t duty = (uint32_t)round(_pwmMaxDuty * (absSpeed / MaxSpeed));

	DPRINT("DcMotorCore: Standard duty calculation: "); DPRINTLN(duty);
	return duty;
}




/**
 * @brief Convert a hardware duty cycle value back to a normalized speed.
 */
float DcMotorCore::dutyToSpeed(uint32_t duty) {
		// --- Case 1: Locked Anti-Phase mode (PH/EN) ---
		// Logic: No direction pin defined. Duty cycle 50% maps back to 0% speed.
		// Formula: speed = (((duty * 2) - max_duty) / max_duty) * 100
	if (!_dirPin.has_value()) {
		float speed = (((((float)duty * 2.0f) - _pwmMaxDuty) / _pwmMaxDuty) * MaxSpeed);
		
		DPRINT("DcMotorCore: PH/EN speed conversion: "); DPRINTLN(speed);
		return speed;
	}

		// --- Case 2 & 3: Unidirectional or Speed/Dir mode ---
		// Logic: Duty cycle is a direct magnitude (0 to 100%).
		// Formula: speed = (duty / max_duty) * 100
	float speed = (((float)duty / _pwmMaxDuty) * MaxSpeed);

	DPRINT("DcMotorCore: Standard speed conversion: "); DPRINTLN(speed);
	return speed;
}



/**
 * @brief Maps the input speed into the configured min/max margin range.
 * 
 * @details This linear mapping ensures the motor starts at _minMargin 
 *          and reaches its limit at _maxMargin.
 * 
 * @param speed Input speed from MinSpeed to MaxSpeed (or negative)
 * @return float Mapped speed ready for PWM conversion
 */

float DcMotorCore::speedInMargin(float speed) {
		// --- 1. Pass-through if no margins are set or speed is zero ---
	if (!_marginAreSet || speed == MinSpeed) {
		return speed;
	}

		// --- 2. Store direction and work with absolute value ---
	bool isNegative = (speed < 0.0f);
	float absSpeed = isNegative ? -speed : speed;

		// --- 3. Linear Mapping (float precision) ---
		// Formula: min + (input * (max - min) / MaxSpeed)
	float mapped = _minMargin + (absSpeed * (_maxMargin - _minMargin) / MaxSpeed);

		// --- 4. Safety Clamping ---
	if (mapped > _maxMargin) mapped = _maxMargin;

	return isNegative ? -mapped : mapped;
}



/**
 * @brief Reverts a physical speed (with margins) back to its normalized 0-100% value.
 * 
 * @param speed Mapped speed from PWM duty cycle
 * @return float Normalized speed (MinSpeed to MaxSpeed)
 */

float DcMotorCore::revertMargedSpeed(float speed) {
		// --- 1. Pass-through if no margins or speed is at minimum ---
	if (!_marginAreSet || speed == MinSpeed) {
		return speed;
	}

		// --- 2. Store direction and work with absolute value ---
	bool isNegative = (speed < 0.0f);
	float absSpeed = isNegative ? -speed : speed;

		// --- 3. Inverse Linear Mapping (float precision) ---
		// Formula: (absSpeed - min) * MaxSpeed / (max - min)
	float normalized = (absSpeed - _minMargin) * MaxSpeed / (_maxMargin - _minMargin);

		// --- 4. Clamping for telemetry safety ---
	if (normalized < MinSpeed) normalized = MinSpeed;
	if (normalized > MaxSpeed) normalized = MaxSpeed;

	return isNegative ? -normalized : normalized;
}



/**
 * @brief Set the physical direction pin state based on speed value.
 */
bool DcMotorCore::dirPinFromSpeed(float speed) {
		// --- 1. Check if a direction pin is defined ---
		// If nullopt (PH/EN) or -1 (Unidirectional), we exit without action
	if (!_dirPin.has_value() || _dirPin.value() == -1) {
		return false;
	}

		// --- 2. Set physical state based on CEI standard ---
		// Positive speed (CW) = HIGH, Negative speed (CCW) = LOW
	if (speed >= MinSpeed) {
		digitalWrite(_dirPin.value(), HIGH);
		DPRINTLN("    -> Direction pin set to HIGH (CW/Positive).");
	} 
	else {
		digitalWrite(_dirPin.value(), LOW);
		DPRINTLN("    -> Direction pin set to LOW (CCW/Negative).");
	}

	return true;
}



/**
 * @brief Verify if a GPIO pin is safe and capable of output.
 */
bool DcMotorCore::isSafeOutput(uint8_t pin) {
		// --- 1. Hardware capability check ---
		// Excludes input-only pins (like 34-39) based on Esp32 variant
	if (!GPIO_IS_VALID_OUTPUT_GPIO(pin)) {
		DPRINT("DcMotorCore: Pin "); DPRINT(pin); DPRINTLN(" is NOT a valid output.");
		return false;
	}

		// --- 2. SPI Flash protection ---
		// Excludes pins 6 to 11 to prevent system crash
	if (pin >= 6 && pin <= 11) {
		DPRINT("DcMotorCore: Pin "); DPRINT(pin); DPRINTLN(" is reserved for SPI Flash.");
		return false;
	}

	return true;
}




/**
 * @brief Verify if an ActiveLevel value is within valid range.
 */
bool DcMotorCore::isValidActiveLevel(ActiveLevel mode) {
		// Only allow explicit ActiveLow or ActiveHigh states
	return (mode == ActiveLevel::ActiveLow || mode == ActiveLevel::ActiveHigh);
}



/*
 * Legacy static LEDC tracking removed:
 * resource lifetime is now handled by PwmBroker + PwmControl (RAII).
 */