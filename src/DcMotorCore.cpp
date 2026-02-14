/******************************************************************************
 * @file DcMotorCore.cpp
 * @brief Implementation of the universal DC motor controller.
 ******************************************************************************/

#include "DcMotorCore.h"

#include "Arduino.h"

// #include "esp32-hal.h"
// #include "soc/soc_caps.h"
#include "driver/ledc.h"

// =============================================================================
// 1. STATIC MEMBER INITIALIZATION
// =============================================================================

ledc_timer_config_t *DcMotorCore::_timers_config[] = {NULL};	// init timers config pointers
uint8_t	DcMotorCore::_clients_for_timer[] = {0};				// init timers clients channels counter
int8_t	DcMotorCore::_default_timer = NOT_SET;
bool	DcMotorCore::_pwm_channel_used[] = {0};				    // init channels usage log array

int		DcMotorCore::_logHasOccure = NOT_SET;					// init espErr char counter attribute


// =============================================================================
// 2. CONSTRUCTOR & INITIALIZATION
// =============================================================================

/**
 * @brief cMotorCore default constructor
 * @details Initalize motor contoller and default (safe) atributes values
 */

DcMotorCore::DcMotorCore()
{
	_dirPin			   = NOT_SET;
	_enablePin     = NOT_SET;
	_enablePinMode = NOT_SET;
	_sleepPin		   = NOT_SET;
	_sleepPinMode	 = NOT_SET;
	
	_pwmTimer		   = NOT_SET;
	_pwmChannel		 = NOT_SET;
	_pwmMaxDuty		 = NOT_SET;



	_minMargin		 = MIN_SPEED;
	_maxMargin		 = MAX_SPEED;
	_marginAreSet	 = false;
}


// =============================================================================
// 3. HARDWARE ASSIGNMENT
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
	if (timer < 0 || timer >= LEDC_TIMER_MAX) return EXIT_FAILURE;
	_pwmTimer = timer;
	return EXIT_SUCCESS;
}



/**
 * @brief Provide a specific channel for PWM signal
 * 
 * @details Use to set a specific channel for PWM signal. Can be used once and before
 *          attach() (will be ignore otherwise). Usefull to force PWM channel if a 
 *          conflict with anoter library is meet.
 * 
 * @param	channel Channel to use for PWM signal (ledc : High speed timer 0-3)
 */

bool DcMotorCore::useChannel(int8_t channel) {
	if (channel < 0 || channel >= LEDC_CHANNEL_MAX) return EXIT_FAILURE;
	_pwmChannel = channel;
	return EXIT_SUCCESS;



/**
 * @brief Attach motor to pins and configure hardware PWM
 * 
 * @param	pwmPin Pin connected to motor driver PWM input
 * @param	dirPin Pin connected to motor driver direction pin
 * @param	pwmFreq Frequency of PWM output (= DEF_PWM_FREQ if not set)
 * 
 * NOTE: PWM selection had to be moved in a dedicated PWM manager lib
 */

bool DcMotorCore::attach(uint8_t pwmPin, std::optional<int8_t> dirPin, uint32_t pwmFreq) {

  	// --- 1. Interface Management ---
  _dirPin = dirPin;

  if (!_dirPin.has_value()) {
			// CASE 1: PH/EN (Locked Anti-Phase)
			// 0% speed is mapped to 50% hardware duty cycle
		DPRINTLN("DcMotorCore: PH/EN mode (Locked Anti-Phase) selected.");
  } 
  else if (_dirPin.value() == -1) {
			// CASE 2: Unidirectional
			// 0-100% speed only, no physical direction pin used
    DPRINTLN("DcMotorCore: Unidirectional mode selected.");
  } 
  else {
			// CASE 3: Standard SPEED/DIR
			// Uses GPIO n for rotational direction control
    if (!isSafeOutput(_dirPin.value())) return false;
    pinMode(_dirPin.value(), OUTPUT);
    DPRINT("DcMotorCore: Speed/Dir mode on GPIO "); DPRINTLN(_dirPin.value());
  }


  	// --- 2. PWM Pin Validation ---
  if (!isSafeOutput(pwmPin)) return false;

	
  	// --- 3. Resource Allocation (The Broker Transition) ---
  	// Currently handled via internal static arrays.

	DPRINTLN("Setting up PWM signal generator ...");
		// setup PWM timer if not previously configured before attach()
	if (_pwmTimer == NOT_SET) {																					DPRINTLN("  - PWM timer selection :");
			// use default timer if no pwmFreq is provided and if default timer is already defined ...
		if ((pwmFreq == 0) && (_default_timer != NOT_SET)) {													DPRINT("    -> No specific PWM frequency provided. Using previously configured timer "); DPRINT(_default_timer); DPRINTLN(" as default timer.");
			_pwmTimer = _default_timer;
		}
			// ... or select a free PWM timer if available.
		else {																									DPRINTLN("    -> Testing available timer");
			for (uint8_t timer = 0; timer <= LEDC_TIMER_MAX; timer++)
			{		// use first free timer ...
				if (_clients_for_timer[timer] == 0) {															DPRINT("    -> Timer "); DPRINT(timer); DPRINTLN(" free. Using it for PWM signal.");
					_pwmTimer = timer;
					
					break;
				}
					// ... or exit with error if no one are available
				if (timer == (LEDC_TIMER_MAX - 1)) {															DPRINTLN("    -> No suitable free timer available. Abording configuration.");
					return EXIT_FAILURE;
				}
			}
			
				// define default pwm_timer if not already set
			if (_default_timer == NOT_SET) {
					// set PWM frequency to default value if not provided
				if (!pwmFreq) {																					DPRINTLN("    -> No frequency provided to configure timer. Using default value.");
					pwmFreq = DEF_PWM_FREQ;
				}
					// set this timer as default timer
				_default_timer = _pwmTimer;																		DPRINT("    -> Using this timer as default timer with "); DPRINT(pwmFreq); DPRINTLN(" Hz base frequency.");
			}
		}
		
			// create PWM timer config structure if not yet set
		if (_timers_config[_pwmTimer] == NULL) {																DPRINT("  - Setting up timer "); DPRINT(_pwmTimer); DPRINTLN(" configuration.");
			_timers_config[_pwmTimer] = new ledc_timer_config_t;				// create timer config structure
			
				// seeding timer config parameters 
			_timers_config[_pwmTimer]->speed_mode	= LEDC_LOW_SPEED_MODE;		// set timer mode
			_timers_config[_pwmTimer]->freq_hz		= pwmFreq;					// set frequency of PWM signal
			_timers_config[_pwmTimer]->timer_num	= (ledc_timer_t)_pwmTimer;	// set timer index
			_timers_config[_pwmTimer]->clk_cfg		= LEDC_AUTO_CLK;			// set LEDC source clock

				// find max PWM resolution for pwmFreq
			uint8_t resBit = LEDC_TIMER_BIT_MAX;								// max PWM resolution in bit

				// redirect esp log to silent log function
			esp_log_set_vprintf(&DcMotorCore::espSilentLog);
																												DPRINTLN("    -> Computing PWM maximum bit resolution for frequency.");
 			while ((resBit > 0) && (_logHasOccure != 0)) {
					// reset silent log flag tracker
				_logHasOccure = false;

					// test timer with current resBit
				_timers_config[_pwmTimer]->duty_resolution = (ledc_timer_bit_t)resBit;	// write resolution in timer config
				ledc_timer_config(_timers_config[_pwmTimer]);							// try to start timer with current config. Return write to char_in_esp_error_log.
  
				resBit--;
			}
																												DPRINT("    -> Resolution of "); DPRINT(_timers_config[_pwmTimer]->duty_resolution); DPRINTLN(" bits found");
				// reset silent log flag and restore esp log output tu UART0
			_logHasOccure = NOT_SET;
			esp_log_set_vprintf(&vprintf);

			if (resBit <= 0) {																					DPRINTLN("    -> Unexpected error durring PWM timer config. Abording configuration.");
					// free up memory and exit
				delete _timers_config[_pwmTimer];
				
				return EXIT_FAILURE;
			}
		}
	}
	else {																										DPRINT("  - PWM timer specified by user. Using timer ");DPRINT(_pwmTimer); DPRINTLN(" for PWM signal");
		// pwm timer set prior attach(). Statement for debug output.
	}
	
	
		// create PWM channel config structure
	_ledc_channel_config = new ledc_channel_config_t;
																												DPRINTLN("  - PWM channel selection :");
		// auto select PWM channel if no pwmChannel is provided prior setup()
	if (_pwmChannel == NOT_SET) {																				DPRINTLN("    -> No PWM channel specified. Auto-selecting a free one.");
		for (uint8_t channel = 0; channel < LEDC_CHANNEL_MAX; channel++)
		{		// use first free channel ...
			if (_pwm_channel_used[channel] == false) {															DPRINT("      -> Channel "); DPRINT(channel); DPRINTLN(" free. Using it for PWM signal.");
				_pwm_channel_used[channel] = true;
				_pwmChannel = channel;
				
				break;
			}
				// ... or exit with error if no one are available
			if (channel == (LEDC_CHANNEL_MAX - 1)) {															DPRINTLN("      -> No suitable free channel available. Abording configuration.");
					// free up memory and exit
				delete _ledc_channel_config;
				
				if (_clients_for_timer[_pwmTimer] == 0) {
					delete _timers_config[_pwmTimer];
				}
				
				return EXIT_FAILURE;
			}
		}
	}
	
	else {																										DPRINT("    -> PWM channel specified by user. Using channel ");DPRINT(_pwmChannel); DPRINTLN(" for this PWM signal");
		// pwm channel set prior attach(). Statement for debug output.
	}
	
		// seeding PWM channel config parameters 
	_ledc_channel_config->channel				= (ledc_channel_t)_pwmChannel;
	_ledc_channel_config->duty					= 0;
	_ledc_channel_config->hpoint				= 0;
	_ledc_channel_config->gpio_num				= pwmPin;
	_ledc_channel_config->speed_mode			= LEDC_LOW_SPEED_MODE;
	_ledc_channel_config->timer_sel				= (ledc_timer_t)_pwmTimer;
	_ledc_channel_config->intr_type				= LEDC_INTR_DISABLE;
	_ledc_channel_config->flags.output_invert 	= 0;

		// start PWM signal on pwmPin.
	ledc_channel_config(_ledc_channel_config);																	DPRINT("  - Starting PWM signal on pin "); DPRINTLN(pwmPin);
																												DPRINTLN("PWM generator configuration success."); DPRINTLN();
		// register this instance for PWM timer used.
	_clients_for_timer[_pwmTimer]++;
	
		// set ledc fade service on for ledc_set_duty_and_update/ledc_set_fade_time_and_start functions
	ledc_fade_func_install(0);
		
		// set _pwmMaxDuty from duty bit resolution
	_pwmMaxDuty = getMaxDutyVal();
	



		// wake up driver if need
	doNotSleep();

	return EXIT_SUCCESS;
}



/**
 * @brief Set enable pin and its active level
 * 
 * @details Configure motor driver enable pin if available (i.e. in Phase/enable mode)
 * 
 * @param	enablePin Pin connected to motor driver enable input
 * @param	mode Pin mode (active high or active low)
 */

bool DcMotorCore::setEnablePin(uint8_t enablePin, ActiveLevel mode) {
	DPRINTLN("DcMotorCore: Enable pin configuration:");

		// --- 1. Input parameters validation ---
	if (!isSafeOutput(enablePin) || !isValidActiveLevel(mode)) {
		DPRINTLN("    -> Invalid Enable Pin or ActiveLevel mode. Configuration aborted.");
		return false;
	}

		// --- 2. Configuration ---
	_enablePin = enablePin; // Automatically wrapped into std::optional
	_enablePinMode = mode;

	pinMode(_enablePin.value(), OUTPUT);
	DPRINT("    -> Enable pin attached to pin "); DPRINTLN(_enablePin.value());

	// --- 3. Initial state: Set to disabled for safety ---
	return disable();
}



/**
 * @brief Configure decay pin and modes for each electrical state.
 * 
 * @param decayPin Pin connected to motor driver decay input
 * @param lowState Decay mode mode when pin is LOW
 * @param highState Decay mode when pin is HIGH
 */
bool DcMotorCore::setDecayPin(uint8_t decayPin, DecayMode lowState, DecayMode highState) {
	DPRINTLN("DcMotorCore: Decay pin configuration:");

		// --- 1. Safety check ---
	if (!isSafeOutput(decayPin)) {
		DPRINTLN("    -> Invalid Decay Pin. Configuration aborted.");
		return false;
	}

		// --- 2. Assignment ---
	_decayPin = decayPin; // Wrapped into std::optional
	_lowDecayPinMode = lowState;
	_highDecayPinMode = highState;

	pinMode(_decayPin.value(), OUTPUT);
	DPRINT("    -> Decay pin attached to pin "); DPRINTLN(_decayPin.value());

		// --- 3. Initial state: Set to the mode defined for the LOW state ---
	return decayMode(_lowDecayPinMode);
}



/**
 * @brief Configure the motor driver sleep pin and its active level.
 * 
 * @details Used to put the driver into low-power mode or wake it up.
 */
bool DcMotorCore::setSleepPin(uint8_t sleepPin, ActiveLevel mode) {
	DPRINTLN("DcMotorCore: Sleep pin configuration:");

		// --- 1. Input parameters validation ---
	if (!isSafeOutput(sleepPin) || !isValidActiveLevel(mode)) {
		DPRINTLN("    -> Invalid Sleep Pin or ActiveLevel mode. Configuration aborted.");
		return false;
	}

		// --- 2. Configuration ---
	_sleepPin = sleepPin; // Wrapped into std::optional
	_sleepPinMode = mode;

	pinMode(_sleepPin.value(), OUTPUT);
	DPRINT("    -> Sleep pin attached to pin "); DPRINTLN(_sleepPin.value());

		// --- 3. Initial state: Force sleep for maximum safety ---
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

	const float safetyGap = 10.0f;
		// --- 1. Validation with safety gap ---
	if (minMargin >= MIN_SPEED && 
	    maxMargin <= MAX_SPEED && 
	    (maxMargin - minMargin) >= safetyGap) {

		_minMargin = minMargin;
		_maxMargin = maxMargin;

		// --- 2. Smart flag switching ---
		// If margins are at absolute limits, we disable the mapping logic to save CPU
		_marginAreSet = !(_minMargin == MIN_SPEED && _maxMargin == MAX_SPEED);

		DPRINT("    -> Margins updated. Active: "); DPRINTLN(_marginAreSet ? "YES" : "NO (Default)");
		
		return true;
	}

	DPRINTLN("    -> Error: Invalid margin range or gap too small.");
	return false;
}



/**
 * @brief Set motor speed in percent
 * 
 * @param speed Target speed value (-100.0 to 100.0)
 * @return true if speed was set, false if value was invalid
 */

bool DcMotorCore::runAtSpeed(float speed) {
	DPRINTLN("DcMotorCore: Setting motor speed.");

		// --- 1. Validation ---
	if (speedIsValid(speed)) {
		
			// --- 2. Margin Mapping ---
			// We use the internal helper which already checks _marginAreSet
		float mappedSpeed = speedInMargin(speed);

			// --- 3. Direction management ---
		dirPinFromSpeed(mappedSpeed);

			// --- 4. PWM Duty Cycle calculation ---
		uint32_t duty = speedToDuty(mappedSpeed);

			// --- 5. Hardware Update ---
			// Note: Most ESP32 Arduino cores use LEDC_LOW_SPEED_MODE for all channels
			// ledc_set_duty_and_update is a fast way to apply changes
		if (ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, (ledc_channel_t)_pwmChannel, duty, 0) == ESP_OK) {
			return true;
		}
	}

	DPRINTLN("  DcMotorCore: Speed setting aborted.");
	return false;
}



/////////////////////////////////////////////////////////////////////////////////////
/*	accelToSpeed(speed, accel) - set motor speed with acceleration
/		float speed: motor speed in % (0-100%)
/		uint32_t accel: acceleration in ms per speed %                             */
/////////////////////////////////////////////////////////////////////////////////////
bool DcMotorCore::accelToSpeed(float targetSpeed, uint32_t accel)
{																												DPRINTLN("Setting speed with acceleration.");
		// set speed if provide speed is valid
	if(speedIsValid(targetSpeed) == EXIT_SUCCESS && accelIsValid(accel) == EXIT_SUCCESS) {
			// map speed into custom margin if set 
		if (_marginAreSet) {
			targetSpeed = speedInMargin(targetSpeed);
		}
			// set _dirPin direction (if defined) from speed.
		dirPinFromSpeed(targetSpeed);

			// check acceleration factor to avoid negative value underflow
		uint32_t currentDuty = ledc_get_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)_pwmChannel);				DPRINT("currentDuty is : "); DPRINTLN(currentDuty);
		uint32_t target_duty = speedToDuty(targetSpeed);														DPRINT("target_duty is : "); DPRINTLN(target_duty);
		uint32_t fadeInDuty = 0; 
		
			// speed decrease
		if (max(currentDuty, target_duty) == target_duty) {														DPRINTLN("Increasing speed : ");
			fadeInDuty = target_duty - currentDuty;																DPRINT("fadeInDuty is : "); DPRINTLN(fadeInDuty);
		}
			// speed increase
		else {																									DPRINTLN("Decreasing speed : ");
			fadeInDuty = currentDuty - target_duty;																DPRINT("fadeInDuty is : "); DPRINTLN(fadeInDuty);
		}

		uint32_t max_fade_time_ms = ((fadeInDuty * accel * 100) / _pwmMaxDuty);									DPRINT("max_fade_time_ms is : "); DPRINTLN(max_fade_time_ms);
		ledc_set_fade_time_and_start(LEDC_LOW_SPEED_MODE, (ledc_channel_t)_pwmChannel, target_duty, max_fade_time_ms, LEDC_FADE_NO_WAIT);

		return EXIT_SUCCESS;
	}
	return EXIT_FAILURE;
}



/**
 * @brief Stop motor rotation immediately.
 */

void DcMotorCore::stop() {
	DPRINTLN("DcMotorCore: Motor stopped.");

		// --- 1. Interruption of any ongoing hardware fade ---
		// Ensures that the speed 0 command is not ignored by the PWM peripheral
	ledc_fade_stop(LEDC_LOW_SPEED_MODE, (ledc_channel_t)_pwmChannel);

		// --- 2. Force speed to minimum ---
	runAtSpeed(MIN_SPEED);
}



/**
 * @brief Apply the requested decay mode based on hardware mapping.
 */

bool DcMotorCore::decayMode(DecayMode mode) {
	if (_decayPin == NOT_SET || mode == DecayMode::Unset) {
		DPRINTLN("DcMotorCore: Decay command ignored (pin not set or Unset mode).");
		return false;
	}

		// --- 1. Map intent to physical state ---
	if (mode == _lowDecayPinMode) {
		digitalWrite(_decayPin, LOW);
	} 
	else if (mode == _highDecayPinMode) {
		digitalWrite(_decayPin, HIGH);
	} 
	else {
		DPRINTLN("DcMotorCore: Requested mode not supported by this hardware.");
		return false;
	}

	DPRINT("DcMotorCore: Decay mode set to "); DPRINTLN((mode == DecayMode::SlowDecay) ? "SLOW" : "FAST");
	
	return true;
}




/**
 * @brief Get the current decay mode by reading the physical pin state.
 */

DecayMode DcMotorCore::decayMode() {
	if (_decayPin == NOT_SET) return DecayMode::Unset;

	// --- 1. Read physical pin and match with the mapping ---
	if (digitalRead(_decayPin) == LOW) {
		return _lowDecayPinMode;
	} 
	
	else {
		return _highDecayPinMode;
	}
}



/**
 * @brief Enable the motor driver output.
 */

bool DcMotorCore::enable() {
	if (_enablePin == NOT_SET) {
		DPRINTLN("DcMotorCore: Enable pin not set, command ignored.");
		return false;
	}

		// Set pin state based on ActiveLevel
	digitalWrite(_enablePin, (_enablePinMode == ActiveLevel::ActiveHigh) ? HIGH : LOW);
	DPRINTLN("DcMotorCore: Driver enabled.");

	return true;
}



/**
 * @brief Disable the motor driver output.
 */

bool DcMotorCore::disable() {
	if (_enablePin == NOT_SET) {
		DPRINTLN("DcMotorCore: Disable pin not set, command ignored.");
		return false;
	}
	
		// Set pin state to inactive
	digitalWrite(_enablePin, (_enablePinMode == ActiveLevel::ActiveHigh) ? LOW : HIGH);
	DPRINTLN("DcMotorCore: Driver disabled.");

	return true;
}



/**
 * @brief Put the motor driver into sleep mode.
 */

bool DcMotorCore::sleep() {
	if (_sleepPin == NOT_SET) {
		DPRINTLN("DcMotorCore: Sleep pin not set, command ignored.");
		return false;
	}

		// Set pin state to sleep (opposite of ActiveLevel logic)
	digitalWrite(_sleepPin, (_sleepPinMode == ActiveLevel::ActiveHigh) ? LOW : HIGH);
	DPRINTLN("DcMotorCore: Driver is now sleeping.");

	return true;
}




/**
 * @brief Wakeup the motor driver from sleep mode.
 */

bool DcMotorCore::wakeup() {
	if (_sleepPin == NOT_SET) {
		DPRINTLN("DcMotorCore: Wakeup pin not set, command ignored.");
		return false;
	}

		// Set pin state to active
	digitalWrite(_sleepPin, (_sleepPinMode == ActiveLevel::ActiveHigh) ? HIGH : LOW);
	DPRINTLN("DcMotorCore: Driver is now awake.");

	return true;
}



/**
 * @brief Return the current speed with direction (+ for CW, - for CCW).
 */
float DcMotorCore::getSpeed() {
		// --- 1. Safety check ---
	if (_ledc_channel_config == nullptr || _pwmTimer == NOT_SET) {
		DPRINTLN("DcMotorCore Error: getSpeed() called before attach() or timer init.");
		return 0.0f;
	}

		// --- 2. Get raw speed from hardware duty cycle ---
	uint32_t currentDuty = ledc_get_duty(_timers_config[_pwmTimer]->speed_mode, _ledc_channel_config->channel);
	float speed = dutyToSpeed(currentDuty);
	
	DPRINT("DcMotorCore: Raw speed from duty cycle: "); DPRINTLN(speed);

		// --- 3. Revert margin mapping ---
		// revertMargedSpeed internally handles the _marginAreSet check
	speed = revertMargedSpeed(speed);

		// --- 4. Determine direction with ternary operator ---
		// If no dirPin, return raw speed. Otherwise, check physical pin state.
	if (_dirPin == NOT_SET) return speed;

	return (digitalRead(_dirPin) == CLOCKWISE) ? speed : -speed;
}




/**
 * @brief Check if the motor is currently moving.
 * 
 * @return true if current speed is different from MIN_SPEED.
 */
bool DcMotorCore::isMoving() {
	// --- 1. Velocity check ---
	// Since getSpeed() returns a float, we compare against our MIN_SPEED constant
	if (getSpeed() != MIN_SPEED) {
		return true;
	}

	// --- 2. Static state ---
	return false;
}


/**
 * @brief Check if the motor driver is currently in sleep mode.
 */
bool DcMotorCore::isSleeping() {
	// --- 1. Safety check ---
	if (!_sleepPin.has_value()) {
		return false;
	}

	// --- 2. Logic check ---
	// Read the physical pin and compare with the defined Sleep polarity.
	// Since wakeup() sets it to ActiveLevel, sleep is the opposite.
	uint8_t sleepState = (_sleepPinMode == ActiveLevel::ActiveHigh) ? LOW : HIGH;

	return (digitalRead(_sleepPin.value()) == sleepState);
}

/////////////////////////////////////////////////////////////////////////////////////
/*	getPwmTimer() - Return the PWM timer (0-3) use by PWM signal or -1 if not set  */
/////////////////////////////////////////////////////////////////////////////////////
int8_t DcMotorCore::getPwmTimer()
{		// check if PWM is configured and return timer ...
	if (_ledc_channel_config != NULL) {

		return _ledc_channel_config->timer_sel;
	}
		// ... or return NOT_SET if not configured
	return NOT_SET;
}

/////////////////////////////////////////////////////////////////////////////////////
/*	getPwmFreq() - Return the frequency of the PWM signal                          */
/////////////////////////////////////////////////////////////////////////////////////
uint32_t DcMotorCore::getPwmFreq()
{		// check if PWM is configured and return frequency ...
	if (_ledc_channel_config != NULL) {
		return ledc_get_freq(LEDC_LOW_SPEED_MODE, (ledc_timer_t)_pwmTimer);
	}
		// ... or error code if not configured
	return -1;
}

/////////////////////////////////////////////////////////////////////////////////////
/*	getMaxDutyVal() - Return the maximum value of PWM duty cycle                   */
/////////////////////////////////////////////////////////////////////////////////////
uint32_t DcMotorCore::getMaxDutyVal()
{		// check if PWM is configured and return frequency ...
	if (_ledc_channel_config != NULL) {
		uint8_t duty_resolution = _timers_config[_pwmTimer]->duty_resolution;
		
		return (pow(2,duty_resolution) - 1);
	}
		// ... or return error code if not configured
	return -1;
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
	// --- 1. Range validation ---
	// Checks if speed is between -100.0 and 100.0
	if (speed >= -MAX_SPEED && speed <= MAX_SPEED) {
		return true;
	}

	DPRINT("DcMotorCore: Invalid speed requested: "); DPRINTLN(speed);
	DPRINT("    -> Must be between "); DPRINT( -MAX_SPEED); 
	DPRINT(" and "); DPRINTLN(MAX_SPEED);

	return false;
}




/**
 * @brief Convert normalized speed (-100.0 to 100.0) to PWM duty cycle.
 */
uint32_t DcMotorCore::speedToDuty(float speed) {
		// --- Case 1: Phase/enable mode ---
		// duty = max_duty * ((speed + 100) / 200)
	if (!_dirPin.has_value()) {
		uint32_t duty = (uint32_t)round(_pwmMaxDuty * ((speed + MAX_SPEED) / (MAX_SPEED * 2.0f)));
		
		DPRINT("DcMotorCore: PH/EN duty calculation: "); DPRINTLN(duty);
		return duty;
	}

		// --- Case 2 & 3: Unidirectional or Speed/Dir mode ---
		// Formula: duty = max_duty * (abs(speed) / 100)
	float absSpeed = (speed < 0.0f) ? -speed : speed;
	uint32_t duty = (uint32_t)round(_pwmMaxDuty * (absSpeed / MAX_SPEED));

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
		float speed = (((((float)duty * 2.0f) - _pwmMaxDuty) / _pwmMaxDuty) * MAX_SPEED);
		
		DPRINT("DcMotorCore: PH/EN speed conversion: "); DPRINTLN(speed);
		return speed;
	}

		// --- Case 2 & 3: Unidirectional or Speed/Dir mode ---
		// Logic: Duty cycle is a direct magnitude (0 to 100%).
		// Formula: speed = (duty / max_duty) * 100
	float speed = (((float)duty / _pwmMaxDuty) * MAX_SPEED);

	DPRINT("DcMotorCore: Standard speed conversion: "); DPRINTLN(speed);
	return speed;
}



/**
 * @brief Maps the input speed into the configured min/max margin range.
 * 
 * @details This linear mapping ensures the motor starts at _minMargin 
 *          and reaches its limit at _maxMargin.
 * 
 * @param speed Input speed from 0.0 to 100.0 (or negative)
 * @return float Mapped speed ready for PWM conversion
 */

float DcMotorCore::speedInMargin(float speed) {
		// --- 1. Pass-through if no margins are set or speed is zero ---
	if (!_marginAreSet || speed == 0.0f) {
		return speed;
	}

		// --- 2. Store direction and work with absolute value ---
	bool isNegative = (speed < 0.0f);
	float absSpeed = isNegative ? -speed : speed;

		// --- 3. Linear Mapping (float precision) ---
		// Formula: min + (input * (max - min) / 100)
	float mapped = _minMargin + (absSpeed * (_maxMargin - _minMargin) / 100.0f);

		// --- 4. Safety Clamping ---
	if (mapped > _maxMargin) mapped = _maxMargin;

	return isNegative ? -mapped : mapped;
}



/**
 * @brief Reverts a physical speed (with margins) back to its normalized 0-100% value.
 * 
 * @param speed Mapped speed from PWM duty cycle
 * @return float Normalized speed (0.0 to 100.0)
 */

float DcMotorCore::revertMargedSpeed(float speed) {
		// --- 1. Pass-through if no margins or speed is zero ---
	if (!_marginAreSet || speed == 0.0f) {
		return speed;
	}

	bool isNegative = (speed < 0.0f);
	float absSpeed = isNegative ? -speed : speed;

		// --- 2. Inverse Linear Mapping ---
		// Formula: (absSpeed - min) * 100 / (max - min)
	float normalized = (absSpeed - _minMargin) * 100.0f / (_maxMargin - _minMargin);

		// --- 3. Clamping for telemetry safety ---
	if (normalized < 0.0f) normalized = 0.0f;
	if (normalized > 100.0f) normalized = 100.0f;

	return isNegative ? -normalized : normalized;
}



/**
 * @brief Set the physical direction pin state based on speed value.
 */
bool DcMotorCore::dirPinFromSpeed(float speed) {
	// --- 1. Check if a direction pin is defined (Speed/Dir mode) ---
	if (!_dirPin.has_value() || _dirPin.value() == -1) {
		DPRINTLN("DcMotorCore: No direction pin defined for current mode.");
		return false;
	}

	// --- 2. Set direction pin according to speed sign ---
	if (speed >= 0.0f) {
		// Set clockwise for positive and zero speed value
		digitalWrite(_dirPin.value(), CLOCKWISE);
		DPRINTLN("DcMotorCore: Direction pin set to CLOCKWISE.");
	} 
	else {
		// Set counter-clockwise for negative speed value
		digitalWrite(_dirPin.value(), COUNTERCLOCKWISE);
		DPRINTLN("DcMotorCore: Direction pin set to COUNTERCLOCKWISE.");
	}

	return true;
}



/**
 * @brief Verify if a pin is safe and capable of output
 */

bool DcMotorCore::isSafeOutput(uint8_t pin) {
	// --- 1. Check if hardware supports output (exclude 34-39) ---
	if (!GPIO_IS_VALID_OUTPUT_GPIO(pin)) return false;

	// --- 2. Exclude SPI Flash pins (6 to 11) ---
	if (pin >= 6 && pin <= 11) return false;

	return true;
}




/**
 * @brief Verify if an ActiveLevel value is within valid range
 */

bool DcMotorCore::isValidActiveLevel(ActiveLevel mode) {
	return (mode == ActiveLevel::ActiveLow || mode == ActiveLevel::ActiveHigh);
}


/////////////////////////////////////////////////////////////////////////////////////
/*	espSilentLog(...) - read number of character write durring esp_log msg
										                                           */
/////////////////////////////////////////////////////////////////////////////////////
int DcMotorCore::espSilentLog(const char* string, va_list args)
{		// store number of character into "string"
	_logHasOccure = vsnprintf(NULL, 0, string, args);

	return vprintf("coucou", args);
}


/* destructor:
	-> delete (free) *ledc_channel_config memory AND set _pwm_channel_used[channel] = false
	-> _clients_for_timer[_pwmTimer]--;
	-> delete (free) *_timers_config[_pwmTimer] SI _clients_for_timer[_pwmTimer] == 0
	-> _default_timer = NOT_SET SI _default_timer != NOT_SET ET _clients_for_timer[_default_timer] == 0
*/