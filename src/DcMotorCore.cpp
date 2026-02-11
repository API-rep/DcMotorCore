#include "ESP32_PWM_Motor.h"

#include "Arduino.h"

// #include "esp32-hal.h"
// #include "soc/soc_caps.h"
#include "driver/ledc.h"

ledc_timer_config_t *ESP32_PWM_Motor::_timers_config[] = {NULL};	// init timers config pointers
uint8_t	ESP32_PWM_Motor::_clients_for_timer[] = {0};				// init timers clients channels counter
int8_t	ESP32_PWM_Motor::_default_timer = NOT_SET;
bool	ESP32_PWM_Motor::_pwm_channel_used[] = {0};				    // init channels usage log array

int		ESP32_PWM_Motor::_logHasOccure = NOT_SET;					// init espErr char counter attribute

/////////////////////////////////////////////////////////////////////////////////////
/*	ESP32_PWM_Motor constructor - build an instance of the class
/		Initalize motor contoller and default (safe) atributes values              */
/////////////////////////////////////////////////////////////////////////////////////
ESP32_PWM_Motor::ESP32_PWM_Motor()
{
	_dirPin			   = NOT_SET;
	_enablePin     = NOT_SET;
	_enablePinMode = NOT_SET;
	_breakPin		   = NOT_SET;
	_breakPinMode	 = NOT_SET;
	_sleepPin		   = NOT_SET;
	_sleepPinMode	 = NOT_SET;
	
	_pwmTimer		   = NOT_SET;
	_pwmChannel		 = NOT_SET;
	_pwmMaxDuty		 = NOT_SET;



	_minMargin		 = MIN_SPEED;
	_maxMargin		 = MAX_SPEED;
	_marginAreSet	 = false;
}

/////////////////////////////////////////////////////////////////////////////////////
/*	useTimer(timer) - set a specific timer for PWM signal before attach()
/		uint8_t timer: ledc high speed timer to use (0-3)
/	NOTE: Can be used once and before attach() (will be ignore otherwise)           
/         Timer of another motor can be use. Use thisMotor.getPwmTimer() to get it */
/////////////////////////////////////////////////////////////////////////////////////
bool ESP32_PWM_Motor::useTimer(int8_t timer)
{
	if ((timer < LEDC_TIMER_MAX) && (_pwmTimer == NOT_SET)) {
		_pwmTimer = timer;
		return EXIT_SUCCESS;
	}
	
	return EXIT_FAILURE;
}

/////////////////////////////////////////////////////////////////////////////////////
/*	useChannel(channel) - set a specific channel for PWM signal before attach()
/		uint8_t channel: ledc high speed channel to use (0-7)
/	NOTE: Can be used once and before attach() (will be ignore otherwise)           
/         Usefull to force PWM channel to use if a conflict is meet                */
/////////////////////////////////////////////////////////////////////////////////////
bool ESP32_PWM_Motor::useChannel(int8_t channel)
{
	if ((channel < LEDC_CHANNEL_MAX) && (_pwmChannel == NOT_SET)) {
		_pwmChannel = channel;
		
		return EXIT_SUCCESS;
	}
	
	return EXIT_FAILURE;
}

/////////////////////////////////////////////////////////////////////////////////////
/*	attach(pwmPin, dirPin, pwmFreq) - setup motor controller pins and PWM signal
/		uint8_t pwmPin: pin connected to motor driver PWM input
/		uint8_t dirPin: pin connected to motor driver direction pin
/		uint32_t pwmFreq: frequency of PWM output (= DEF_PWM_FREQ if not set)      */
/////////////////////////////////////////////////////////////////////////////////////
bool ESP32_PWM_Motor::attach(uint8_t pwmPin, int8_t dirPin, uint32_t pwmFreq)									
{																												DPRINTLN("Setting up PWM signal generator ...");
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
			_timers_config[_pwmTimer]->speed_mode	= LEDC_HIGH_SPEED_MODE;		// set timer mode
			_timers_config[_pwmTimer]->freq_hz		= pwmFreq;					// set frequency of PWM signal
			_timers_config[_pwmTimer]->timer_num	= (ledc_timer_t)_pwmTimer;	// set timer index
			_timers_config[_pwmTimer]->clk_cfg		= LEDC_AUTO_CLK;			// set LEDC source clock

				// find max PWM resolution for pwmFreq
			uint8_t resBit = LEDC_TIMER_BIT_MAX;								// max PWM resolution in bit

				// redirect esp log to silent log function
			esp_log_set_vprintf(&ESP32_PWM_Motor::espSilentLog);
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
	_ledc_channel_config->speed_mode			= LEDC_HIGH_SPEED_MODE;
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
	
		// set direction pin if provided
	if (dirPin != NOT_SET) {
		_dirPin = dirPin;																						DPRINT("DIR pin set to pin"); DPRINTLN(_dirPin);
		pinMode(_dirPin, OUTPUT);
	}

	else {																										DPRINTLN("No DIR pin provided."); DPRINTLN();
		// direction pin not set. Direction set by pus/minus speed.
	}

		// wake up driver if need
	doNotSleep();

	return EXIT_SUCCESS;
}

/////////////////////////////////////////////////////////////////////////////////////
/*	setDirMode(dirMode) - set motor driver dir mode (speed/dir or phase/enable)
/		uint8_t dirMode: dir pin mode (speed/dir or phase/enable)                      */
/////////////////////////////////////////////////////////////////////////////////////
bool ESP32_PWM_Motor::setEnablePin(uint8_t enablePin, uint8_t mode)
{																												DPRINTLN("Enable pin configuration:");
	if (mode == ESP32_PWM_MOTOR_ACTIVE_LOW || mode == ESP32_PWM_MOTOR_ACTIVE_HIGH) {
		_enablePin = enablePin;																					DPRINT("  Enable pin attached to pin "); DPRINTLN(_enablePin);
		_enablePinMode = mode;																					DPRINTLN("  Enable pin mode correctly set");
		
		pinMode(_enablePin, OUTPUT);
		
		return EXIT_SUCCESS;
	}
																												DPRINTLN("  Bad Enable pin mode provided. Configuration aborded");
	return EXIT_FAILURE;
}

/////////////////////////////////////////////////////////////////////////////////////
/*	setBreakPin(breakPin, mode) - set motor driver break pin and its activemode
/		uint8_t breakPin: pin connected to motor driver break pin
/		uint8_t mode: active mode of the break pin                                 */
/////////////////////////////////////////////////////////////////////////////////////
bool ESP32_PWM_Motor::setBreakPin(uint8_t breakPin, uint8_t mode)
{																												DPRINTLN("Break pin configuration:");
	if (mode == ESP32_PWM_MOTOR_ACTIVE_LOW || mode == ESP32_PWM_MOTOR_ACTIVE_HIGH) {
		_breakPin = breakPin;																					DPRINT("  Break pin attached to pin "); DPRINTLN(_breakPin);
		_breakPinMode = mode;																					DPRINTLN("  Break pin mode correctly set");
		
		pinMode(_breakPin, OUTPUT);
		
		return EXIT_SUCCESS;
	}
																												DPRINTLN("  Bad break pin mode provided. Configuration aborded");
	return EXIT_FAILURE;
}

/////////////////////////////////////////////////////////////////////////////////////
/*	setSleepPin(breakPin, mode) - set motor driver sleep pin and its active mode
/		uint8_t breakPin: pin connected to motor driver sleep pin
/		uint8_t mode: active mode of the sleep pin                                 */
/////////////////////////////////////////////////////////////////////////////////////
bool ESP32_PWM_Motor::setSleepPin(uint8_t sleepPin, uint8_t mode)
{																												DPRINTLN("Sleep pin configuration:");
	if (mode == ESP32_PWM_MOTOR_ACTIVE_LOW || mode == ESP32_PWM_MOTOR_ACTIVE_HIGH) {
		_sleepPin = sleepPin;																					DPRINT("  Sleep pin attached to pin "); DPRINTLN(_sleepPin);
		_sleepPinMode = mode;																					DPRINTLN("  Sleep pin mode correctly set");
		
		pinMode(_sleepPin, OUTPUT);
		
		return EXIT_SUCCESS;
	}
																												DPRINTLN("  Bad sleep pin mode provided. Configuration aborded");
	return EXIT_FAILURE;
}

/////////////////////////////////////////////////////////////////////////////////////
/*	MotorMargin(minMargin, maxMargin) - compensate motor dead zone
/		uint8_t minMargin: minimum speed at which the motor start
/		uint8_t maxMargin: maximum motor speed limit (in booth direction)          */
/////////////////////////////////////////////////////////////////////////////////////
bool ESP32_PWM_Motor::setMargin(uint8_t minMargin, uint8_t maxMargin)
{																												DPRINTLN("  Setting up minMargin/maxMargin");
	if (minMargin >= MIN_SPEED && maxMargin <= MAX_SPEED && minMargin < maxMargin) {
		_minMargin = minMargin;
		_maxMargin = maxMargin;

			// check if custom margin are set and switc on/off marginAreSet flag
		if (minMargin == MIN_SPEED && maxMargin == MAX_SPEED) {
			_marginAreSet = false;																				DPRINTLN("    -> Min/max Margin set (or reset) to default value.");
		}

		else {
			_marginAreSet = true;																				DPRINTLN("    -> New minMargin/maxMargin set correctly.");
		}

		return EXIT_SUCCESS;
	}
																												DPRINTLN("    -> Bad minMargin and/or maxMargin value provided.");
	return EXIT_FAILURE;
}

/////////////////////////////////////////////////////////////////////////////////////
/*	runAtSpeed(speed) - set motor speed (PWM duty cycle)
/		float speed: motor speed in % (0-100%)                                     */
/////////////////////////////////////////////////////////////////////////////////////
bool ESP32_PWM_Motor::runAtSpeed(float speed)
{																												DPRINTLN("Setting motor speed.");
		// set speed if provide speed is valid
	if (speedIsValid(speed) == EXIT_SUCCESS) {
			// map speed into custom margin if set 
		if (_marginAreSet) {
			speed = speedInMargin(speed);
		}

			// set _dirPin direction (if defined) from speed
		dirPinFromSpeed(speed);

			// compute PWM duty cycle from booth mode (DIR pin or not)
		uint32_t duty = speedToDuty(speed);

		ledc_set_duty_and_update(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)_pwmChannel, duty, 0);

		return EXIT_SUCCESS;
	}
																												DPRINTLN("  Speed setting aborded.");
	return EXIT_FAILURE;
}

/////////////////////////////////////////////////////////////////////////////////////
/*	accelToSpeed(speed, accel) - set motor speed with acceleration
/		float speed: motor speed in % (0-100%)
/		uint32_t accel: acceleration in ms per speed %                             */
/////////////////////////////////////////////////////////////////////////////////////
bool ESP32_PWM_Motor::accelToSpeed(float targetSpeed, uint32_t accel)
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
		uint32_t currentDuty = ledc_get_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)_pwmChannel);				DPRINT("currentDuty is : "); DPRINTLN(currentDuty);
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
		ledc_set_fade_time_and_start(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)_pwmChannel, target_duty, max_fade_time_ms, LEDC_FADE_NO_WAIT);

		return EXIT_SUCCESS;
	}
	return EXIT_FAILURE;
}

/////////////////////////////////////////////////////////////////////////////////////
/*	stop() -  stop motor rotatation                                           */
/////////////////////////////////////////////////////////////////////////////////////
void ESP32_PWM_Motor::stop()
{																												DPRINTLN("Motor stoped.");
	runAtSpeed(MIN_SPEED);
}

/////////////////////////////////////////////////////////////////////////////////////
/*	enable() -  enable motor driver                                                */
/////////////////////////////////////////////////////////////////////////////////////
bool ESP32_PWM_Motor::enable() 
{	
	if (_enablePin != NOT_SET) {																					DPRINTLN("Motor driver enable");
			// set enable pin direction in regard of enable pin mode
		if (_enablePinMode == ESP32_PWM_MOTOR_ACTIVE_LOW) {
			digitalWrite(_enablePin, LOW);
		}

		else {
			digitalWrite(_enablePin, HIGH);
		}
		
		return EXIT_SUCCESS;
	}
																												DPRINT("Enable configuation aborded. No enable pin configured.");
	return EXIT_FAILURE;
}

/////////////////////////////////////////////////////////////////////////////////////
/*	enable() -  disable motor driver                                               */
/////////////////////////////////////////////////////////////////////////////////////
bool ESP32_PWM_Motor::disable()
{	
	if (_enablePin != NOT_SET) {																					DPRINTLN("Motor driver enable");
			// set enable pin direction in regard of enable pin mode
		if (_enablePinMode == ESP32_PWM_MOTOR_ACTIVE_LOW) {
			digitalWrite(_enablePin, HIGH);
		}

		else {
			digitalWrite(_enablePin, LOW);
		}
		
		return EXIT_SUCCESS;
	}
																												DPRINT("Enable configuation aborded. No enable pin configured.");
	return EXIT_FAILURE;
}

/////////////////////////////////////////////////////////////////////////////////////
/*	doBreak() - enable motor driver break mode                                     */
/////////////////////////////////////////////////////////////////////////////////////
bool ESP32_PWM_Motor::doBreak()
{	
	if (_breakPin != NOT_SET) {																					DPRINTLN("Motor driver break mode enable");
			// set break pin direction in regard of break pin mode
		if (_breakPinMode == ESP32_PWM_MOTOR_ACTIVE_LOW) {
			digitalWrite(_breakPin, LOW);
		}

		else {
			digitalWrite(_breakPin, HIGH);
		}
		
		return EXIT_SUCCESS;
	}
																												DPRINT("Break mode configuation aborded. No break pin configured.");
	return EXIT_FAILURE;
}

/////////////////////////////////////////////////////////////////////////////////////
/*	doNotBreak()() - disable motor driver break mode                               */
/////////////////////////////////////////////////////////////////////////////////////
bool ESP32_PWM_Motor::doNotBreak()
{	
	if (_breakPin != NOT_SET) {																					DPRINTLN("Motor driver break mode disable");
			// set break pin direction in regard of break pin mode
		if (_breakPinMode == ESP32_PWM_MOTOR_ACTIVE_LOW) {
			digitalWrite(_breakPin, HIGH);
		}

		else {
			digitalWrite(_breakPin, LOW);
		}
		
		return EXIT_SUCCESS;
	}
																													DPRINT("Break mode configuation aborded. No break pin configured.");
	return EXIT_FAILURE;
}

/////////////////////////////////////////////////////////////////////////////////////
/*	sleep() - put motor driver into sleep mode                                     */
/////////////////////////////////////////////////////////////////////////////////////
bool ESP32_PWM_Motor::sleep()
{	
	if (_sleepPin != NOT_SET) {																					DPRINTLN("Motor driver put into sleep mode");
			// set sleep pin direction in regard of sleep pin mode
		if (_sleepPinMode == ESP32_PWM_MOTOR_ACTIVE_LOW) {
			digitalWrite(_sleepPin, LOW);
		}

		else {
			digitalWrite(_sleepPin, HIGH);
		}
		
		return EXIT_SUCCESS;
	}
																												DPRINTLN("Sleep mode configuation aborded. No sleep pin configured.");
	return EXIT_FAILURE;
}

/////////////////////////////////////////////////////////////////////////////////////
/*	doNotSleep()() - put motor driver into active mode                             */
/////////////////////////////////////////////////////////////////////////////////////
bool ESP32_PWM_Motor::doNotSleep()
{	
	if (_sleepPin != NOT_SET) {																					DPRINTLN("Motor driver put into active mode");
			// set sleep pin direction in regard of sleep pin mode
		if (_sleepPinMode == ESP32_PWM_MOTOR_ACTIVE_LOW) {
			digitalWrite(_sleepPin, HIGH);
		}

		else {
			digitalWrite(_sleepPin, LOW);
		}
		
		return EXIT_SUCCESS;
	}
																												DPRINTLN("Wakeup aborded. No sleep pin configured.");
	return EXIT_FAILURE;
}

/////////////////////////////////////////////////////////////////////////////////////
/*	wakeup()() - wakeup motor driver from sleep mode                               */
/////////////////////////////////////////////////////////////////////////////////////
bool ESP32_PWM_Motor::wakeup()
{	
	return doNotSleep();
}

/////////////////////////////////////////////////////////////////////////////////////
/*	getSpeed() - Return the current speed with direction (+ for CW, - for CCW)     */
/////////////////////////////////////////////////////////////////////////////////////
float ESP32_PWM_Motor::getSpeed()
{		// get duty cycle from ledc
	float speed = dutyToSpeed(ledc_get_duty(_timers_config[_pwmTimer]->speed_mode, _ledc_channel_config->channel));
//	uint32_t duty = ledc_get_duty(_timers_config[_pwmTimer]->speed_mode, _ledc_channel_config->channel);
																												DPRINT("getSpeed() : raw speed from ledc duty cycle : "); DPRINTLN(speed);
		// revert speed if mapped into margin 
	if (_marginAreSet) {
		Serial.print("Speed not reverted : "); Serial.println(speed);
		speed = revertMargedSpeed(speed);
		Serial.print("Speed reverted : "); Serial.println(speed);
	}

		// return [-100,100] speed for "no dirPin" mode, [0,100] for CW direction "dirPin" mode 
	if ((_dirPin == NOT_SET) || (digitalRead(_dirPin) == CLOCKWISE)) {
		return speed;			
	}
		// else, return [-100,0] speed for CCW direction in "dirPin" mode 
	return -speed;
}

/////////////////////////////////////////////////////////////////////////////////////
/*	isMoving() - return true if motor is moving                                    */
/////////////////////////////////////////////////////////////////////////////////////
bool ESP32_PWM_Motor::isMoving()
{		// check current speed (acceleration mode) or target speed (simple mode)
	if (getSpeed() != MIN_SPEED) {
		return true;
	}
		// not moving
	return false;

}

/////////////////////////////////////////////////////////////////////////////////////
/*	isBreaking() - check if motor driver is in breaking mode                       */
/////////////////////////////////////////////////////////////////////////////////////
bool ESP32_PWM_Motor::isBreaking()
{		// check if motor driver is in break mode ...
	if (((digitalRead(_breakPin) == 1) && (_breakPinMode == ESP32_PWM_MOTOR_ACTIVE_HIGH)) || 
	    ((digitalRead(_breakPin) == 0) && (_breakPinMode == ESP32_PWM_MOTOR_ACTIVE_LOW))) {

		return true;
	}
		// ... or not.
	return false;
}

/////////////////////////////////////////////////////////////////////////////////////
/*	isSleeping() - check if motor driver is in sleep mode                          */
/////////////////////////////////////////////////////////////////////////////////////
bool ESP32_PWM_Motor::isSleeping()
{		// check if motor driver is in sleep mode ...
	if (((digitalRead(_sleepPin) == 1) && (_sleepPinMode == ESP32_PWM_MOTOR_ACTIVE_HIGH)) || 
	    ((digitalRead(_sleepPin) == 0) && (_sleepPinMode == ESP32_PWM_MOTOR_ACTIVE_LOW))) {

		return true;
	}
		// ... or not.
	return false;
}

/////////////////////////////////////////////////////////////////////////////////////
/*	getPwmTimer() - Return the PWM timer (0-3) use by PWM signal or -1 if not set  */
/////////////////////////////////////////////////////////////////////////////////////
int8_t ESP32_PWM_Motor::getPwmTimer()
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
uint32_t ESP32_PWM_Motor::getPwmFreq()
{		// check if PWM is configured and return frequency ...
	if (_ledc_channel_config != NULL) {
		return ledc_get_freq(LEDC_HIGH_SPEED_MODE, (ledc_timer_t)_pwmTimer);
	}
		// ... or error code if not configured
	return -1;
}

/////////////////////////////////////////////////////////////////////////////////////
/*	getMaxDutyVal() - Return the maximum value of PWM duty cycle                   */
/////////////////////////////////////////////////////////////////////////////////////
uint32_t ESP32_PWM_Motor::getMaxDutyVal()
{		// check if PWM is configured and return frequency ...
	if (_ledc_channel_config != NULL) {
		uint8_t duty_resolution = _timers_config[_pwmTimer]->duty_resolution;
		
		return (pow(2,duty_resolution) - 1);
	}
		// ... or return error code if not configured
	return -1;
}

/////////////////////////////////////////////////////////////////////////////////////
/*	accelIsValid(accel) - check if motor acceleration in valid
/		uint32_t accel: acceleration factor in ms per speed %                      */
/////////////////////////////////////////////////////////////////////////////////////
bool ESP32_PWM_Motor::accelIsValid(uint32_t accel)
{																												DPRINTLN("  Checking acceleration factor");
	uint32_t uint32MaxVal = -1;
		// test if (accel x _pwmMaxDuty x 100) don't overflow uint32_t max value
	if ((_pwmMaxDuty * accel * 100) < uint32MaxVal) {
		return EXIT_SUCCESS;
	}
																												DPRINT("    Acceleration factor to big. Try with a value lower than "); DPRINTLN(uint32MaxVal / (100 * _pwmMaxDuty));
	return EXIT_FAILURE;

}

/////////////////////////////////////////////////////////////////////////////////////
/*	speedIsValid(speed) - check if abs speed is in -MAX_SPEED <-> MAX_SPEED range
		float speed :  motor speed in %                                            */
/////////////////////////////////////////////////////////////////////////////////////
bool ESP32_PWM_Motor::speedIsValid(float speed)	
{		// check if speed value is in MIN_SPEED <-> +/-MAX_SPEED range
	if (speed == constrain(speed, -MAX_SPEED, MAX_SPEED)) { 													DPRINTLN("  Checking speed value.");
		return EXIT_SUCCESS;
	}
																												DPRINTLN("    Invalid speed value.");
	return EXIT_FAILURE;
}

/////////////////////////////////////////////////////////////////////////////////////
/*	speedToDuty(speed) - convert speed to duty value
		float speed :  motor speed in %                                            */
/////////////////////////////////////////////////////////////////////////////////////
uint32_t ESP32_PWM_Motor::speedToDuty(float speed)
{
	if (_dirPin == NOT_SET) {
			// dir pin not set, duty set in 0 <- 50(neutral) -> 100% range
			DPRINT("  No dir speed convert to duty value "); DPRINT(round(_pwmMaxDuty * ((speed + MAX_SPEED) / (MAX_SPEED * 2))));
		return round(_pwmMaxDuty * ((speed + MAX_SPEED) / (MAX_SPEED * 2)));
	}

	else {
			// dir pin set, duty set in 0 <-> 100% range
				DPRINT("  Dir Speed convert to duty value "); DPRINTLN(round(_pwmMaxDuty * (abs(speed) / MAX_SPEED))); 
		return round(_pwmMaxDuty * (abs(speed) / MAX_SPEED));
	}
}

/////////////////////////////////////////////////////////////////////////////////////
/*	dutyToSpeed(duty) - convert duty cycle to speed
		uint32_t duty :  duty cycle                                                */
/////////////////////////////////////////////////////////////////////////////////////
float ESP32_PWM_Motor::dutyToSpeed(uint32_t duty)
{
	if (_dirPin == NOT_SET) {
			// dir pin not set, speed retuned set in 0 <-> 100% range
		return ((((((float) duty * 2) - _pwmMaxDuty) / _pwmMaxDuty) * MAX_SPEED));
	}
	
	else {
			// dir pin set, speed retuned in -100% <-> 100% range
		return ((((float) duty / _pwmMaxDuty) * MAX_SPEED));
	}
}

/////////////////////////////////////////////////////////////////////////////////////
/*	speedInMargin(speed) - map provide speed into min/max margin range. 
                           zero is not mapped and is retunned unchanged.
		float speed :  speed to convert                                            */
/////////////////////////////////////////////////////////////////////////////////////
float ESP32_PWM_Motor::speedInMargin(float speed)
{		// zero value exception
	if (speed == 0) {																							DPRINTLN("  Zero value returned as it.");
		return 0.0;
	}	
		// map positive speed value into margin ...	
	else if (speed >= 0) {																						DPRINTLN("  Positive speed value mapped to ");
		return ((speed * (((float)(_maxMargin - _minMargin)) / 100)) + _minMargin);
	}
		// ... or negative one (sign is keeped)
	else {																										DPRINTLN("  Negative speed value mapped to ");
		return ((speed * (((float)(_maxMargin - _minMargin)) / 100)) - _minMargin);
	}
}

/////////////////////////////////////////////////////////////////////////////////////
/*	revertMargedSpeed(speed) - rool back marged speed to base speed .
                               zero is not mapped and is retunned unchanged.
		float speed :  speed to convert                                            */
/////////////////////////////////////////////////////////////////////////////////////
float ESP32_PWM_Motor::revertMargedSpeed(float speed)
{		// zero value exception
	if (speed == 0) {																							DPRINTLN("  Zero value returned as it.");
		return 0.0;
	}	
		// map positive speed value into margin ...	
	else if (speed >= 0) {																						DPRINTLN("  Positive speed value reverted");
//		return ((speed * (((float)(_maxMargin - _minMargin)) / 100)) + _minMargin);
		return ((speed - _minMargin) / (((float)(_maxMargin - _minMargin)) / 100));
	}
		// ... or negative one (sign is keeped)
	else {																										DPRINTLN("  Negative speed value reverted to ");
//		return ((speed / (((float)(_maxMargin - _minMargin)) / 100)) + _minMargin);
		return ((speed + _minMargin) / (((float)(_maxMargin - _minMargin)) / 100));
	}
}

/////////////////////////////////////////////////////////////////////////////////////
/*	dirPinFromSpeed() - set dirPin direction from speed value for speed/dir mode   */
/////////////////////////////////////////////////////////////////////////////////////
bool ESP32_PWM_Motor::dirPinFromSpeed(float speed)
{		// set direction pin if defined
	if (_dirPin != NOT_SET) {
			// clockwise for positive and zero speed value
		if (speed >= 0) {
			digitalWrite(_dirPin, CLOCKWISE);																	DPRINTLN("Direction pin direction set clockwise");
		}
			// counterclockwise for negative speed value
		else {
			digitalWrite(_dirPin, COUNTERCLOCWISE);																DPRINTLN("Direction pin direction set counterclockwise");
		}

		return EXIT_SUCCESS;
	}
																												DPRINTLN("No direction pin defined or function not available for current dir mode.");
	return EXIT_FAILURE;
}

/////////////////////////////////////////////////////////////////////////////////////
/*	espSilentLog(...) - read number of character write durring esp_log msg
										                                           */
/////////////////////////////////////////////////////////////////////////////////////
int ESP32_PWM_Motor::espSilentLog(const char* string, va_list args)
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