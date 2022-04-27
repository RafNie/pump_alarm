/*
 * 	Pump station alarm
 * 	ATTiny85
 *
 *  Created on: 11 sty 2022
 *      Author: Rafał Niedźwiedziński
 */
#include <avr/io.h>
#include <inttypes.h>
#include <stdlib.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <assert.h>

// User definitions:
// Time to activate an alarm when the current values are below the threshold [seconds]
#define seconds_to_alarm (2L*24*60*60)
// Time to clear a current monitor counter by the current values over the threshold [seconds]
#define seconds_of_current_measure 4
// Time of values of the enable input below the threshold to disable a current monitor counter [seconds]
#define seconds_of_inactivity_to_disable_alarm (24L*60*60)
// threshold of enable input: ~ 1/8 Ucc=5V in ADC steps
#define enable_input_analog_threshold 256
// current transformer ratio in mV/A
#define current_transformet_ratio 98
// RMS current threshold in mA
#define rms_current_alarm_threshold 3000
// End of user definitions

// Pins definitions
#define BUZZ (PB1)
#define ALARM_RESET (PB0)
#define CURRENT_TRANSFORMER_ADC (_BV(MUX0))
#define ENABLE_MONITOR_ADC (_BV(MUX1))

#if seconds_of_inactivity_to_disable_alarm >= seconds_to_alarm
#error Error of time definitions: seconds_of_inactivity_to_disable_alarm should be less than seconds_to_alarm
#endif

// Don't edit:
// ADC constants
#define fixed_point_precision 1000
#define root_squere_of_two 1414L
#define adc_steps 1024
#define adc_Vref 2560
// Alarm threshold calculation (fixed point)
#define max_current_threshold ((rms_current_alarm_threshold*root_squere_of_two)/fixed_point_precision)
#define voltage_threshold ((max_current_threshold*current_transformet_ratio)/fixed_point_precision)
#define current_threshold ((adc_steps*voltage_threshold)/adc_Vref)
// ADC steps per 1A RMS
#define mV_per_A (((1*root_squere_of_two)*current_transformet_ratio)/fixed_point_precision)
#define steps_per_ampere ((adc_steps*mV_per_A)/adc_Vref)
//

#define heart_beat_period 20
#define samples_number 198

#define eeprom_addr_enable_safe_power_off ((uint8_t*)0x10)
#define eeprom_addr_safe_power_off ((uint8_t*)0x00)

typedef enum {
	alarm,
	start_current_measuring,
	current_measuring,
	checking_alarm_conditions,
	no_alarm
} States;

typedef enum {
	current_monitoring_disabled,
	current_monitoring_enabled
} CurrentMonitoring;

typedef enum {
	false = 0,
	true = 1
} Bool;

typedef enum {
	rising_edge,
	falling_edge,
	stable_high,
	stable_low
} PinChangeState;

Bool enableSafePoweroOff = true;
Bool lastResetPinState = true;
CurrentMonitoring enableState = current_monitoring_enabled; // switched by the ADC enable input
volatile States state = no_alarm;
volatile uint32_t currentMonitorCounter = seconds_to_alarm;
volatile uint32_t enableMonitorCounter = seconds_of_inactivity_to_disable_alarm;
volatile uint16_t currentMonitorBuffer[samples_number];
volatile uint8_t currentMonitorBufferIndex = samples_number;
volatile uint8_t heartBeatCounter = heart_beat_period;
uint8_t measurementsAboveThresholdCounter = seconds_of_current_measure;
Bool currentPresentationFeatureIsEnable = false;

void checkeEnableOfCurrentPresentationFeatureAtStartup();
void presentCurrentLevelIfNeed(uint16_t currentValue);

void handleCounters();
void configureTimer();
void configureIO();
void precessAlarm();
void precessResetAlarm();
void processEnableInput();
void processCurrentMeasurement();
void checkCurrentValue();
void checkAlarmConditions();
void processHeartBeat();
Bool isResetPinPressed();
void beep();
void safePowerOff();
void checkSafePowerOff();
void processUnsafePowerOff();
void processSwitchingActivationOfSafePowerOff();

int main(void) {

	CLKPR = _BV(CLKPCE);
	CLKPR = _BV(CLKPS2);// prescaler 16, for 1MHz FCPU

	configureIO();
	checkeEnableOfCurrentPresentationFeatureAtStartup();
	beep(); _delay_ms(150); beep();
	if (currentPresentationFeatureIsEnable) {
		_delay_ms(150); beep();
	}

	processSwitchingActivationOfSafePowerOff();
	checkSafePowerOff();

	configureTimer();

	while (1) {
		switch(state) {
		case alarm:
			precessAlarm();
			precessResetAlarm();
			break;
		case start_current_measuring:
			processEnableInput();
			processCurrentMeasurement();
			break;
		case current_measuring:
			break;
		case checking_alarm_conditions:
			checkAlarmConditions();
			break;
		case no_alarm:
			break;
		}
		safePowerOff();
		processHeartBeat();
	};
	return 0;
}

inline Bool isResetPinPressed() {
	return (PINB & _BV(ALARM_RESET)) ? false : true;
}

inline PinChangeState getPinChangeState() {
	Bool presentResetPinState = isResetPinPressed();
	PinChangeState state = stable_low;
	if (lastResetPinState == true && presentResetPinState == false) {
		state = falling_edge;
	} else if (lastResetPinState == false && presentResetPinState == true) {
		state = rising_edge;
	}else if (lastResetPinState == true && presentResetPinState == true) {
		state = stable_high;
	}
	lastResetPinState = presentResetPinState;
	return state;
}

void _delay_s(uint16_t sec)
{
	sec *= 100;
	for (uint16_t i=0; i<sec; i++)
	{
		_delay_ms(10);
	}
}

void processSwitchingActivationOfSafePowerOff() {
	enableSafePoweroOff = eeprom_read_byte(eeprom_addr_enable_safe_power_off);
	_delay_s(2);
	if (isResetPinPressed()) {
		eeprom_write_byte(eeprom_addr_enable_safe_power_off, !enableSafePoweroOff);
		enableSafePoweroOff = !enableSafePoweroOff;
		beep(); _delay_ms(150);
		if (enableSafePoweroOff) {
			_delay_ms(150); beep();
		}
	}
}

void safePowerOff() {
	if (enableSafePoweroOff) {
		switch (getPinChangeState()) {
		case rising_edge:
			eeprom_write_byte(eeprom_addr_safe_power_off, true);
			break;
		case falling_edge:
			eeprom_write_byte(eeprom_addr_safe_power_off, false);
			break;
		default:
			break;
		}
	}
}

void checkSafePowerOff() {
	if (enableSafePoweroOff && (false == eeprom_read_byte(eeprom_addr_safe_power_off))) {
		processUnsafePowerOff();
	}
}

void processUnsafePowerOff() {
	while (!isResetPinPressed()) {
		beep();
		_delay_ms(500);
	}
}

void checkeEnableOfCurrentPresentationFeatureAtStartup() {
	if (isResetPinPressed()) {
		_delay_ms(300);
		if (isResetPinPressed()) {
			currentPresentationFeatureIsEnable = true;
		}
	}
}

void presentCurrentLevelIfNeed(uint16_t currentValue) {
	if (currentPresentationFeatureIsEnable) {
		uint8_t ampere = currentValue/steps_per_ampere;
		for (uint8_t i=0; i<ampere; i++) {
			beep(); _delay_ms(100);
		}
	}
}

void beep() {
	for (uint16_t i = 0; i<50; i++) {
		PORTB ^= _BV(BUZZ); _delay_us(150);
	}
	PORTB &= ~_BV(BUZZ);
}

void processHeartBeat() {
	if (heartBeatCounter == 0) {
		if (state != alarm)
			beep();
		heartBeatCounter = heart_beat_period;
	}
}

void precessAlarm() {
	if (heartBeatCounter%2) {
		PORTB ^= _BV(BUZZ);
		_delay_us(180);
	}
	else {
		PORTB &= ~_BV(BUZZ);
	}
}

uint16_t findMinMaxDiff(volatile uint16_t* buf, uint32_t size) {
	uint16_t min = *buf, max = *buf;
	if (size > 0) {
		for (int i=0; i<size; i++) {
			if (buf[i] > max) max = buf[i];
			if (buf[i] < min) min = buf[i];
		}
	}
	return max-min;
}

void checkCurrentValue() {
	uint16_t Ipp = findMinMaxDiff(currentMonitorBuffer, samples_number);
	uint16_t I = Ipp/2;
	presentCurrentLevelIfNeed(I);
	if (I > current_threshold) {
		if (measurementsAboveThresholdCounter) measurementsAboveThresholdCounter--;
	}
	else {
		measurementsAboveThresholdCounter = seconds_of_current_measure;
	}
	if (measurementsAboveThresholdCounter == 0) {
		currentMonitorCounter = seconds_to_alarm;
	}
}

void checkAlarmConditions() {
	checkCurrentValue();
	if (currentMonitorCounter == 0) {
		state = alarm;
	}
	else {
		state = no_alarm;
	}
}

void processCurrentMeasurement() {
	if (enableState == current_monitoring_disabled) {
		state = no_alarm;
		return;
	}
	ADMUX = CURRENT_TRANSFORMER_ADC;
	_delay_ms(2);

	sei();
	ADCSRA |= _BV(ADIE) | _BV(ADATE); // ADC interrupt enable, start in free running mode
	ADCSRA |= _BV(ADSC);
	state = current_measuring;
}

uint16_t getEnableInputADCValue() {
	ADMUX = ENABLE_MONITOR_ADC;
	_delay_ms(2);
	ADCSRA |= _BV(ADSC); // Start conversion - first conversion may be not to accurate
	while (ADCSRA & _BV(ADSC)){} // wait for conversion
	ADCSRA |= _BV(ADSC);
	while (ADCSRA & _BV(ADSC)){}
	return ADC;
}

void acousticSignalOfCurrentMonitoringDisabled() {
	if (heartBeatCounter % 2) {
		beep();
	}
}

void processEnableInput() {
	uint16_t enableVal = getEnableInputADCValue();
	if (enableVal > enable_input_analog_threshold) {
		enableMonitorCounter = seconds_of_inactivity_to_disable_alarm;
		enableState = current_monitoring_enabled;
	}
	if (enableMonitorCounter == 0) {
		enableState = current_monitoring_disabled;
		acousticSignalOfCurrentMonitoringDisabled();
	}
}

void precessResetAlarm() {
	if (isResetPinPressed()) {
		_delay_ms(30);
		if (isResetPinPressed()) {
			currentMonitorCounter = seconds_to_alarm;
			enableMonitorCounter = seconds_of_inactivity_to_disable_alarm;
			state = no_alarm;
			PORTB &= ~_BV(BUZZ);
		}
	}
}

void configureIO() {
	MCUCR &= ~_BV(PUD);

	// BUZ: Dout
	DDRB |= _BV(BUZZ);

	// ADC current transformer: PB2
	// ADC enable monitor: PB4
	ADMUX |= (_BV(REFS1) | _BV(REFS2)); //Uref 2,56V without C
	ADCSRA |= (_BV(ADEN) | _BV(ADPS0) | _BV(ADPS1)); // enable ADC and prescaler 8
	// conversion time 13,5 ADCclk cycles

	// Alarm reset button: Din pullup
	PORTB |= _BV(ALARM_RESET);
	DDRB &= ~_BV(ALARM_RESET);
}

void configureTimer() {
	// Setup Timer1 to interrupt every 1s
	PLLCSR  &= ~_BV( PCKE);
	TCCR1 = 0;
	TCNT1 = 0;
	GTCCR = _BV(PSR1);
	OCR1A = 243;
	TIMSK = _BV(OCIE1A);
	// Start timer in CTC mode; prescaler = 4096;
	TCCR1 = _BV(CTC1) | _BV(CS13) | _BV(CS12) | _BV(CS10);
	sei();
}

inline void handleCounters() {
	if (enableState == current_monitoring_enabled && currentMonitorCounter) currentMonitorCounter--;
	if (enableMonitorCounter) enableMonitorCounter--;
	if (heartBeatCounter) heartBeatCounter--;
}

ISR(TIMER1_COMPA_vect) {
	handleCounters();
	if (state == no_alarm)
		state = start_current_measuring;
}

ISR(ADC_vect) {
	currentMonitorBuffer[--currentMonitorBufferIndex] = ADC;
	if (currentMonitorBufferIndex == 0) {
		ADCSRA &= ~(_BV(ADIE) | _BV(ADATE)); //stop ADC
		currentMonitorBufferIndex = samples_number;
		state = checking_alarm_conditions;
	}
}

