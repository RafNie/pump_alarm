/*
 * main.c
 *
 *  Created on: 11 sty 2022
 *      Author: Rafał Niedźwiedziński
 */
#include <avr/io.h>
#include <inttypes.h>
#include <stdlib.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>

enum Alarm {
	alarm,
	start_current_measuring,
	current_measuring,
	checking_alarm_conditions,
	no_alarm
};

enum Enable {
	off,
	on
};

// Pins definitions
#define BUZZ (PB1)
//#define LED (PB1)
#define ALARM_RESET (PB0)
#define CURRENT_TRANSFORMER_ADC (_BV(MUX0))
#define ENABLE_MONITOR_ADC (_BV(MUX1))

//User definitions:
//
// seconds without current values above threshold to turn on alarm
#define seconds_to_alarm 20
//(2*24*60*60)
// seconds without values of enable input above the threshold to disable alarm counter
#define seconds_of_inactivity_to_disable_alarm 10
// (24*60*60)
// threshold enable input: ~ 1/8 Ucc=5V in ADC steps
#define enable_monitor_analog_threshold 256
// current transformer ratio in mV/A
#define current_transformet_ratio 100
// RMS current threshold in mA
#define rms_current_alarm_threshold 3000L


// Don't edit >>
// Alarm threshold calculation (fixed point):
#define fixed_point 1000
#define root_squere_of_two 1414
#define max_current_threshold ((rms_current_alarm_threshold*root_squere_of_two)/fixed_point)
#define voltage_threshold ((max_current_threshold*current_transformet_ratio)/fixed_point)
#define adc_steps 1024
#define adc_Vref 2560
#define alarm_threshold ((adc_steps*voltage_threshold)/adc_Vref)
// <<

#define measurements_above_threshold 7
#define hart_beate_threshold 10
#define samples_number 167


enum Enable enableState = on; // switched by the ADC enable input
volatile enum Alarm alarm_state = no_alarm;
volatile uint32_t currentMonitorCounter = seconds_to_alarm;
volatile uint32_t enableMonitorCounter = seconds_of_inactivity_to_disable_alarm;
volatile uint16_t currentMonitorBuffer[samples_number];
volatile uint8_t currentMonitorBufferIndex = samples_number - 1;
volatile uint8_t hartBeatCounter = hart_beate_threshold;
uint8_t measurementsAboveThresholdCounter = measurements_above_threshold;

void _delay_s(uint16_t sec);
void handleCounters();
void configureTimer();
void configureIO();
void precessAlarm();
void precessResetAlarm();
void procesEnableInput();
void processCurrentMonitorInput();
void checkCurrentValue();
void checkAlarmConditions();
void precessHartReate();
void beep();

int main(void) {

	CLKPR = _BV(CLKPCE);
	CLKPR = _BV(CLKPS2);// prescaler 16

	configureIO();
	//enable watchdog
	beep(); _delay_ms(100); beep();
	configureTimer();

	while (1) {
		switch(alarm_state) {
		case alarm:
			precessAlarm();
			precessResetAlarm();
			break;
		case start_current_measuring:
			procesEnableInput();
			processCurrentMonitorInput();
			break;
		case current_measuring:
			break;
		case checking_alarm_conditions:
			checkAlarmConditions();
			break;
		case no_alarm:
			break;
		}
		precessHartReate();
	};
	return 0;
}

void beep() {
	for (uint16_t i = 0; i<70; i++) {
		PORTB ^= _BV(BUZZ); _delay_us(150);
	}
	PORTB &= ~_BV(BUZZ);
}

void precessHartReate() {
	if (hartBeatCounter == 0) {
		if (alarm_state != alarm)
			beep();
		hartBeatCounter = hart_beate_threshold;
	}
}

void precessAlarm() {
	if (hartBeatCounter%2) {
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
		for (int i=1; i<size; i++) {
			if (buf[i] > max) max = buf[i];
			if (buf[i] < min) min = buf[i];
		}
	}
	return max-min;
}

void checkCurrentValue() {
	uint16_t Ipp = findMinMaxDiff(currentMonitorBuffer, samples_number - 1);
	if (Ipp/2 > alarm_threshold) {
		measurementsAboveThresholdCounter--;
	}
	else {
		measurementsAboveThresholdCounter = measurements_above_threshold;
	}
	if (measurementsAboveThresholdCounter == 0) {
		currentMonitorCounter = seconds_to_alarm;
	}
}

void checkAlarmConditions() {
	checkCurrentValue();
	if (currentMonitorCounter == 0) {
		alarm_state = alarm;
	}
	else {
		alarm_state = no_alarm;
	}
}

void processCurrentMonitorInput() {
	if (enableState == off) {
		alarm_state = no_alarm;
		return;
	}
	ADMUX = CURRENT_TRANSFORMER_ADC;
	_delay_ms(2);
	sei();
	ADCSRA |= _BV(ADIE) | _BV(ADATE);// ADC interrupt enable, start in free running mode
	ADCSRA |= _BV(ADSC);
	alarm_state = current_measuring;
}

uint16_t getEnableInputADC() {
	ADMUX = ENABLE_MONITOR_ADC;
	_delay_ms(2);
	ADCSRA |= _BV(ADSC); // Start conversion - first conversion may be not to accurate
	while (ADCSRA & _BV(ADSC)){} // wait for conversion
	ADCSRA |= _BV(ADSC);
	while (ADCSRA & _BV(ADSC)){}
	return ADC;
}

void procesEnableInput() {
	uint16_t enableVal = getEnableInputADC();
	if (enableVal > enable_monitor_analog_threshold) {
		enableMonitorCounter = seconds_of_inactivity_to_disable_alarm;
		enableState = on;
	}
	if (enableMonitorCounter == 0) {
		enableState = off;
	}
}

inline uint8_t isResetPinPressed() {
	return (PINB & _BV(ALARM_RESET)) ? 0 : 1;
}

void precessResetAlarm() {
	if (isResetPinPressed()) {
		_delay_ms(30);
		if (isResetPinPressed()) {
			currentMonitorCounter = seconds_to_alarm;
			enableMonitorCounter = seconds_of_inactivity_to_disable_alarm;
			alarm_state = no_alarm;
			PORTB &= ~_BV(BUZZ);
			// or reset MCU
		}
	}
}

void configureIO() {
	MCUCR &= ~_BV(PUD);

	// BUZ: Dout
	// LED: Dout
	DDRB |= _BV(BUZZ);// | _BV(LED);
	//PORTB |= _BV(LED);

	// ADC current transformer: PB2
	// ADC enable monitor: PB4
	ADMUX |= (_BV(REFS1) | _BV(REFS2)); //Uref 2,56V without C
	ADCSRA |= (_BV(ADEN) | _BV(ADPS0) | _BV(ADPS1) | _BV(ADPS2)); // enable ADC and prescaler 128

	// Alarm reset: Din pullup
	PORTB |= _BV(ALARM_RESET);
	DDRB &= ~_BV(ALARM_RESET);
}

void _delay_s(uint16_t sec)
{
	sec *= 100;
	for (uint16_t i=0; i<sec; i++)
	{
		_delay_ms(10);
	}
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
	if (enableState == on && currentMonitorCounter) currentMonitorCounter--;
	if (enableMonitorCounter) enableMonitorCounter--;
	if (hartBeatCounter) hartBeatCounter--;
}

ISR(TIMER1_COMPA_vect) {
	handleCounters();
	if (alarm_state == no_alarm)
		alarm_state = start_current_measuring;
}

ISR(ADC_vect) {
	currentMonitorBuffer[currentMonitorBufferIndex--] = ADC;
	if (currentMonitorBufferIndex == 0) {
		ADCSRA &= ~(_BV(ADIE) | _BV(ADATE)); //stop ADC
		currentMonitorBufferIndex = samples_number - 1;
		alarm_state = checking_alarm_conditions;
	}
}





