#pragma once

#include <Arduino.h>
#include <Adafruit_LiquidCrystal.h>

// pins
const unsigned int RED_LED = D0;
const unsigned int POT = D1;
const unsigned int THRM = D2;
const unsigned int SW1 = D3;
const unsigned int GREEN_LED = D4;
const unsigned int SW2 = D5;
const unsigned int CLK = D6;
const unsigned int DAT = D7;
const unsigned int RELAY = D8;

// states
const unsigned int FINISHED = 0;
const unsigned int CHANGE_TEMP = 1;
const unsigned int CHANGE_TIME = 2;
const unsigned int PRE_HEATING = 3;
const unsigned int HEATING = 4;
const unsigned int PAUSED = 5;

// global variables
int current_temp_g = 0;
int setpoint_temp_fahrenheit_g = 0u;
unsigned long start_cooking_time_sec_g = 0ul;
unsigned int cooking_duration_sec_g = 0;
double duty_cycle_g = 0;
unsigned int state_g = CHANGE_TEMP;
Adafruit_LiquidCrystal lcd(0);

byte degree_char[8] = {
	0b00110,
	0b01001,
	0b01001,
	0b00110,
	0b00000,
	0b00000,
	0b00000,
	0b00000
};

// data structures
enum EventType {
  SW1_PRESS,
  SW2_PRESS,
  NONE
};

struct Event {
    EventType type = EventType::NONE;
};


/**
 * @return temp in fahrenheit
 */
double analogToTemp(unsigned int thermistor_value);

/**
 * @return temp in fahrenheit
 */
double potToTemp(unsigned int potentiometer_value);

/**
 * @return time in seconds
 */
unsigned int potToTime(unsigned int potentiometer_value);

/**
 * @ return time formatted as hh:mm
 */
String formatTime(unsigned long t_s);

/**
 * @ return time formatted as ###°F
 */
String formatTemp(unsigned int temp_fahrenheit);

/**
 * For a given desired teperature, there is a duty cycle that acheives it.
 * We should use a reasonable PWM period fast enough to minimize fluctuations.
 * PID will be used to guide the duty cycle.
 *
 */
void control_heater();
