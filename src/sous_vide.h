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
const unsigned int HEATING = 3;
const unsigned int PAUSED = 4;

// global variables
unsigned int setpoint_temp_fahrenheit_g = 0u;
unsigned long start_cook_time_g = 0ul;
unsigned int cooking_duration_g = 0;
double duty_cycle_g = 0;
unsigned int state_g = CHANGE_TEMP;
Adafruit_LiquidCrystal lcd(0);

// data structures
enum EventType {
  SW1_PRESS,
  SW2_PRESS,
  NONE
};

struct Event {
    EventType type = EventType::NONE;
    bool valid = false;
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
String formatTime(unsigned long t_millis);

/**
 * @ return time formatted as ###°F
 */
String formatTemp(unsigned long temp_fahrenheit);

/**
 * For a given desired teperature, there is a duty cycle that acheives it.
 * We should use a reasonable PWM period fast enough to minimize fluctuations.
 * PID will be used to guide the duty cycle.
 *
 */
void control_heater(double current_temp);
