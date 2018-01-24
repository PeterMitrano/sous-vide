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
unsigned int setpoint_temp_f = 0u;
unsigned long t = 0ul;
unsigned long start_cook_time = 0ul;
unsigned int cooking_duration = 0;
unsigned int temp = 0;
double duty_cycle = 0;
unsigned int global_state = CHANGE_TEMP;
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
 * For a given desired teperature, there is a duty cycle that acheives it.
 * We should use a reasonable PWM period fast enough to minimize fluctuations.
 * PID will be used to guide the duty cycle.
 *
 */
void control_heater(double current_temp);

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
