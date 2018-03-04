#pragma once

#include <deque>
#include <Arduino.h>
#include <Adafruit_LiquidCrystal.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>

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
const unsigned int SCAN_SSID = 0;
const unsigned int SET_SSID = 1;
const unsigned int SET_PASSWORD = 2;
const unsigned int WAITING_FOR_CONNECTION = 3;
const unsigned int CHANGE_TEMP = 4;
const unsigned int CHANGE_TIME = 5;
const unsigned int PRE_HEATING = 6;
const unsigned int HEATING = 7;
const unsigned int PAUSED = 8;
const unsigned int FINISHED = 9;

// global variables
extern int current_temp_g;
extern int setpoint_temp_fahrenheit_g;
extern int time_left_sec_g;
extern ESP8266WebServer server;
extern std::deque<int> temps;

// data structures
enum EventType {
  SW1_PRESS,
  SW2_PRESS,
  SW1_LONG_PRESS,
  SW2_LONG_PRESS,
  NONE
};

struct Event {
    EventType type = EventType::NONE;
};


/**
 * @return temp in fahrenheit
 */
double analogToTemp(const unsigned int thermistor_value);

/**
 * @return temp in fahrenheit
 */
double potToTemp(const unsigned int potentiometer_value);

/**
 * @return time in seconds
 */
unsigned int potToTime(const unsigned int potentiometer_value);

/**
 * @ return time formatted as hh:mm
 */
String formatTime(const unsigned long t_s);

/**
 * @ return time formatted as ###°F
 */
String formatTemp(const unsigned int temp_fahrenheit);

/**
 * For a given desired teperature, there is a duty cycle that acheives it.
 * We should use a reasonable PWM period fast enough to minimize fluctuations.
 * PID will be used to guide the duty cycle.
 *
 */
void control_heater();

