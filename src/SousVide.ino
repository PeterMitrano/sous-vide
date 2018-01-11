#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LiquidCrystal.h>

#include "sous_vide.h"

enum EventType {
  SW1_PRESS,
  SW2_PRESS,
  NONE
};

struct Event {
    EventType type = EventType::NONE;
    bool valid = false;
};

Event checkForEvent() {
  static int sw1_c= 0;
  static int sw2_c= 0;

  Event event;
  if (!digitalRead(SW1)) {
    sw1_c++;
  }
  else if (digitalRead(SW1)) {
    if (sw1_c > 5) {
      event.valid = true;
      event.type = SW1_PRESS;
    }
    sw1_c = 0;
  }

  if (!digitalRead(SW2)) {
    sw2_c++;
  }
  else if (digitalRead(SW2)) {
    if (sw2_c > 5) {
      event.valid = true;
      event.type = SW2_PRESS;
    }
    sw2_c = 0;
  }
  return event;
}

unsigned int global_state = CHANGE_TEMP;
unsigned int setpoint_temp_f = 0u;
unsigned long t = 0ul;
unsigned long start_cook_time = 0ul;
unsigned int cooking_duration = 0, temp = 0;
double duty_cycle = 0;

// we don't use the latch pin
Adafruit_LiquidCrystal lcd(0);

void setup() {
  pinMode(RED_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(SW1, INPUT_PULLUP);
  pinMode(SW2, INPUT_PULLUP);
  pinMode(RELAY, OUTPUT);
  Wire.begin(DAT, CLK);
  pinMode(POT, OUTPUT);
  pinMode(THRM, OUTPUT);

  Wire.begin(DAT, CLK);
  lcd.begin(16, 2);
  lcd.clear();
  lcd.setBacklight(HIGH);
  lcd.setCursor(0, 0);

  Serial.begin(9600);
  Serial.println("Setup.");

  // set initial cooking_duration
  t = millis();
}

// Start in the set temp state.
//
// CHANGE_TEMP
//  - turn potentiometer to set temperature
//  - press SW1 to go to CHANGE_TIME
//  - SW2 does nothing
//  - turn pot to set cooking_duration
//
// CHANGE_TIME
//  - press SW1 to go to HEATING
//  - press SW2 to go back to setting cooking_duration
//
// HEATING
//  - start count-down timer
//  - SW1 does nothing
//  - press SW2 to go to PAUSE
//  - pot does nothing
//
// PAUSE
//  - press SW1 to resume, go to HEATING
//  - press SW2 to confirm cancel
//  - pot does nothing
//
// FINISHED
//  - if the timer finishes, turn off the heater
//  - none of the controls do anything
void loop() {
  static unsigned long last_event_time = 0ul;
  unsigned long now = millis();
  if (now - last_event_time > 20) {
    last_event_time = now;
    digitalWrite(POT, HIGH);
    digitalWrite(THRM, LOW);
    static unsigned int potentiometer_value = 0;
    potentiometer_value = 0.8 * potentiometer_value + 0.2 * analogRead(A0);
    double pot_as_temp = potToTemp(potentiometer_value);
    digitalWrite(POT, LOW);
    digitalWrite(THRM, HIGH);
    unsigned int thermistor_value = analogRead(A0);
    double current_temp = analogToTemp(thermistor_value);
    digitalWrite(POT, LOW);
    digitalWrite(THRM, LOW);

    Event evt = checkForEvent();

    switch (global_state) {
      case CHANGE_TEMP:
        digitalWrite(RELAY, LOW);
        digitalWrite(BLUE_LED, LOW);
        digitalWrite(RED_LED, LOW);

        if (evt.valid && evt.type == EventType::SW1_PRESS) {
          Serial.println("going to CHANGE_TIME.");
          global_state = CHANGE_TIME;
        }

        setpoint_temp_f = pot_as_temp;

        lcd.setCursor(0, 0);
        lcd.print("Temp: ");
        lcd.setCursor(0, 1);
        lcd.print(String(pot_as_temp) + String("     "));

        Serial.print("change temp. ");
        Serial.print("temp = ");
        Serial.println(pot_as_temp);
        break;

      case CHANGE_TIME:
        digitalWrite(RELAY, LOW);
        digitalWrite(BLUE_LED, LOW);
        digitalWrite(RED_LED, LOW);
        if (evt.valid && evt.type == EventType::SW1_PRESS) {
          Serial.println("going to HEATING.");
          global_state = HEATING;
          start_cook_time = millis();
        }
        else if (evt.valid && evt.type == EventType::SW2_PRESS) {
          global_state = CHANGE_TEMP;
        }

        cooking_duration = potToTime(potentiometer_value);
        lcd.setCursor(0, 0);
        lcd.print("Time: ");
        lcd.setCursor(0, 1);
        {
          String duration_string = String(cooking_duration/(60*60)) + ":" + String(cooking_duration/60%60) + String("     ");
          lcd.print(duration_string);
          Serial.print("change time. ");
          Serial.print("time = ");
          Serial.println(duration_string);
        }
        break;

      case HEATING:
        digitalWrite(BLUE_LED, LOW);
        digitalWrite(RED_LED, HIGH);
        if (evt.valid && evt.type == EventType::SW2_PRESS) {
          Serial.println("going to PAUSED.");
          global_state = PAUSED;
        }

        lcd.setCursor(0, 0);
        lcd.print("status... ");
        lcd.setCursor(0, 1);
        lcd.print(String(current_temp));
        lcd.print("    ");
        lcd.print(String(millis() - start_cook_time));

        control_heater(current_temp);

        if (millis() - start_cook_time >= cooking_duration) {
          Serial.println("going to finished.");
          digitalWrite(BLUE_LED, HIGH);
          digitalWrite(RED_LED, LOW);
          global_state = FINISHED;
        }

        Serial.println("heating.");
        break;

      case PAUSED:
        digitalWrite(RELAY, LOW);
        if (evt.valid && evt.type == EventType::SW1_PRESS) {
          Serial.println("going to heating.");
          global_state = HEATING;
        }
        else if (evt.valid && evt.type == EventType::SW2_PRESS) {
          Serial.println("going to finished.");
          global_state = FINISHED;
        }

        Serial.println("paused.");
        break;

      case FINISHED:
        Serial.println("FINISHED.");
        // [[fallthrough]]
      default:
        digitalWrite(BLUE_LED, LOW);
        digitalWrite(RED_LED, LOW);
        digitalWrite(RELAY, LOW);
        break;
    }
  }

  // Software PWM of relay
  static unsigned long pwm_t0 = 0ul;
  static constexpr double pwm_period_ms = 10000;
  digitalWrite(RELAY, (now - pwm_t0 < duty_cycle * pwm_period_ms));
  if (now - pwm_t0 > pwm_period_ms) {
    pwm_t0 = now;
  }
}

/**
 * @return temp in fahrenheit
 */
double analogToTemp(unsigned int thermistor_value) {
  static constexpr int ADC_TOP = 1023;
  static constexpr int BETA = 3977;
  static constexpr double LNA = -4.12858298874828;
  static constexpr int R_REF = 1100;
  double temp = (R_REF * thermistor_value) / (ADC_TOP - thermistor_value);
  temp = BETA / (log(temp) - LNA);
  temp = temp - 273.15;
  return temp;
}

/**
 * @return temp in fahrenheit
 */
double potToTemp(unsigned int potentiometer_value) {
  static constexpr int minimum_temp = 100; // degrees F
  static constexpr int maximum_temp = 170; // degrees F
  return map(potentiometer_value, 417, 938, minimum_temp, maximum_temp);
}

/**
 * @return time in seconds
 */
unsigned int potToTime(unsigned int potentiometer_value) {
  static constexpr unsigned int minimum_time = 5; // X minutes
  static constexpr unsigned int maximum_time = 4 * 60; // X hours
  static constexpr unsigned int intervals = (maximum_time - minimum_time) / 5;
  return (minimum_time + 5 * map(potentiometer_value, 420, 930, 0, intervals)) * 60;
}

/**
 * For a given desired teperature, there is a duty cycle that acheives it.
 * We should use a reasonable PWM period fast enough to minimize fluctuations.
 * PID will be used to guide the duty cycle.
 *
 */
void control_heater(double current_temp) {
  static constexpr double kP = 0.0;
  static constexpr double kI = 0.0;
  static constexpr double kD = 0.0;
  static double integral = 0;

  double error = setpoint_temp_f - current_temp;

  static double last_error = error;

  integral += error;

  double derivative = error - last_error;
  duty_cycle = kP * error + kI * integral + kD * derivative;

  // enforce limit on duty cycle
  duty_cycle = fmax(fmin(duty_cycle, 1), 0);

  last_error = error;

}
