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

/**
 * @return temp in fahrenheit
 */
double analogToTemp(unsigned int thermistor_value) {
  return thermistor_value;
}

/**
 * @return temp in fahrenheit
 */
double potToTemp(unsigned int potentiometer_value) {
  constexpr static int minimum_temp = 100; // degrees F
  constexpr static int maximum_temp = 170; // degrees F
  return map(potentiometer_value, 417, 938, minimum_temp, maximum_temp);
}

/**
 * @return time in seconds
 */
unsigned int potToTime(unsigned int potentiometer_value) {
  constexpr static unsigned int minimum_time = 5 * 60; // X minutes
  constexpr static unsigned int maximum_time = 4 * 60 * 60; // X hours
  return map(potentiometer_value, 417, 938, minimum_time, maximum_time);
}

void control_heater(double current_temp) {
}

unsigned int global_state = CHANGE_TEMP;
unsigned int setpoint_temp_f = 0u;
unsigned long t = 0ul;
unsigned long start_cook_time = 0ul;
unsigned int cooking_duration = 0, temp = 0;

// we don't use the latch pin
Adafruit_LiquidCrystal lcd(DAT, CLK, 0);

void setup() {
  pinMode(RED_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(SW1, INPUT_PULLUP);
  pinMode(SW2, INPUT_PULLUP);
  pinMode(RELAY, OUTPUT);
  Wire.begin(DAT, CLK);
  pinMode(POT, OUTPUT);
  pinMode(THRM, OUTPUT);

  lcd.begin(16, 2);
  lcd.setBacklight(HIGH);
  lcd.setCursor(0, 1);

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
  digitalWrite(POT, HIGH);
  digitalWrite(THRM, LOW);
  unsigned int potentiometer_value = analogRead(A0);
  double pot_as_temp = potToTemp(potentiometer_value);
  Serial.println(potentiometer_value);
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
      lcd.setCursor(1, 0);
      lcd.print(String(pot_as_temp));

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
      lcd.setCursor(1, 0);
      lcd.print(String(cooking_duration));
      Serial.print("change time. ");
      Serial.print("time = ");
      Serial.println(cooking_duration);
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
      lcd.setCursor(1, 0);
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

  delay(20);
}
