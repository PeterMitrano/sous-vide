#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LiquidCrystal.h>

#include "sous_vide.h"

enum EventType {
  SW1_PRESS,
  SW2_PRESS,
  NONE
};

class Event {
  public:
    EventType type;

    Event() {
      valid = false;
      type = EventType::NONE;
    }

    explicit operator bool() {
      return valid;
    }


  private:
    bool valid;
};

Event waitForEvent() {
  Event event;
  return event;
}

unsigned int potToTemp(unsigned int potentiometer_value) {}
unsigned int potToTime(unsigned int potentiometer_value) {}
void control_heater() {}

unsigned int global_state = CHANGE_TEMP;
unsigned int setpoint_temp_f = 0u;
unsigned long t = 0ul;
unsigned long start_cook_time = 0ul;
unsigned int cooking_duration = 0, temp = 0;
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
  unsigned int potentiometer_value = analogRead(A0);

  Event evt = waitForEvent();

  if (!evt) {
    return;
  }

  switch (global_state) {
    case CHANGE_TEMP:
      digitalWrite(RELAY, LOW);
      digitalWrite(BLUE_LED, LOW);
      digitalWrite(RED_LED, LOW);
      if (evt.type == EventType::SW1_PRESS) {
        global_state = CHANGE_TIME;
      }

      temp = potToTemp(potentiometer_value);
      lcd.setCursor(0, 0);
      lcd.print("Temp: ");
      lcd.setCursor(1, 0);
      lcd.print(String(temp));
      break;

    case CHANGE_TIME:
      digitalWrite(RELAY, LOW);
      digitalWrite(BLUE_LED, LOW);
      digitalWrite(RED_LED, LOW);
      if (evt.type == EventType::SW1_PRESS) {
        global_state = HEATING;
        start_cook_time = millis();
      }
      else if (evt.type == EventType::SW2_PRESS) {
        global_state = CHANGE_TEMP;
      }

      cooking_duration = potToTime(potentiometer_value);
      lcd.setCursor(0, 0);
      lcd.print("Time: ");
      lcd.setCursor(1, 0);
      lcd.print(String(cooking_duration));
      break;

    case HEATING:
      digitalWrite(BLUE_LED, LOW);
      digitalWrite(RED_LED, HIGH);
      if (evt.type == EventType::SW2_PRESS) {
        global_state = PAUSED;
      }

      lcd.setCursor(0, 0);
      lcd.print("status... ");
      lcd.setCursor(1, 0);
      lcd.print(String(cooking_duration) + "    " + String(millis() - start_cook_time));

      control_heater();

      if (millis() - start_cook_time >= cooking_duration) {
        digitalWrite(BLUE_LED, HIGH);
        digitalWrite(RED_LED, LOW);
        global_state = FINISHED;
      }

      break;

    case PAUSED:
      digitalWrite(RELAY, LOW);
      if (evt.type == EventType::SW1_PRESS) {
        global_state = HEATING;
      }
      else if (evt.type == EventType::SW2_PRESS) {
        global_state = FINISHED;
      }


    case FINISHED:
      // [[fallthrough]]
    default:
      digitalWrite(BLUE_LED, LOW);
      digitalWrite(RED_LED, LOW);
      digitalWrite(RELAY, LOW);
      break;
  }

}
