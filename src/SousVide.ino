#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LiquidCrystal.h>

#include "sous_vide.h"
#include "event_handler.h"

unsigned int global_state = CHANGE_TEMP;
unsigned int setpoint_temp_f = 0u;
unsigned long t = 0ul;
Adafruit_LiquidCrystal lcd(0);
EventHandler event_handler;

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

  // set initial time
  t = millis();
}

// Start in the set temp state.
//
// CHANGE_TEMP
//  - turn potentiometer to set temperature
//  - press SW1 to go to CHANGE_TIME
//  - SW2 does nothing
//  - turn pot to set time
//
// CHANGE_TIME
//  - press SW1 to go to HEATING
//  - press SW2 to go back to setting time
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
  unsigned int pot = analogRead(A0);

  switch (global_state) {
    case CHANGE_TEMP:
      break;
    case CHANGE_TIME:
      break;
    case HEATING:
      break;
    case FINISHED:
    default:
      digitalWrite(RELAY, 0);
      break;
  }

}
