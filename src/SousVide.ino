#include <Arduino.h>
#include <Wire.h>

const unsigned int RED_LED = D0;
const unsigned int POT = D1;
const unsigned int THRM = D2;
const unsigned int SW1 = D3;
const unsigned int BLUE_LED = D4;
const unsigned int SW2 = D5;
const unsigned int CLK = D6;
const unsigned int DAT = D7;
const unsigned int RELAY = D8;

unsigned long t = 0;
bool relay_state = false;

void setup() {
  pinMode(RED_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(SW1, INPUT_PULLUP);
  pinMode(SW2, INPUT_PULLUP);
  pinMode(RELAY, OUTPUT);
  Wire.begin(DAT, CLK);
  pinMode(POT, OUTPUT);
  pinMode(THRM, OUTPUT);

  Serial.begin(9600);
  Serial.println("Setup.");

  t = millis();
}

void loop() {
  if (!digitalRead(SW1)) {
    digitalWrite(THRM, HIGH);
    digitalWrite(POT, LOW);
    digitalWrite(RED_LED, HIGH);
    unsigned int thm = analogRead(A0);
    Serial.print("sw1 ");
    Serial.print("thm: ");
    Serial.println(thm);

  }
  else if (!digitalRead(SW2)) {
    digitalWrite(THRM, LOW);
    digitalWrite(POT, HIGH);
    digitalWrite(BLUE_LED, HIGH);
    unsigned int pot = analogRead(A0);
    Serial.print("sw2 ");
    Serial.print("pot: ");
    Serial.println(pot);

  }
  else if (digitalRead(SW1) && digitalRead(SW2)) {
    // neither pressed
    digitalWrite(BLUE_LED, LOW);
    digitalWrite(RED_LED, LOW);
    digitalWrite(THRM, LOW);
    digitalWrite(POT, LOW);
    Serial.print("no analog ");
    Serial.println(analogRead(A0));
  }
  else {
    Serial.println("shouldn't be possible?");
  }

  Wire.beginTransmission(0x0);
  Wire.write(0xAA);
  Wire.endTransmission(true);

  unsigned long now = millis();
  if (now - t > 5000) {
    t = now;
    relay_state = !relay_state;
  }

  digitalWrite(RELAY, relay_state);

  delay(100);
}
