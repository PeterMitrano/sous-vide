// Include math library for calculations
#include <math.h>

// Define ADC parameters
#define ADC_TOP 1023

// Define thermistor parameters
#define BETA 3977
#define LNA -4.12858298874828

// Define Reference Resistor
#define R_REF 1100

const unsigned int enable = D2;
const unsigned int led = D0;

void setup() {
  Serial.begin(115200);
  pinMode(enable, OUTPUT);
  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);
  digitalWrite(enable, HIGH);
}

void loop() {
  unsigned int raw = analogRead(A0);

  double x = (R_REF * raw) / (ADC_TOP - raw);
  x = BETA / (log(x) - LNA);
  x = x - 273.15;

  Serial.print(raw);
  Serial.print(" ");
  Serial.println(x);
  delay(1000);
}
