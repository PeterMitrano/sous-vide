const unsigned int pot_enable = D1;
const unsigned int thermistor_enable = D2;
const unsigned int button = D3;

void setup() {
  Serial.begin(115200);
  pinMode(thermistor_enable, OUTPUT);
  pinMode(pot_enable, OUTPUT);
  pinMode(button, INPUT_PULLUP);
  digitalWrite(thermistor_enable, LOW);
  digitalWrite(pot_enable, LOW);
}

void loop() {
  unsigned int r = analogRead(A0);

  if (!digitalRead(button)) {
    digitalWrite(pot_enable, HIGH);
    digitalWrite(thermistor_enable, LOW);
    Serial.print("pot: ");
  }
  else {
    digitalWrite(pot_enable, LOW);
    digitalWrite(thermistor_enable, HIGH);
    Serial.print("thm: ");
  }
  
  Serial.println(r);
  delay(100);
}
