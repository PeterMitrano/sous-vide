const unsigned int enable = D2;

void setup() {
  Serial.begin(115200);
  pinMode(enable, OUTPUT);
  digitalWrite(enable, HIGH);
}

void loop() {
  unsigned int r = analogRead(A0);
  Serial.println(r);
  delay(1000);
}
