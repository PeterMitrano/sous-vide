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
  unsigned int r = analogRead(A0);
  Serial.println(r);
  delay(1000);
}
