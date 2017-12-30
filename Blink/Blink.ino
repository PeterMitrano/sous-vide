int led = D4;


void setup() {
  Serial.begin(9600);
  pinMode(led, OUTPUT);
  Serial.println(led);
  digitalWrite(led, LOW);
  delay(3000);
}


void loop() {
  digitalWrite(led, HIGH);
  delay(500);
  digitalWrite(led, LOW);
  delay(500);
  Serial.println(".");
}
