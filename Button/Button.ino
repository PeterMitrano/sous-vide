int button = D8;


void setup() {
  Serial.begin(9600);
  pinMode(button, INPUT_PULLUP);
}


void loop() {
  if (digitalRead(button)) {
    Serial.println("on");  
  }
}
