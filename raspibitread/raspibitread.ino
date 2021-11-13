void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Transmission Start");
  Serial.flush();
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0) {
    byte x = Serial.read();
    for (int i = 0; i < 8; i++) {
      Serial.print(bitRead(x,8-i));
      Serial.flush();
    }
  }
}
