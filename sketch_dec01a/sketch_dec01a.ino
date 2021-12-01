void setup() {
  // put your setup code here, to run once:
  pinMode(2,INPUT);
  pinMode(5,INPUT);
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (digitalRead(5) == HIGH){
    Serial.println(analogRead(2));
  }
}
