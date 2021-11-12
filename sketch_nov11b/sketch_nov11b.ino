void setup() {
  // put your setup code here, to run once:
  pinMode(5,OUTPUT);
  pinMode(4,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  
  digitalWrite(5,LOW);
  digitalWrite(4,HIGH);
  delayMicroseconds(250);
  digitalWrite(5,LOW);
  digitalWrite(4,LOW);
  delayMicroseconds(250);
}
