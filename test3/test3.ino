void setup() {
  // put your setup code here, to run once:
  #define OUT1 7
  #define OUT2 6
  #define IN 8
  #define SPEED 500
  pinMode(OUT1, OUTPUT);
  pinMode(OUT2, OUTPUT);
  pinMode(IN, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  while (digitalRead(IN) == HIGH) {
    digitalWrite(OUT1,HIGH);
    digitalWrite(OUT2,HIGH);
    delayMicroseconds(SPEED);
    digitalWrite(OUT1,LOW);
    digitalWrite(OUT2,LOW);
    delayMicroseconds(SPEED);
  }
}
