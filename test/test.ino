#include <Wire.h>

int i = 0;
bool enable = true;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  attachInterrupt(0,sw, RISING);
  randomSeed(analogRead(0));
  int a = 1;
}

void loop() {
  a++;
  Serial.print(a);
}

void sw() {
  enable = !enable;
}
