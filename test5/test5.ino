#include <TimedAction.h>
#include <PrivateServo.h>
int upper = 8;
int lower = 9;

PrivateServo PServo = PServos(8, 9);

void setup() {
  // put your setup code here, to run once:
  pinMode(upper, OUTPUT);
  pinMode(lower, OUTPUT);
  Serial.begin(9600);
  Serial.println("Put the angle")
}

void loop() {
  // put your main code here, to run repeatedly:
  while (Serial.available() > 10) {
    int size = Serial.available();
    byte *data;
    if ((data = (byte *) malloc(sizeof(byte) * size)) == NULL) {
      Serial.println("malloc error");
    } else {
      for (i = 0; i < size; i++) {
        data[i] = Serial.read()
      }
      int servo1 = data[0];
      int degree1 = a2i(data[2]) * 100 + a2i(data[3]) * 10 + a2i(data[4]);
      int servo2 = data[6];
      int degree1 = a2i(data[7]) * 100 + a2i(data[8]) * 10 + a2i(data[9]);
      Serial.println(servo1);
      Serial.println(degree1);
      Serial.println(servo2);
      Serial.println(degree2);
      PServo.setDegrees(degree1, degree2);
      PServo.work();
    }
  }
}

int a2i(int num) {
  return num - 48;
}
