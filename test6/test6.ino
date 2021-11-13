#include <TimedAction.h>
int upper = 8;
int lower = 9;


void setup() {
  // put your setup code here, to run once:
  pinMode(upper, OUTPUT);
  pinMode(lower, OUTPUT);
  Serial.begin(9600);
  Serial.println("Put the angle");
}

int a2i(int num) {
  return num - 48;
}

void loop() {
  if (Serial.available() > 0) {
    int data = Serial.read();
    Serial.println(data);
    delay(500);
  }
}

void lop() {
  // put your main code here, to run repeatedly:
  while (Serial.available() > 10) {
    int size = Serial.available();
    byte *data;
    if ((data = (byte *) malloc(sizeof(byte) * size)) == NULL) {
      Serial.println("malloc error");
    } else {
      for (int i = 0; i < size; i++) {
        data[i] = Serial.read();
      }
      int servo1 = data[0];
      int degree1 = a2i(data[2]) * 100 + a2i(data[3]) * 10 + a2i(data[4]);
      int servo2 = data[6];
      int degree2 = a2i(data[8]) * 100 + a2i(data[9]) * 10 + a2i(data[10]);
      Serial.println(servo1);
      Serial.println(degree1);
      Serial.println(servo2);
      Serial.println(degree2);
      if (degree1 >= 180) {
        degree1 = 180;
      } else if (degree1 <= 0) {
        degree1 = 0;
      }
      if (degree2 >= 180) {
        degree2 = 180;
      } else if (degree2 <= 0) {
        degree2 = 0;
      }
      float pulse_width1 = (degree1 + 180 ) * 50 / 9;
      float pulse_width2 = (degree2 + 180) * 50 / 9;
      float sum_width = 20000;
      if (upper != -1) {
        digitalWrite(upper,HIGH);
        digitalWrite(lower,HIGH);
        int current_time = micros();
        if (degree1 <= degree2) {
        while (micros() - current_time <= pulse_width1);
        digitalWrite(upper,LOW);
        while (micros() - current_time <= pulse_width2);
        digitalWrite(lower,LOW);
        while (micros() - current_time <= sum_width);
        } else {
        while (micros() - current_time <= pulse_width2);
        digitalWrite(upper,LOW);
        while (micros() - current_time <= pulse_width1);
        digitalWrite(lower,LOW);
        while (micros() - current_time <= sum_width);
        }
      }
      else {
        int current_time = micros();
        digitalWrite(lower,HIGH);
        while (micros() - current_time <= pulse_width1);
        digitalWrite(lower,LOW);
        while (micros() - current_time <= sum_width);
      }  
    }
  }
}
