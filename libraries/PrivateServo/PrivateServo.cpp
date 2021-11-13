#include "PrivateServo.h"

PrivateServo::PrivateServo() {
}

void PrivateServo::setup(int servo) {
  active = true;
  pin1 = servo;
  degree1 = 0;
  pin2 = -1;
  degree2 = 0;
  pinMode(pin1,OUTPUT);
}

void PrivateServo::setup(int servo1, int servo2) {
  active = true;
  pin1 = servo1;
  degree1 = 0;
  pin2 = servo2;
  degree2 = 0;
  pinMode(pin1,OUTPUT);
  pinMode(pin2,OUTPUT);
}

void PrivateServo::setup(int servo, float dgr) {
  active = true;
  pin1 = servo;
  degree1 = dgr;
  pin2 = -1;
  degree2 = 0;
  pinMode(pin1,OUTPUT);
}

void PrivateServo::setup(int servo1, float dgr1, int servo2, float dgr2) {
  active = true;
  pin1 = servo1;
  degree1 = dgr1;
  pin2 = servo2;
  degree2 = dgr2;
  pinMode(pin1,OUTPUT);
  pinMode(pin2,OUTPUT);
}

void PrivateServo::setDegree1(float dgr1){
  degree1 = dgr1;
}

void PrivateServo::setDegree2(float dgr2){
  degree2 = dgr2;
}

void PrivateServo::resetDegree(){
  degree1 = 90;
  degree2 = 90;
}

void PrivateServo::setDegrees(float dgr1, float dgr2){
  degree1 = dgr1;
  if (pin2 != -1) {
    degree2 = dgr2;
  }
}

void PrivateServo::disable(){
  active = false;
}

void PrivateServo::enable(){
  active = true;
}

void PrivateServo::work(){
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
  pulse_width1 = (degree1 + 180 ) * 50 / 9;
  pulse_width2 = (degree2 + 180) * 50 / 9;
  sum_width = 20000;
  if (pin2 != -1) {
    digitalWrite(pin1,HIGH);
    digitalWrite(pin2,HIGH);
    current_time = micros();
    if (degree1 <= degree2) {
      while (micros() - current_time <= pulse_width1);
      digitalWrite(pin1,LOW);
      while (micros() - current_time <= pulse_width2);
      digitalWrite(pin2,LOW);
      while (micros() - current_time <= sum_width);
    } else {
      while (micros() - current_time <= pulse_width2);
      digitalWrite(pin2,LOW);
      while (micros() - current_time <= pulse_width1);
      digitalWrite(pin1,LOW);
      while (micros() - current_time <= sum_width);
    }
  } else {
    current_time = micros();
    digitalWrite(pin1,HIGH);
    while (micros() - current_time <= pulse_width1);
    digitalWrite(pin1,LOW);
    while (micros() - current_time <= sum_width);
  }    
}