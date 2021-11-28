#include "Stepping.h"

int cw_r=8;
int cw_l=9;
int ccw_r=10;
int ccw_l=11;
int awo_r=5;
int awo_l=6;
float base_step = 0.018;
float angvel=0.36;//deg/ms 14.4
unsigned long t=1000;
float intrvl = base_step / angvel;
Stepping Step = Stepping();



void setup() {
  // put your setup code here, to run once:
  pinMode(cw_r,OUTPUT);
  pinMode(cw_l,OUTPUT);
  pinMode(ccw_r,OUTPUT);
  pinMode(ccw_l,OUTPUT);
  pinMode(awo_r,OUTPUT);
  pinMode(awo_l,OUTPUT);
  Serial.begin(9600);
  Serial.println("Transmission Start");
  Step.setup(cw_r,cw_l,ccw_r,ccw_l,awo_r,awo_l);
  Step.set_step_degree(0.018);
  Step.move_forward_t(angvel,t);
  delay(10);
  Step.move_backward_t(angvel,t);
//  for (int i=0; i<=1000;i++){
//    digitalWrite(ccw_r,LOW);
//    digitalWrite(cw_l,LOW);
//    digitalWrite(cw_r,HIGH);
//    digitalWrite(ccw_l,HIGH);
//    digitalWrite(awo_r, LOW);
//    digitalWrite(awo_l, LOW);
//    delayMicroseconds((unsigned long)(intrvl*1000));
//    digitalWrite(cw_r,LOW);
//    digitalWrite(ccw_l,LOW);
//    delayMicroseconds((unsigned long)(intrvl*1000));
//  }
  delay(10);
  Step.move_forward_a(angvel,angvel*t);
  delay(10);
  Step.move_backward_a(angvel,angvel*t);
  delay(10);
//  Step.move_right_t(angvel*2,t*2,angvel/2,t/2);
//  delay(10);
//  Step.move_left_t(angvel*2,t*2,angvel/2,t/2);
}

void loop() {
}
