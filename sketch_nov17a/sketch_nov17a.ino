#include "Stepping.h"

const int cw_r=8;
const int cw_l=9;
const int ccw_r=10;
const int ccw_l=11;
const float angvel=14.4;
const long t=10000;
float intrvl = 0.18 / angvel;
Stepping Step = Stepping();



void setup() {
  // put your setup code here, to run once:
  pinMode(cw_r,OUTPUT);
  pinMode(cw_l,OUTPUT);
  pinMode(ccw_r,OUTPUT);
  pinMode(ccw_l,OUTPUT);
  Serial.begin(9600);
  Serial.println("Transmission Start");
  Step.setup(cw_r,cw_l,ccw_r,ccw_l);
  Step.move_forward_t(angvel,t);
  delay(10);
  Step.move_backward_t(angvel,t);
  delay(10);
  Step.move_forward_t(angvel,t);
  delay(10);
  Step.move_right_t(angvel*2,t*2,angvel/2,t);
  delay(10);
  Step.move_left_t(angvel*2,t*2,angvel/2,t);
}

void loop() {
}
