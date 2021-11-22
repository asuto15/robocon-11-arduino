#include "Stepping.h"

Stepping::Stepping(){
}

void Stepping::setup(int cw1, int cw2, int ccw1, int ccw2, int awo1, int awo2){
  cw_r = cw1;
  cw_l = cw2;
  ccw_r = ccw1;
  ccw_l = ccw2;
  lock_r = awo1;
  lock_l = awo2;
  pinMode(cw_r,OUTPUT);
  pinMode(cw_l,OUTPUT);
  pinMode(ccw_r,OUTPUT);
  pinMode(ccw_l,OUTPUT);
  pinMode(lock_r,OUTPUT);
  pinMode(lock_l,OUTPUT);
}

void Stepping::set_step_degree(float deg){
  step_degree = deg;
}

void Stepping::forward(float intrvl){//ms
  digitalWrite(cw_r,LOW);
  digitalWrite(ccw_l,LOW);
  digitalWrite(ccw_r,HIGH);
  digitalWrite(cw_l,HIGH);
  digitalWrite(lock_r, LOW);
  digitalWrite(lock_l, LOW);
  delayMicroseconds((unsigned long)(intrvl*1000));
  digitalWrite(ccw_r,LOW);
  digitalWrite(cw_l,LOW);
  delayMicroseconds((unsigned long)(intrvl*1000));
}

void Stepping::backward(float intrvl){//ms
  digitalWrite(ccw_r,LOW);
  digitalWrite(cw_l,LOW);
  digitalWrite(cw_r,HIGH);
  digitalWrite(ccw_l,HIGH);
  digitalWrite(lock_r, LOW);
  digitalWrite(lock_l, LOW);
  delayMicroseconds((unsigned long)(intrvl*1000));
  digitalWrite(cw_r,LOW);
  digitalWrite(ccw_l,LOW);
  delayMicroseconds((unsigned long)(intrvl*1000));
}

void Stepping::right_f(float intrvl){//ms
  digitalWrite(cw_r,LOW);
  digitalWrite(ccw_r,LOW);
  digitalWrite(ccw_l,LOW);
  digitalWrite(cw_l,HIGH);
  digitalWrite(lock_r, LOW);
  digitalWrite(lock_l, LOW);
  delayMicroseconds((unsigned long)(intrvl*1000));
  digitalWrite(cw_l,LOW);
  delayMicroseconds((unsigned long)(intrvl*1000));
}

void Stepping::right_b(float intrvl){//ms
  digitalWrite(cw_l,LOW);
  digitalWrite(ccw_r,LOW);
  digitalWrite(ccw_l,LOW);
  digitalWrite(cw_r,HIGH);
  digitalWrite(lock_r, LOW);
  digitalWrite(lock_l, LOW);
  delayMicroseconds((unsigned long)(intrvl*1000));
  digitalWrite(cw_r,LOW);
  delayMicroseconds((unsigned long)(intrvl*1000));
}

void Stepping::left_f(float intrvl){//ms
  digitalWrite(cw_r,LOW);
  digitalWrite(cw_l,LOW);
  digitalWrite(ccw_l,LOW);
  digitalWrite(ccw_r,HIGH);
  digitalWrite(lock_r, LOW);
  digitalWrite(lock_l, LOW);
  delayMicroseconds((unsigned long)(intrvl*1000));
  digitalWrite(ccw_r,LOW);
  delayMicroseconds((unsigned long)(intrvl*1000));
}

void Stepping::left_b(float intrvl){//ms
  digitalWrite(cw_r,LOW);
  digitalWrite(cw_l,LOW);
  digitalWrite(ccw_r,LOW);
  digitalWrite(ccw_l,HIGH);
  digitalWrite(lock_r, LOW);
  digitalWrite(lock_l, LOW);
  delayMicroseconds((unsigned long)(intrvl*1000));
  digitalWrite(ccw_l,LOW);
  delayMicroseconds((unsigned long)(intrvl*1000));
}

void Stepping::delayMilliseconds(float intrvl){//ms
  unsigned long timer = (unsigned long)(intrvl*1000);
  unsigned long previous = micros();
  while(micros() - previous >= timer);
  return;
}

void Stepping::go_forward(float intrvl){
  Stepping::forward(intrvl);
  Stepping::forward(intrvl);
  Stepping::forward(intrvl);
  Stepping::forward(intrvl);
}

void Stepping::go_backward(float intrvl){
  Stepping::backward(intrvl);
  Stepping::backward(intrvl);
  Stepping::backward(intrvl);
  Stepping::backward(intrvl);
}

void Stepping::turn_right(float intrvl, int forward, int right){
  for (int i = 0; i < forward * 4; i++){
    Stepping::forward(intrvl);
  }
  for (int i = 0; i < right * 4; i++){
    Stepping::right_f(intrvl);
  }
}

void Stepping::turn_left(float intrvl, int forward, int left){
  for (int i = 0; i < forward * 4; i++){
    Stepping::forward(intrvl);
  }
  for (int i = 0; i < left * 4; i++){
    Stepping::left_f(intrvl);
  }
}

void Stepping::turn_right(float intrvl){
  Stepping::turn_right(intrvl,1,1);
}

void Stepping::turn_left(float intrvl){
  Stepping::turn_left(intrvl,1,1);
}

void Stepping::right_f_d(float ang_vel, float dist){
  interval = step_degree / ang_vel;
  step_amount = dist * 360 / (2 * 3.14 * tire_radius * step_degree);//n
  for (int i = 0; i < (int)(step_amount/4); i++){
    Stepping::right_f(interval);
  }
}

void Stepping::right_f_t(float ang_vel, float time){
  interval = step_degree / ang_vel;
  step_amount = ang_vel * time * 360 / (2 * 3.14 * tire_radius * step_degree);//n
  for (int i = 0; i < (int)(step_amount/4); i++){
    Stepping::right_f(interval);
  }
}

void Stepping::right_b_d(float ang_vel, float dist){
  interval = step_degree / ang_vel;
  step_amount = dist * 360 / (2 * 3.14 * tire_radius * step_degree);//n
  for (int i = 0; i < (int)(step_amount/4); i++){
    Stepping::right_b(interval);
  }
}

void Stepping::right_b_t(float ang_vel, float time){
  interval = step_degree / ang_vel;
  step_amount = ang_vel * time * 360 / (2 * 3.14 * tire_radius * step_degree);//n
  for (int i = 0; i < (int)(step_amount/4); i++){
    Stepping::right_b(interval);
  }
}

void Stepping::left_f_d(float ang_vel, float dist){
  interval = step_degree / ang_vel;
  step_amount = dist * 360 / (2 * 3.14 * tire_radius * step_degree);//n
  for (int i = 0; i < (int)(step_amount/4); i++){
    Stepping::left_f(interval);
  }
}

void Stepping::left_f_t(float ang_vel, float time){
  interval = step_degree / ang_vel;
  step_amount = ang_vel * time * 360 / (2 * 3.14 * tire_radius * step_degree);//n
  for (int i = 0; i < (int)(step_amount/4); i++){
    Stepping::left_f(interval);
  }
}

void Stepping::left_b_d(float ang_vel, float dist){
  interval = step_degree / ang_vel;
  step_amount = dist * 360 / (2 * 3.14 * tire_radius * step_degree);//n
  for (int i = 0; i < (int)(step_amount/4); i++){
    Stepping::left_b(interval);
  }
}

void Stepping::left_b_t(float ang_vel, float time){
  interval = step_degree / ang_vel;
  step_amount = ang_vel * time * 360 / (2 * 3.14 * tire_radius * step_degree);//n
  for (int i = 0; i < (int)(step_amount/4); i++){
    Stepping::left_b(interval);
  }
}

void Stepping::move_forward_d(float ang_vel, float dist){
  interval = step_degree / ang_vel;
  step_amount = dist * 360 / (2 * 3.14 * tire_radius * step_degree);//n
  for (int i = 0; i < (int)(step_amount/4); i++){
    Stepping::go_forward(interval);
  }
}

void Stepping::move_forward_t(float ang_vel, float time){//deg/ms,ms
  interval = step_degree / ang_vel;//ms
  step_amount = ang_vel * time * 360 / (2 * 3.14 * tire_radius * step_degree);//n
  for (int i = 0; i < (int)(step_amount/4); i++){
    Stepping::go_forward(interval);
  }
}

void Stepping::move_backward_d(float ang_vel, float dist){
  interval = step_degree / ang_vel;
  step_amount = dist * 360 / (2 * 3.14 * tire_radius * step_degree);//n
  for (int i = 0; i < (int)(step_amount/4); i++){
    Stepping::go_backward(interval);
  }
}

void Stepping::move_backward_t(float ang_vel, float time){
  interval = step_degree / ang_vel;
  step_amount = ang_vel * time * 360 / (2 * 3.14 * tire_radius * step_degree);
  for (int i = 0; i < (int)(step_amount/4); i++){
    Stepping::go_backward(interval);
  }
}

void Stepping::move_right_d(float ang_vel_r, float dist_r, float ang_vel_l, float dist_l){
  interval_r = step_degree / ang_vel_r;
  interval_l = step_degree / ang_vel_l;
  step_amount_r = dist_r * 360 / (2 * 3.14 * tire_radius * step_degree);//n
  step_amount_l = dist_l * 360 / (2 * 3.14 * tire_radius * step_degree);//n
  if (step_amount_r >= step_amount_l){
    for (int i = 0; i < (int)((step_amount_r-step_amount_l)/4); i++){
      Stepping::go_forward((interval_r+interval_l)/2);
    }
    for (int i = 0; i < (int)(step_amount_l/8); i++){
      Stepping::turn_right(interval);
    }
  } else if (step_amount_l < step_amount_r){
    for (int i = 0; i < (int)((step_amount_l-step_amount_r)/4); i++){
      Stepping::go_forward(interval);
    }
    for (int i = 0; i < (int)(step_amount_r/8); i++){
      Stepping::turn_right(interval);
    }
  }
}

void Stepping::move_right_t(float ang_vel_r, float time_r, float ang_vel_l, float time_l){
  interval_r = step_degree / ang_vel_r;
  interval_l = step_degree / ang_vel_l;
  step_amount_r = ang_vel_r * time_r * 360 / (2 * 3.14 * tire_radius * step_degree);//n
  step_amount_l = ang_vel_l * time_l * 360 / (2 * 3.14 * tire_radius * step_degree);//n
  if (step_amount_r >= step_amount_l){
    for (int i = 0; i < (int)((step_amount_r-step_amount_l)/4); i++){
      Stepping::go_forward((interval_r+interval_l)/2);
    }
    for (int i = 0; i < (int)(step_amount_l/8); i++){
      Stepping::turn_right(interval);
    }
  } else if (step_amount_l < step_amount_r){
    for (int i = 0; i < (int)((step_amount_l-step_amount_r)/4); i++){
      Stepping::go_forward(interval);
    }
    for (int i = 0; i < (int)(step_amount_r/8); i++){
      Stepping::turn_right(interval);
    }
  }
}

void Stepping::move_left_d(float ang_vel_r, float dist_r, float ang_vel_l, float dist_l){
  interval_r = step_degree / ang_vel_r;
  interval_l = step_degree / ang_vel_l;
  step_amount_r = dist_r * 360 / (2 * 3.14 * tire_radius * step_degree);//n
  step_amount_l = dist_l * 360 / (2 * 3.14 * tire_radius * step_degree);//n
  if (step_amount_r >= step_amount_l){
    for (int i = 0; i < (int)((step_amount_r-step_amount_l)/4); i++){
      Stepping::go_forward((interval_r+interval_l)/2);
    }
    for (int i = 0; i < (int)(step_amount_l/8); i++){
      Stepping::turn_left(interval);
    }
  } else if (step_amount_l < step_amount_r){
    for (int i = 0; i < (int)((step_amount_l-step_amount_r)/4); i++){
      Stepping::go_forward(interval);
    }
    for (int i = 0; i < (int)(step_amount_r/8); i++){
      Stepping::turn_left(interval);
    }
  }
}

void Stepping::move_left_t(float ang_vel_r, float time_r, float ang_vel_l, float time_l){
  interval_r = step_degree / ang_vel_r;
  interval_l = step_degree / ang_vel_l;
  step_amount_r = ang_vel_r * time_r * 360 / (2 * 3.14 * tire_radius * step_degree);//n
  step_amount_l = ang_vel_l * time_l * 360 / (2 * 3.14 * tire_radius * step_degree);//n
  if (step_amount_r >= step_amount_l){
    for (int i = 0; i < (int)((step_amount_r-step_amount_l)/4); i++){
      Stepping::go_forward((interval_r+interval_l)/2);
    }
    for (int i = 0; i < (int)(step_amount_l/8); i++){
      Stepping::turn_left(interval);
    }
  } else if (step_amount_l < step_amount_r){
    for (int i = 0; i < (int)((step_amount_l-step_amount_r)/4); i++){
      Stepping::go_forward(interval);
    }
    for (int i = 0; i < (int)(step_amount_r/8); i++){
      Stepping::turn_left(interval);
    }
  }
}

void Stepping::lock_right(){
  digitalWrite(lock_r, HIGH);
}

void Stepping::unlock_right(){
  digitalWrite(lock_r, LOW);
}

void Stepping::lock_left(){
  digitalWrite(lock_l, HIGH);
}

void Stepping::unlock_left(){
  digitalWrite(lock_l, LOW);
}

void Stepping::lock_both(){
  digitalWrite(lock_r, HIGH);
  digitalWrite(lock_l, HIGH);
}

void Stepping::unlock_both(){
  digitalWrite(lock_r, LOW);
  digitalWrite(lock_l, LOW);
}