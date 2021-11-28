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
  digitalWrite(lock_r, LOW);
  digitalWrite(lock_l, LOW);
  digitalWrite(cw_r,LOW);
  digitalWrite(ccw_l,LOW);
  digitalWrite(ccw_r,HIGH);
  digitalWrite(cw_l,HIGH);
  delayMicroseconds((unsigned long)(intrvl*1000));
  digitalWrite(ccw_r,LOW);
  digitalWrite(cw_l,LOW);
  delayMicroseconds((unsigned long)(intrvl*1000));
  digitalWrite(ccw_r,HIGH);
  digitalWrite(cw_l,HIGH);
  delayMicroseconds((unsigned long)(intrvl*1000));
  digitalWrite(ccw_r,LOW);
  digitalWrite(cw_l,LOW);
  delayMicroseconds((unsigned long)(intrvl*1000));
  digitalWrite(ccw_r,HIGH);
  digitalWrite(cw_l,HIGH);
  delayMicroseconds((unsigned long)(intrvl*1000));
  digitalWrite(ccw_r,LOW);
  digitalWrite(cw_l,LOW);
  delayMicroseconds((unsigned long)(intrvl*1000));
  digitalWrite(ccw_r,HIGH);
  digitalWrite(cw_l,HIGH);
  delayMicroseconds((unsigned long)(intrvl*1000));
  digitalWrite(ccw_r,LOW);
  digitalWrite(cw_l,LOW);
  delayMicroseconds((unsigned long)(intrvl*1000));
}

void Stepping::backward(float intrvl){//ms
  digitalWrite(lock_r, LOW);
  digitalWrite(lock_l, LOW);
  digitalWrite(ccw_r,LOW);
  digitalWrite(cw_l,LOW);
  digitalWrite(cw_r,HIGH);
  digitalWrite(ccw_l,HIGH);
  delayMicroseconds((unsigned long)(intrvl*1000));
  digitalWrite(cw_r,LOW);
  digitalWrite(ccw_l,LOW);
  delayMicroseconds((unsigned long)(intrvl*1000));
  digitalWrite(cw_r,HIGH);
  digitalWrite(ccw_l,HIGH);
  delayMicroseconds((unsigned long)(intrvl*1000));
  digitalWrite(cw_r,LOW);
  digitalWrite(ccw_l,LOW);
  delayMicroseconds((unsigned long)(intrvl*1000));
  digitalWrite(cw_r,HIGH);
  digitalWrite(ccw_l,HIGH);
  delayMicroseconds((unsigned long)(intrvl*1000));
  digitalWrite(cw_r,LOW);
  digitalWrite(ccw_l,LOW);
  delayMicroseconds((unsigned long)(intrvl*1000));
  digitalWrite(cw_r,HIGH);
  digitalWrite(ccw_l,HIGH);
  delayMicroseconds((unsigned long)(intrvl*1000));
  digitalWrite(cw_r,LOW);
  digitalWrite(ccw_l,LOW);
  delayMicroseconds((unsigned long)(intrvl*1000));
}

void Stepping::right_f(float intrvl){//ms
  digitalWrite(lock_r, LOW);
  digitalWrite(lock_l, LOW);
  digitalWrite(cw_r,LOW);
  digitalWrite(ccw_r,LOW);
  digitalWrite(ccw_l,LOW);
  digitalWrite(cw_l,HIGH);
  delayMicroseconds((unsigned long)(intrvl*1000));
  digitalWrite(cw_l,LOW);
  delayMicroseconds((unsigned long)(intrvl*1000));
  digitalWrite(cw_l,HIGH);
  delayMicroseconds((unsigned long)(intrvl*1000));
  digitalWrite(cw_l,LOW);
  delayMicroseconds((unsigned long)(intrvl*1000));
  digitalWrite(cw_l,HIGH);
  delayMicroseconds((unsigned long)(intrvl*1000));
  digitalWrite(cw_l,LOW);
  delayMicroseconds((unsigned long)(intrvl*1000));
  digitalWrite(cw_l,HIGH);
  delayMicroseconds((unsigned long)(intrvl*1000));
  digitalWrite(cw_l,LOW);
  delayMicroseconds((unsigned long)(intrvl*1000));
}

void Stepping::right_b(float intrvl){//ms
  digitalWrite(lock_r, LOW);
  digitalWrite(lock_l, LOW);
  digitalWrite(cw_l,LOW);
  digitalWrite(ccw_r,LOW);
  digitalWrite(ccw_l,LOW);
  digitalWrite(cw_r,HIGH);
  delayMicroseconds((unsigned long)(intrvl*1000));
  digitalWrite(cw_r,LOW);
  delayMicroseconds((unsigned long)(intrvl*1000));
  digitalWrite(cw_r,HIGH);
  delayMicroseconds((unsigned long)(intrvl*1000));
  digitalWrite(cw_r,LOW);
  delayMicroseconds((unsigned long)(intrvl*1000));
  digitalWrite(cw_r,HIGH);
  delayMicroseconds((unsigned long)(intrvl*1000));
  digitalWrite(cw_r,LOW);
  delayMicroseconds((unsigned long)(intrvl*1000));
  digitalWrite(cw_r,HIGH);
  delayMicroseconds((unsigned long)(intrvl*1000));
  digitalWrite(cw_r,LOW);
  delayMicroseconds((unsigned long)(intrvl*1000));
}

void Stepping::left_f(float intrvl){//ms
  digitalWrite(lock_r, LOW);
  digitalWrite(lock_l, LOW);
  digitalWrite(cw_r,LOW);
  digitalWrite(cw_l,LOW);
  digitalWrite(ccw_l,LOW);
  digitalWrite(ccw_r,HIGH);
  delayMicroseconds((unsigned long)(intrvl*1000));
  digitalWrite(ccw_r,LOW);
  delayMicroseconds((unsigned long)(intrvl*1000));
  digitalWrite(ccw_r,HIGH);
  delayMicroseconds((unsigned long)(intrvl*1000));
  digitalWrite(ccw_r,LOW);
  delayMicroseconds((unsigned long)(intrvl*1000));
  digitalWrite(ccw_r,HIGH);
  delayMicroseconds((unsigned long)(intrvl*1000));
  digitalWrite(ccw_r,LOW);
  delayMicroseconds((unsigned long)(intrvl*1000));
  digitalWrite(ccw_r,HIGH);
  delayMicroseconds((unsigned long)(intrvl*1000));
  digitalWrite(ccw_r,LOW);
  delayMicroseconds((unsigned long)(intrvl*1000));
}

void Stepping::left_b(float intrvl){//ms
  digitalWrite(lock_r, LOW);
  digitalWrite(lock_l, LOW);
  digitalWrite(cw_r,LOW);
  digitalWrite(cw_l,LOW);
  digitalWrite(ccw_r,LOW);
  digitalWrite(ccw_l,HIGH);
  delayMicroseconds((unsigned long)(intrvl*1000));
  digitalWrite(ccw_l,LOW);
  delayMicroseconds((unsigned long)(intrvl*1000));
  digitalWrite(ccw_l,HIGH);
  delayMicroseconds((unsigned long)(intrvl*1000));
  digitalWrite(ccw_l,LOW);
  delayMicroseconds((unsigned long)(intrvl*1000));
  digitalWrite(ccw_l,HIGH);
  delayMicroseconds((unsigned long)(intrvl*1000));
  digitalWrite(ccw_l,LOW);
  delayMicroseconds((unsigned long)(intrvl*1000));
  digitalWrite(ccw_l,HIGH);
  delayMicroseconds((unsigned long)(intrvl*1000));
  digitalWrite(ccw_l,LOW);
  delayMicroseconds((unsigned long)(intrvl*1000));
}

void Stepping::delayMilliseconds(float intrvl){//ms
  unsigned long previous = micros();
  unsigned long timer = (unsigned long)(intrvl*1000);
  while(micros() - previous <= timer);
  return;
}

// void Stepping::forward(float intrvl){
//   Stepping::forward(intrvl);
//   Stepping::forward(intrvl);
//   Stepping::forward(intrvl);
//   Stepping::forward(intrvl);
// }

// void Stepping::backward(float intrvl){
//   Stepping::backward(intrvl);
//   Stepping::backward(intrvl);
//   Stepping::backward(intrvl);
//   Stepping::backward(intrvl);
// }

void Stepping::turn_right(float intrvl, int forward, int right){
  for (int i = 0; i < forward; i++){
    Stepping::forward(intrvl);
  }
  for (int i = 0; i < right; i++){
    Stepping::right_f(intrvl);
  }
}

void Stepping::turn_left(float intrvl, int forward, int left){
  for (int i = 0; i < forward; i++){
    Stepping::forward(intrvl);
  }
  for (int i = 0; i < left; i++){
    Stepping::left_f(intrvl);
  }
}

void Stepping::turn_right(float intrvl){
  Stepping::turn_right(intrvl,1,1);
}

void Stepping::turn_left(float intrvl){
  Stepping::turn_left(intrvl,1,1);
}

void Stepping::right_f(float ang_vel, float angle){
  angular_vel = (ang_vel>=12)?12:ang_vel;
  interval = step_degree / (angular_vel * 2);
  step_amount = angle / step_degree;//n
  for (int i = 0; i < (int)(step_amount/4); i++){
    Stepping::right_f(interval);
  }
}

void Stepping::right_f_t(float ang_vel, float time){
  angular_vel = (ang_vel>=12)?12:ang_vel;
  interval = step_degree / (angular_vel * 2);
  step_amount = angular_vel * time / step_degree;//n
  for (int i = 0; i < (int)(step_amount/4); i++){
    Stepping::right_f(interval);
  }
}

void Stepping::right_b(float ang_vel, float angle){
  angular_vel = (ang_vel>=12)?12:ang_vel;
  interval = step_degree / (angular_vel * 2);
  step_amount = angle / step_degree;//n
  for (int i = 0; i < (int)(step_amount/4); i++){
    Stepping::right_b(interval);
  }
}

void Stepping::right_b_t(float ang_vel, float time){
  angular_vel = (ang_vel>=12)?12:ang_vel;
  interval = step_degree / (angular_vel * 2);
  step_amount = angular_vel * time / step_degree;//n
  for (int i = 0; i < (int)(step_amount/4); i++){
    Stepping::right_b(interval);
  }
}

void Stepping::left_f(float ang_vel, float angle){
  angular_vel = (ang_vel>=12)?12:ang_vel;
  interval = step_degree / (angular_vel * 2);
  step_amount = angle / step_degree;//n
  for (int i = 0; i < (int)(step_amount/4); i++){
    Stepping::left_f(interval);
  }
}

void Stepping::left_f_t(float ang_vel, float time){
  angular_vel = (ang_vel>=12)?12:ang_vel;
  interval = step_degree / (angular_vel * 2);
  step_amount = angular_vel * time / step_degree;//n
  for (int i = 0; i < (int)(step_amount/4); i++){
    Stepping::left_f(interval);
  }
}

void Stepping::left_b(float ang_vel, float angle){
  angular_vel = (ang_vel>=12)?12:ang_vel;
  interval = step_degree / (angular_vel * 2);
  step_amount = angle / step_degree;//n
  for (int i = 0; i < (int)(step_amount/4); i++){
    Stepping::left_b(interval);
  }
}

void Stepping::left_b_t(float ang_vel, float time){
  angular_vel = (ang_vel>=12)?12:ang_vel;
  interval = step_degree / (angular_vel * 2);
  step_amount = angular_vel * time / step_degree;//n
  for (int i = 0; i < (int)(step_amount/4); i++){
    Stepping::left_b(interval);
  }
}

void Stepping::move_forward(float ang_vel, float angle){//deg/ms,deg
  angular_vel = (ang_vel>=12)?12:ang_vel;
  interval = step_degree / (angular_vel * 2);
  step_amount = angle / step_degree;//n
  for (int i = 0; i < (int)(step_amount/4); i++){
    Stepping::forward(interval);
  }
}

void Stepping::move_forward_t(float ang_vel, float time){//deg/ms,ms
  angular_vel = (ang_vel>=12)?12:ang_vel;
  interval = step_degree / (angular_vel * 2);//ms
  step_amount = angular_vel * time / step_degree;//n
  for (int i = 0; i < (int)(step_amount/4); i++){
    Stepping::forward(interval);
  }
}

void Stepping::move_backward(float ang_vel, float angle){
  angular_vel = (ang_vel>=12)?12:ang_vel;
  interval = step_degree / (angular_vel * 2);
  step_amount = angle / step_degree;//n
  for (int i = 0; i < (int)(step_amount/4); i++){
    Stepping::backward(interval);
  }
}

void Stepping::move_backward_t(float ang_vel, float time){
  angular_vel = (ang_vel>=12)?12:ang_vel;
  interval = step_degree / (angular_vel * 2);
  step_amount = angular_vel * time / step_degree;
  for (int i = 0; i < (int)(step_amount/4); i++){
    Stepping::backward(interval);
  }
}

void Stepping::move_right(float ang_vel_r, float angle_r, float ang_vel_l, float angle_l){
  angular_vel_r = (ang_vel_r>=12)?12:ang_vel_r;
  angular_vel_l = (ang_vel_l>=12)?12:ang_vel_l;
  interval_r = step_degree / (angular_vel_r * 2);
  interval_l = step_degree / (angular_vel_l * 2);
  step_amount_r = angle_r / step_degree;//n
  step_amount_l = angle_l / step_degree;//n
  if (step_amount_r >= step_amount_l){
    for (int i = 0; i < (int)((step_amount_r-step_amount_l)/4); i++){
      Stepping::forward((interval_r+interval_l)/2);
    }
    for (int i = 0; i < (int)(step_amount_l/8); i++){
      Stepping::turn_right(interval);
    }
  } else if (step_amount_l < step_amount_r){
    for (int i = 0; i < (int)((step_amount_l-step_amount_r)/4); i++){
      Stepping::forward(interval);
    }
    for (int i = 0; i < (int)(step_amount_r/8); i++){
      Stepping::turn_right(interval);
    }
  }
}

void Stepping::move_right_t(float ang_vel_r, float time_r, float ang_vel_l, float time_l){
  angular_vel_r = (ang_vel_r>=12)?12:ang_vel_r;
  angular_vel_l = (ang_vel_l>=12)?12:ang_vel_l;
  interval_r = step_degree / (angular_vel_r * 2);
  interval_l = step_degree / (angular_vel_l * 2);
  step_amount_r = angular_vel_r * time_r / step_degree;//n
  step_amount_l = angular_vel_l * time_l / step_degree;//n
  if (step_amount_r >= step_amount_l){
    for (int i = 0; i < (int)((step_amount_r-step_amount_l)/4); i++){
      Stepping::forward((interval_r+interval_l)/2);
    }
    for (int i = 0; i < (int)(step_amount_l/8); i++){
      Stepping::turn_right(interval);
    }
  } else if (step_amount_l < step_amount_r){
    for (int i = 0; i < (int)((step_amount_l-step_amount_r)/4); i++){
      Stepping::forward(interval);
    }
    for (int i = 0; i < (int)(step_amount_r/8); i++){
      Stepping::turn_right(interval);
    }
  }
}

void Stepping::move_left(float ang_vel_r, float angle_r, float ang_vel_l, float angle_l){
  angular_vel_r = (ang_vel_r>=12)?12:ang_vel_r;
  angular_vel_l = (ang_vel_l>=12)?12:ang_vel_l;
  interval_r = step_degree / (angular_vel_r * 2);
  interval_l = step_degree / (angular_vel_l * 2);
  step_amount_r = angle_r / step_degree;//n
  step_amount_l = angle_l / step_degree;//n
  if (step_amount_r >= step_amount_l){
    for (int i = 0; i < (int)((step_amount_r-step_amount_l)/4); i++){
      Stepping::forward((interval_r+interval_l)/2);
    }
    for (int i = 0; i < (int)(step_amount_l/8); i++){
      Stepping::turn_left(interval);
    }
  } else if (step_amount_l < step_amount_r){
    for (int i = 0; i < (int)((step_amount_l-step_amount_r)/4); i++){
      Stepping::forward(interval);
    }
    for (int i = 0; i < (int)(step_amount_r/8); i++){
      Stepping::turn_left(interval);
    }
  }
}

void Stepping::move_left_t(float ang_vel_r, float time_r, float ang_vel_l, float time_l){
  angular_vel_r = (ang_vel_r>=12)?12:ang_vel_r;
  angular_vel_l = (ang_vel_l>=12)?12:ang_vel_l;
  interval_r = step_degree / (angular_vel_r * 2);
  interval_l = step_degree / (angular_vel_l * 2);
  step_amount_r = angular_vel_r * time_r / step_degree;//n
  step_amount_l = angular_vel_l * time_l / step_degree;//n
  if (step_amount_r >= step_amount_l){
    for (int i = 0; i < (int)((step_amount_r-step_amount_l)/4); i++){
      Stepping::forward((interval_r+interval_l)/2);
    }
    for (int i = 0; i < (int)(step_amount_l/8); i++){
      Stepping::turn_left(interval);
    }
  } else if (step_amount_l < step_amount_r){
    for (int i = 0; i < (int)((step_amount_l-step_amount_r)/4); i++){
      Stepping::forward(interval);
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

void Stepping::rotate(int rot_dir_r, int rot_dir_l, float intrvl){
  if (rot_dir_r == 2 & rot_dir_l == 1){
    forward(intrvl);
  } else if (rot_dir_r == 1 & rot_dir_l == 2){
    backward(intrvl);
  } else if (rot_dir_r == 2 & rot_dir_l == 0){
    right_f(intrvl);
  } else if (rot_dir_r == 1 & rot_dir_l == 0){
    right_b(intrvl);
  } else if (rot_dir_r == 0 & rot_dir_l == 1){
    left_f(intrvl);
  } else if (rot_dir_r == 0 & rot_dir_l == 2){
    left_b(intrvl);
  } else {
    delay(intrvl);
  }
}

void Stepping::rot_independent(int rot_dir_r, float ang_vel_r, float angle_r, int rot_dir_l, float ang_vel_l, float angle_l){
  angular_vel_r = (ang_vel_r>=12)?12:ang_vel_r;
  angular_vel_l = (ang_vel_l>=12)?12:ang_vel_l;
  step_amount_r = angle_r;
  step_amount_l = angle_l;
  step_count_r = 0;
  step_count_l = 0;
  interval_r = step_degree / (angular_vel_r * 2);
  interval_l = step_degree / (angular_vel_l * 2);
  previous_r = micros();
  previous_l = micros();
  enable = true;
  while (step_count_r < step_amount_r | step_count_l < step_amount_l) {
    if (micros()-previous_r >= interval_r){
      rotate(rot_dir_r, rot_dir_l, interval_r);
      step_count_r++;
      previous_r = micros();
    } else if (micros()-previous_l >= interval_l){
      rotate(rot_dir_r, rot_dir_l, interval_l);
      step_count_l++;
      previous_l = micros();
    }
  }
  if (step_count_r <= step_amount_r) {
    while (step_count_l < step_amount_l){
      if (micros()-previous_l >= interval_l){
        rotate(rot_dir_r, rot_dir_l, interval_l);
        step_count_l++;
        previous_l = micros();
      }
    }
  } else if (step_count_l <= step_amount_l) {
    while (step_count_r < step_amount_r){
      if (micros()-previous_r >= interval_r){
        rotate(rot_dir_r, rot_dir_l, interval_r);
        step_count_r++;
        previous_r = micros();
      }
    }
  }
}



