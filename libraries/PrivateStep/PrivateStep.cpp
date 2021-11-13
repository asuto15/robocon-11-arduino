#include "PrivateStep.h"
#include "TimedAction.h"

int PrivateStep::forward1=0;
int PrivateStep::forward2=0;
int PrivateStep::back1=0;
int PrivateStep::back2=0;
int PrivateStep::lock1=0;
int PrivateStep::lock2=0;
int PrivateStep::change_step1=0;
int PrivateStep::change_step2=0;
float PrivateStep::speed1=0.0;
float PrivateStep::speed2=0.0;
float PrivateStep::time1=0.0;
float PrivateStep::time2=0.0;
float PrivateStep::degree1=0.0;
float PrivateStep::degree2=0.0;
float PrivateStep::basestep=0.0;
int PrivateStep::step_amount1=0;
int PrivateStep::step_amount2=0;
float PrivateStep::frequency1=0.0;
float PrivateStep::frequency2=0.0;
float PrivateStep::sum_width1=0.0;
float PrivateStep::sum_width2=0.0;
float PrivateStep::pulse_width1=0.0;
float PrivateStep::pulse_width2=0.0;
unsigned long PrivateStep::current_time1=0;
unsigned long PrivateStep::current_time2=0;
int PrivateStep::step_count1=0;
int PrivateStep::step_count2=0;
bool PrivateStep::enable1=true;
bool PrivateStep::enable2=true;
int PrivateStep::pulse_pin=0;
int PrivateStep::rot_pin=0;
int PrivateStep::lock_pin=0;
int PrivateStep::unlock_pin=0;
int PrivateStep::rot_dir1=0;
int PrivateStep::rot_dir2=0;
int PrivateStep::signaltype=0;
int PrivateStep::rot_dir=0;

PrivateStep::PrivateStep(){
}

void PrivateStep::setup(int cw1, int ccw1, int awo1, int cs1) {
  PrivateStep::forward1 = cw1;
  PrivateStep::back1 = ccw1;
  PrivateStep::lock1 = awo1;
  PrivateStep::change_step1 = cs1;
  PrivateStep::speed1 = 0;
  PrivateStep::time1 = 0;
  PrivateStep::degree1 = 0;
  PrivateStep::forward2 = -1;
  PrivateStep::back2 = -1;
  PrivateStep::basestep = 1.8;
}

void PrivateStep::setup(int cw1, int ccw1, int awo1) {
  PrivateStep::forward1 = cw1;
  PrivateStep::back1 = ccw1;
  PrivateStep::lock1 = awo1;
  PrivateStep::change_step1 = 0;
  PrivateStep::speed1 = 0;
  PrivateStep::time1 = 0;
  PrivateStep::degree1 = 0;
  PrivateStep::forward2 = -1;
  PrivateStep::back2 = -1;
  PrivateStep::basestep = 1.8;
}

void PrivateStep::setup(int cw1, int ccw1, int awo1, int cs1, int cw2, int ccw2, int awo2, int cs2) {
  PrivateStep::forward1 = cw1;
  PrivateStep::back1 = ccw1;
  PrivateStep::lock1 = awo1;
  PrivateStep::change_step1 = cs1;
  PrivateStep::speed1 = 0;
  PrivateStep::time1 = 0;
  PrivateStep::degree1 = 0;
  PrivateStep::forward2 = cw2;
  PrivateStep::back2 = ccw2;
  PrivateStep::lock2 = awo2;
  PrivateStep::change_step2 = cs2;
  PrivateStep::speed2 = 0;
  PrivateStep::time2 = 0;
  PrivateStep::degree2 = 0;
  PrivateStep::basestep = 1.8;
}

void PrivateStep::setup(int cw1, int ccw1, int awo1, int cw2, int ccw2, int awo2) {
  PrivateStep::forward1 = cw1;
  PrivateStep::back1 = ccw1;
  PrivateStep::lock1 = awo1;
  PrivateStep::change_step1 = 0;
  PrivateStep::speed1 = 0;
  PrivateStep::time1 = 0;
  PrivateStep::degree1 = 0;
  PrivateStep::forward2 = cw2;
  PrivateStep::back2 = ccw2;
  PrivateStep::lock2 = awo2;
  PrivateStep::change_step2 = 0;
  PrivateStep::speed2 = 0;
  PrivateStep::time2 = 0;
  PrivateStep::degree2 = 0;
  PrivateStep::basestep = 1.8;
}

void PrivateStep::setTime(float spd1, float tim1) {
  PrivateStep::speed1 = spd1;
  PrivateStep::time1 = tim1;
  PrivateStep::degree1 = spd1 * tim1;
  PrivateStep::speed2 = 0;
  PrivateStep::time2 = 0;
  PrivateStep::degree2 = 0;
}

void PrivateStep::setTimes(float spd1, float tim1, float spd2, float tim2) {
  PrivateStep::speed1 = spd1;
  PrivateStep::time1 = tim1;
  PrivateStep::degree1 = spd1 * tim1;
  PrivateStep::speed2 = spd2;
  PrivateStep::time2 = tim2;
  PrivateStep::degree2 = spd2 * tim2;
}

void PrivateStep::setDegree(float spd1, float dgr1) {
  PrivateStep::speed1 = spd1;
  PrivateStep::time1 = dgr1 / spd1;
  PrivateStep::degree1 = dgr1;
  PrivateStep::speed2 = 0;
  PrivateStep::time2 = 0;
  PrivateStep::degree2 = 0;
}

void PrivateStep::setDegrees(float spd1, float dgr1, float spd2, float dgr2) {
  PrivateStep::speed1 = spd1;
  PrivateStep::time1 = dgr1 / spd1;
  PrivateStep::degree1 = dgr1;
  PrivateStep::speed2 = spd2;
  PrivateStep::time2 = dgr2 / spd2;
  PrivateStep::degree2 = dgr2;
}

void PrivateStep::pulse(int pin, int stp_amnt, float pls_wdth){
  PrivateStep::pulse_pin = pin;
  PrivateStep::step_amount1 = stp_amnt;
  PrivateStep::pulse_width1 = pls_wdth;
  PrivateStep::step_count1 = 0;
  while (PrivateStep::step_count1 <= PrivateStep::step_amount1) {
    PrivateStep::current_time1 = micros();
    digitalWrite(PrivateStep::pulse_pin,HIGH);
    while (micros() - PrivateStep::current_time1 <= PrivateStep::pulse_width1);
    digitalWrite(PrivateStep::pulse_pin,LOW);
    while (micros() - PrivateStep::current_time1 <= PrivateStep::sum_width1);
    PrivateStep::step_count1 += 1;
  }
}

void PrivateStep::cw1(){
  PrivateStep::current_time1 = micros();
  digitalWrite(PrivateStep::forward1,HIGH);
  while (micros() - PrivateStep::current_time1 <= PrivateStep::pulse_width1);
  digitalWrite(PrivateStep::forward1,LOW);
  while (micros() - PrivateStep::current_time1 <= PrivateStep::sum_width1);
}

void PrivateStep::ccw1(){
  PrivateStep::current_time1 = micros();
  digitalWrite(PrivateStep::back1,HIGH);
  while (micros() - PrivateStep::current_time1 <= PrivateStep::pulse_width1);
  digitalWrite(PrivateStep::back1,LOW);
  while (micros() - PrivateStep::current_time1 <= PrivateStep::sum_width1);
}

void PrivateStep::cw2(){
  PrivateStep::current_time1 = micros();
  digitalWrite(PrivateStep::forward2,HIGH);
  while (micros() - PrivateStep::current_time1 <= PrivateStep::pulse_width2);
  digitalWrite(PrivateStep::forward2,LOW);
  while (micros() - PrivateStep::current_time1 <= PrivateStep::sum_width2);
}

void PrivateStep::ccw2(){
  PrivateStep::current_time1 = micros();
  digitalWrite(PrivateStep::back2,HIGH);
  while (micros() - PrivateStep::current_time1 <= PrivateStep::pulse_width2);
  digitalWrite(PrivateStep::back2,LOW);
  while (micros() - PrivateStep::current_time1 <= PrivateStep::sum_width2);
}

void PrivateStep::work(int sgnltyp, int dir){
  PrivateStep::signaltype = sgnltyp;
  PrivateStep::rot_dir = dir;
  if (PrivateStep::signaltype == 10 && PrivateStep::rot_dir == 0) {
    rot(PrivateStep::forward1);
  } else if (PrivateStep::signaltype == 10 && PrivateStep::rot_dir == 1) {
    rot(PrivateStep::back1);
  } else if (PrivateStep::signaltype == 10 && PrivateStep::rot_dir == 2) {
    lock(PrivateStep::lock1);
  } else if (PrivateStep::signaltype == 20 && PrivateStep::rot_dir == 0) {
    rot(PrivateStep::forward2);
  } else if (PrivateStep::signaltype == 20 && PrivateStep::rot_dir == 1) {
    rot(PrivateStep::back2);
  } else if (PrivateStep::signaltype == 20 && PrivateStep::rot_dir == 2) {
    lock(PrivateStep::lock2);
  } else if (PrivateStep::signaltype == 30 && PrivateStep::rot_dir == 3) {
    rot_both(3);
  } else if (PrivateStep::signaltype == 30 && PrivateStep::rot_dir == 4) {
    rot_both(4);
  } else if (PrivateStep::signaltype == 30 && PrivateStep::rot_dir == 5) {
    rot_both(5);
  } else if (PrivateStep::signaltype == 30 && PrivateStep::rot_dir == 6) {
    rot_both(6);
  } else if (PrivateStep::signaltype == 30 && PrivateStep::rot_dir == 2) {
    lock(PrivateStep::lock1);
    lock(PrivateStep::lock2);
  } 
}

void PrivateStep::work_p(int sgnltyp, int dir){
  PrivateStep::signaltype = sgnltyp;
  PrivateStep::rot_dir = dir;
  if (PrivateStep::signaltype == 10 && PrivateStep::rot_dir == 0) {
    rot_p1(PrivateStep::forward1);
  } else if (PrivateStep::signaltype == 10 && PrivateStep::rot_dir == 1) {
    rot_p2(PrivateStep::back1);
  } else if (PrivateStep::signaltype == 10 && PrivateStep::rot_dir == 2) {
    lock(PrivateStep::lock1);
  } else if (PrivateStep::signaltype == 20 && PrivateStep::rot_dir == 0) {
    rot_p1(PrivateStep::forward2);
  } else if (PrivateStep::signaltype == 20 && PrivateStep::rot_dir == 1) {
    rot_p2(PrivateStep::back2);
  } else if (PrivateStep::signaltype == 20 && PrivateStep::rot_dir == 2) {
    lock(PrivateStep::lock2);
  } else if (PrivateStep::signaltype == 30 && PrivateStep::rot_dir == 3) {
    rot_p_both(3);
  } else if (PrivateStep::signaltype == 30 && PrivateStep::rot_dir == 4) {
    rot_p_both(4);
  } else if (PrivateStep::signaltype == 30 && PrivateStep::rot_dir == 5) {
    rot_p_both(5);
  } else if (PrivateStep::signaltype == 30 && PrivateStep::rot_dir == 6) {
    rot_p_both(6);
  } else if (PrivateStep::signaltype == 30 && PrivateStep::rot_dir == 2) {
    lock(PrivateStep::lock1);
    lock(PrivateStep::lock2);
  } 
}

void PrivateStep::rot(int pin){
  PrivateStep::rot_pin = pin;
  if (PrivateStep::rot_pin == PrivateStep::forward1 || PrivateStep::rot_pin == PrivateStep::back1) {
    PrivateStep::step_amount1 = PrivateStep::degree1 / PrivateStep::basestep;
    PrivateStep::frequency1 = PrivateStep::speed1 / PrivateStep::basestep;
    PrivateStep::sum_width1 = 1 / PrivateStep::frequency1;
    PrivateStep::pulse_width1 = PrivateStep::sum_width1 / 2;
    PrivateStep::step_count1 = 0;
    digitalWrite(PrivateStep::lock1, LOW);
    while (PrivateStep::step_count1 <= PrivateStep::step_amount1) {
      pulse(PrivateStep::rot_pin, PrivateStep::step_amount1, PrivateStep::pulse_width1);
      PrivateStep::step_count1 += 1;
    }
  } else {
    PrivateStep::step_amount2 = PrivateStep::degree2 / PrivateStep::basestep;
    PrivateStep::frequency2 = PrivateStep::speed2 / PrivateStep::basestep;
    PrivateStep::sum_width2 = 1 / PrivateStep::frequency2;
    PrivateStep::pulse_width2 = PrivateStep::sum_width2 / 2;
    PrivateStep::step_count2 = 0;
    digitalWrite(PrivateStep::lock2, LOW);
    while (PrivateStep::step_count2 <= PrivateStep::step_amount1) {
      pulse(PrivateStep::rot_pin, PrivateStep::step_amount2, PrivateStep::pulse_width2);
      PrivateStep::step_count2 += 1;
    }
  }
}

void PrivateStep::lock(int pin){
  PrivateStep::lock_pin = pin;
  if (PrivateStep::lock_pin == PrivateStep::lock1) {
    digitalWrite(PrivateStep::lock1, HIGH);
  } else if (PrivateStep::lock_pin == PrivateStep::lock2) {
    digitalWrite(PrivateStep::lock2, HIGH);
  }
}

void PrivateStep::do_lock1(){
  digitalWrite(PrivateStep::lock1, HIGH);
}

void PrivateStep::do_lock2(){
  digitalWrite(PrivateStep::lock2, HIGH);
}

void PrivateStep::unlock(int pin){
  PrivateStep::unlock_pin = pin;
  if (PrivateStep::unlock_pin == PrivateStep::lock1) {
    digitalWrite(PrivateStep::lock1, LOW);
  } else if (PrivateStep::unlock_pin == PrivateStep::lock2) {
    digitalWrite(PrivateStep::lock2, LOW);
  }
}

void PrivateStep::rot_both(int num){
  PrivateStep::step_amount1 = PrivateStep::degree1 / PrivateStep::basestep;
  PrivateStep::frequency1 = PrivateStep::speed1 / PrivateStep::basestep;
  PrivateStep::sum_width1 = 1 / PrivateStep::frequency1;
  PrivateStep::pulse_width1 = PrivateStep::sum_width1 / 2;
  PrivateStep::step_count1 = 0;
  PrivateStep::step_amount2 = PrivateStep::degree2 / PrivateStep::basestep;
  PrivateStep::frequency2 = PrivateStep::speed2 / PrivateStep::basestep;
  PrivateStep::sum_width2 = 1 / PrivateStep::frequency2;
  PrivateStep::pulse_width2 = PrivateStep::sum_width2 / 2;
  PrivateStep::step_count2 = 0;
  digitalWrite(PrivateStep::lock1, LOW);
  digitalWrite(PrivateStep::lock2, LOW);
  if (num == 3) {
    TimedAction pulse1 = TimedAction(PrivateStep::sum_width1,PrivateStep::cw1);
    TimedAction pulse2 = TimedAction(PrivateStep::sum_width2,PrivateStep::ccw2);
    while (PrivateStep::step_count1 <= PrivateStep::step_amount1 || PrivateStep::step_count2 <= PrivateStep::step_amount2) {
      pulse1.check();
      pulse2.check();
    }
    if (PrivateStep::step_count1 <= PrivateStep::step_amount1) {
      while (PrivateStep::step_count2 <= PrivateStep::step_amount2) {
        pulse2.check();
        PrivateStep::step_count2 += 1;
      }
    } else {
      while (PrivateStep::step_count1 <= PrivateStep::step_amount1) {
        pulse1.check();
        PrivateStep::step_count1 += 1;
      }
    }
  } else if (num == 4) {
    TimedAction pulse1 = TimedAction(PrivateStep::sum_width1,PrivateStep::ccw1);
    TimedAction pulse2 = TimedAction(PrivateStep::sum_width2,PrivateStep::cw2);
    while (PrivateStep::step_count1 <= PrivateStep::step_amount1 || PrivateStep::step_count2 <= PrivateStep::step_amount2) {
      pulse1.check();
      pulse2.check();
    }
    if (PrivateStep::step_count1 <= PrivateStep::step_amount1) {
      while (PrivateStep::step_count2 <= PrivateStep::step_amount2) {
        pulse2.check();
        PrivateStep::step_count2 += 1;
      }
    } else {
      while (PrivateStep::step_count1 <= PrivateStep::step_amount1) {
        pulse1.check();
        PrivateStep::step_count1 += 1;
      }
    }
  } else if (num == 5) {
    TimedAction pulse1 = TimedAction(PrivateStep::sum_width1,PrivateStep::ccw1);
    TimedAction pulse2 = TimedAction(PrivateStep::sum_width2,PrivateStep::ccw2);
    while (PrivateStep::step_count1 <= PrivateStep::step_amount1 || PrivateStep::step_count2 <= PrivateStep::step_amount2) {
      pulse1.check();
      pulse2.check();
    }
    if (PrivateStep::step_count1 <= PrivateStep::step_amount1) {
      while (PrivateStep::step_count2 <= PrivateStep::step_amount2) {
        pulse2.check();
        PrivateStep::step_count2 += 1;
      }
    } else {
      while (PrivateStep::step_count1 <= PrivateStep::step_amount1) {
        pulse1.check();
        PrivateStep::step_count1 += 1;
      }
    }
  } else if (num == 6) {
    TimedAction pulse1 = TimedAction(PrivateStep::sum_width1,PrivateStep::cw1);
    TimedAction pulse2 = TimedAction(PrivateStep::sum_width2,PrivateStep::cw2);
    while (PrivateStep::step_count1 <= PrivateStep::step_amount1 || PrivateStep::step_count2 <= PrivateStep::step_amount2) {
      pulse1.check();
      pulse2.check();
    }
    if (PrivateStep::step_count1 <= PrivateStep::step_amount1) {
      while (PrivateStep::step_count2 <= PrivateStep::step_amount2) {
        pulse2.check();
        PrivateStep::step_count2 += 1;
      }
    } else {
      while (PrivateStep::step_count1 <= PrivateStep::step_amount1) {
        pulse1.check();
        PrivateStep::step_count1 += 1;
      }
    }
  } else {
    TimedAction pulse1 = TimedAction(PrivateStep::sum_width1,PrivateStep::do_lock1);
    TimedAction pulse2 = TimedAction(PrivateStep::sum_width2,PrivateStep::do_lock2);
    while (PrivateStep::step_count1 <= PrivateStep::step_amount1 || PrivateStep::step_count2 <= PrivateStep::step_amount2) {
      pulse1.check();
      pulse2.check();
    }
    if (PrivateStep::step_count1 <= PrivateStep::step_amount1) {
      while (PrivateStep::step_count2 <= PrivateStep::step_amount2) {
        pulse2.check();
        PrivateStep::step_count2 += 1;
      }
    } else {
      while (PrivateStep::step_count1 <= PrivateStep::step_amount1) {
        pulse1.check();
        PrivateStep::step_count1 += 1;
      }
    }
  }
}

void PrivateStep::rot_p1(int dir){
  PrivateStep::rot_dir1 = dir;
  PrivateStep::frequency1 = PrivateStep::speed1 / PrivateStep::basestep;
  PrivateStep::sum_width1 = 1 / PrivateStep::frequency1;
  PrivateStep::pulse_width1 = PrivateStep::sum_width1 / 2;
  digitalWrite(PrivateStep::lock1, LOW);
  PrivateStep::enable1 = true;
  if (PrivateStep::rot_dir1 == 0) {
    TimedAction pulse1 = TimedAction(PrivateStep::sum_width1,PrivateStep::cw1);
    while (PrivateStep::enable1) {
      pulse1.check();
    }
  } else if (PrivateStep::rot_dir1 == 1) {
    TimedAction pulse1 = TimedAction(PrivateStep::sum_width1,PrivateStep::cw1);
    while (PrivateStep::enable1) {
      pulse1.check();
    }
  }
  
}

void PrivateStep::stop_p1(){
  PrivateStep::enable1 = false;
}

void PrivateStep::rot_p2(int dir){
  PrivateStep::rot_dir2 = dir;
  PrivateStep::frequency2 = PrivateStep::speed2 / PrivateStep::basestep;
  PrivateStep::sum_width2 = 1 / PrivateStep::frequency2;
  PrivateStep::pulse_width2 = PrivateStep::sum_width2 / 2;
  digitalWrite(PrivateStep::lock2, LOW);
  PrivateStep::enable2 = true;
  if (PrivateStep::rot_dir2 == 0) {
    TimedAction pulse2 = TimedAction(PrivateStep::sum_width2,PrivateStep::cw2);
    while (PrivateStep::enable2) {
      pulse2.check();
    }
  } else if (PrivateStep::rot_dir2 == 1) {
    TimedAction pulse2 = TimedAction(PrivateStep::sum_width2,PrivateStep::cw2);
    while (PrivateStep::enable2) {
      pulse2.check();
    }
  }
  
}

void PrivateStep::stop_p2(){
  PrivateStep::enable2 = false;
}

void PrivateStep::rot_p_both(int num){
  PrivateStep::step_amount1 = PrivateStep::degree1 / PrivateStep::basestep;
  PrivateStep::frequency1 = PrivateStep::speed1 / PrivateStep::basestep;
  PrivateStep::sum_width1 = 1 / PrivateStep::frequency1;
  PrivateStep::pulse_width1 = PrivateStep::sum_width1 / 2;
  PrivateStep::step_count1 = 0;
  PrivateStep::step_amount2 = PrivateStep::degree2 / PrivateStep::basestep;
  PrivateStep::frequency2 = PrivateStep::speed2 / PrivateStep::basestep;
  PrivateStep::sum_width2 = 1 / PrivateStep::frequency2;
  PrivateStep::pulse_width2 = PrivateStep::sum_width2 / 2;
  PrivateStep::step_count2 = 0;
  digitalWrite(PrivateStep::lock1, LOW);
  digitalWrite(PrivateStep::lock2, LOW);
  if (num == 3) {
    TimedAction pulse1 = TimedAction(PrivateStep::sum_width1,PrivateStep::cw1);
    TimedAction pulse2 = TimedAction(PrivateStep::sum_width2,PrivateStep::ccw2);
    while (PrivateStep::enable1 || PrivateStep::enable2) {
      pulse1.check();
      pulse2.check();
    }
  } else if (num == 4) {
    TimedAction pulse1 = TimedAction(PrivateStep::sum_width1,PrivateStep::ccw1);
    TimedAction pulse2 = TimedAction(PrivateStep::sum_width2,PrivateStep::cw2);
    while (PrivateStep::enable1 || PrivateStep::enable2) {
      pulse1.check();
      pulse2.check();
    }
  } else if (num == 5) {
    TimedAction pulse1 = TimedAction(PrivateStep::sum_width1,PrivateStep::ccw1);
    TimedAction pulse2 = TimedAction(PrivateStep::sum_width2,PrivateStep::ccw2);
    while (PrivateStep::enable1 || PrivateStep::enable2) {
      pulse1.check();
      pulse2.check();
    }
  } else if (num == 6) {
    TimedAction pulse1 = TimedAction(PrivateStep::sum_width1,PrivateStep::cw1);
    TimedAction pulse2 = TimedAction(PrivateStep::sum_width2,PrivateStep::cw2);
    while (PrivateStep::enable1 || PrivateStep::enable2) {
      pulse1.check();
      pulse2.check();
    }
  } else {
    TimedAction pulse1 = TimedAction(PrivateStep::sum_width1,PrivateStep::do_lock1);
    TimedAction pulse2 = TimedAction(PrivateStep::sum_width2,PrivateStep::do_lock2);
    while (PrivateStep::enable1 || PrivateStep::enable2) {
      pulse1.check();
      pulse2.check();
    }
  }
}

void PrivateStep::stop_p_both(){
  PrivateStep::enable1 = false;
  PrivateStep::enable2 = false;
}