#ifndef PRIVATESTEP_H
#define PRIVATESTEP_H

#include <Arduino.h>

class PrivateStep {

  public:
    PrivateStep();
    void setup(int cw1, int ccw1, int awo1, int cs1);
    void setup(int cw1, int ccw1, int awo1);
    void setup(int cw1, int ccw1, int awo1, int cs1, int cw2, int ccw2, int awo2, int cs2);
    void setup(int cw1, int ccw1, int awo1, int cw2, int ccw2, int awo2);
    void setTime(float spd1, float tim1);
    void setTimes(float spd1, float tim1, float spd2, float tim2);
    void setDegree(float spd1, float dgr1);
    void setDegrees(float spd1, float dgr1, float spd2, float dgr2);
    void pulse(int pin, int step_amount, float pulse_width);
    void work(int signaltype, int rot_dir);
    void work_p(int signaltype, int rot_dir);
    void rot(int pin);
    void lock(int pin);
    void unlock(int pin);
    void rot_both(int num);
    void rot_p1(int dir);
    void stop_p1();
    void rot_p2(int dir);
    void stop_p2();
    void rot_p_both(int num);
    void stop_p_both();
    static void do_lock1();
    static void do_lock2();

    static void cw1();
    static void ccw1();
    static void cw2();
    static void ccw2();
    static int forward1;
    static int forward2;
    static int back1;
    static int back2;
    static int lock1;
    static int lock2;
    static int change_step1;
    static int change_step2;
    static float speed1;
    static float speed2;
    static float time1;
    static float time2;
    static float degree1;
    static float degree2;
    static float basestep;
    static int step_amount1;
    static int step_amount2;
    static float frequency1;
    static float frequency2;
    static float sum_width1;
    static float sum_width2;
    static float pulse_width1;
    static float pulse_width2;
    static unsigned long current_time1;
    static unsigned long current_time2;
    static int step_count1;
    static int step_count2;
    static bool enable1;
    static bool enable2;
    static int pulse_pin;
    static int rot_pin;
    static int lock_pin;
    static int unlock_pin;
    static int rot_dir1;
    static int rot_dir2;
    static int signaltype;
    static int rot_dir;
};

#endif