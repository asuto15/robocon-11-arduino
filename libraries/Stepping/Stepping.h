#ifndef STEPPING_H
#define STEPPING_H

#include <Arduino.h>

class Stepping {

  public:
    Stepping();
    void setup(int cw1, int cw2, int ccw1, int ccw2, int awo1, int awo2);
    void set_step_degree(float deg);
    void forward(float intrvl);
    void backward(float intrvl);
    void right_f(float intrvl);
    void right_b(float intrvl);
    void left_f(float intrvl);
    void left_b(float intrvl);
    void delayMilliseconds(float intrvl);
    void go_forward(float intrvl);
    void go_backward(float intrvl);
    void turn_right(float intrvl, int forward, int right);
    void turn_left(float intrvl, int forward, int left);
    void turn_right(float intrvl);
    void turn_left(float intrvl);
    void right_f_d(float ang_vel, float dist);
    void right_f_t(float ang_vel, float time);
    void right_b_d(float ang_vel, float dist);
    void right_b_t(float ang_vel, float time);
    void left_f_d(float ang_vel, float dist);
    void left_f_t(float ang_vel, float time);
    void left_b_d(float ang_vel, float dist);
    void left_b_t(float ang_vel, float time);
    void move_forward_d(float ang_vel, float step_amnt);
    void move_forward_t(float ang_vel, float time);
    void move_backward_d(float ang_vel, float step_amnt);
    void move_backward_t(float ang_vel, float time);
    void move_right_d(float ang_vel_r, float dist_r, float ang_vel_l, float dist_l);
    void move_right_t(float ang_vel_r, float time_r, float ang_vel_l, float time_l);
    void move_left_d(float ang_vel_r, float dist_r, float ang_vel_l, float dist_l);
    void move_left_t(float ang_vel_r, float time_r, float ang_vel_l, float time_l);
    void lock_right();
    void unlock_right();
    void lock_left();
    void unlock_left();
    void lock_both();
    void unlock_both();
    int cw_r;
    int cw_l;
    int ccw_r;
    int ccw_l;
    int lock_r;
    int lock_l;
    float interval;
    float interval_r;
    float interval_l;
    long step_amount;
    long step_amount_r;
    long step_amount_l;
    // long step_amount_r;
    // long step_amount_l;
    float step_degree = 0.18;//(°)
    float tire_radius = 9.0;//(cm)
};

#endif