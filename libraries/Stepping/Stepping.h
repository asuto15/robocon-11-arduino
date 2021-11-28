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
    void turn_right(float intrvl, int forward, int right);
    void turn_left(float intrvl, int forward, int left);
    void turn_right(float intrvl);
    void turn_left(float intrvl);
    void right_f(float ang_vel, float angle);
    void right_f_t(float ang_vel, float time);
    void right_b(float ang_vel, float angle);
    void right_b_t(float ang_vel, float time);
    void left_f(float ang_vel, float angle);
    void left_f_t(float ang_vel, float time);
    void left_b(float ang_vel, float angle);
    void left_b_t(float ang_vel, float time);
    void move_forward(float ang_vel, float step_amnt);
    void move_forward_t(float ang_vel, float time);
    void move_backward(float ang_vel, float step_amnt);
    void move_backward_t(float ang_vel, float time);
    void move_right(float ang_vel_r, float angle_r, float ang_vel_l, float angle_l);
    void move_right_t(float ang_vel_r, float time_r, float ang_vel_l, float time_l);
    void move_left(float ang_vel_r, float angle_r, float ang_vel_l, float angle_l);
    void move_left_t(float ang_vel_r, float time_r, float ang_vel_l, float time_l);
    void lock_right();
    void unlock_right();
    void lock_left();
    void unlock_left();
    void lock_both();
    void unlock_both();
    void rotate(int rot_dir_r, int rot_dir_l, float intrvl);
    void rot_independent(int rot_dir_r, float ang_vel_r, float angle_r, int rot_dir_l, float ang_vel_l, float angle_l);
    int cw_r;
    int cw_l;
    int ccw_r;
    int ccw_l;
    int lock_r;
    int lock_l;
    float angular_vel;
    float angular_vel_r;
    float angular_vel_l;
    float interval;
    float interval_r;
    float interval_l;
    long step_amount;
    long step_amount_r;
    long step_amount_l;
    long step_count_r;
    long step_count_l;
    long previous_r;
    long previous_l;
    bool enable;
    float step_degree = 0.018;//(°)
    float tire_radius = 45;//(mm)
};

#endif