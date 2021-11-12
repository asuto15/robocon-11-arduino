#ifndef PRIVATESERVO_H
#define PRIVATESERVO_H

#include <Arduino.h>

class PrivateServo {

  public:
    PrivateServo();
    void setup(int pin);
    void setup(int pin1, int pin2);
    void setup(int pin, float degree);
    void setup(int pin1, float degree1, int pin2, float degree2);
    void setDegree1(float degree1);
    void setDegree2(float degree2);
    void resetDegree();
    void setDegrees(float degree1, float degree2);
    void disable();
    void enable();
    void work();

  private:
    bool active;
    int pin1;
    int pin2;
    float degree1;
    float degree2;
    float pulse_width1;
    float pulse_width2;
    float sum_width;
    long current_time;
};

#endif