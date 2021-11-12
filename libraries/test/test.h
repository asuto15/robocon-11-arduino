#ifndef TEST
#define TEST
#include <Arduino.h>

class Test {
public:
  Test();
  
void on();
void off();
void execute(int num);

private:
  bool enable;
};

#endif