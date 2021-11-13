#include "test.h"

Test::Test() {
    Serial.begin(9600));
    enable = true;
}

void Test::execute(int num) {
    if (enable) {
      Serial.println(num);
    }
}

void Test::on(){
    enable = true;
}

void Test::off(){
    enable = false;
}