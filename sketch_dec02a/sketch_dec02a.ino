#include <Arduino.h>
#include <M5Stack.h>
#include <M5StackUpdater.h>  // version 0.5.2

#include <EEPROM.h>

const int line_tracer = 2;
int bright_thresh = EEPROM.read(0);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  int lt_value = analogRead(line_tracer);
  if (lt_value >= bright_thresh){
    Serial.printf(">#{\"SignalType\" : %d, \"UniqueID\" : %d, \"AccelX\" : %.2f, \"AccelY\" : %.2f, \"AccelZ\" : %.2f, \
        \"GyroX\" : %.2f, \"GyroY\" : %.2f, \"GyroZ\" : %.2f, \"Direction\" : %.2f, \"Temperature\" : %.2f, \"LineTracer\" : %.2f}\n",
        (int)90, (int)0, (float)0,
        (float)0, (float)0, (float)0,
        (float)0, (float)0, (float)0,
        (int)0, (int)1);
  }
  M5.Lcd.printf("%d",lt_value);
}
