#define M5STACK_MYBUILD

//#if defined(ARDUINO_MYBUILD)
//  #include <MPU9250.h>
//  #define SERIAL_BUFFER_SIZE 256
//  const int right_cw = 8;
//  const int right_ccw = 9;
//  const int right_lock = 6;
//  const int left_cw = 10;
//  const int left_ccw = 11;
//  const int left_lock = 7;
//  const int upper_sv = 4;
//  const int lower_sv = 5;
//  const int pulse_pin = 14;
//  const int temp_pin = 15;
//  const int line_tracer = 16;
//  const int rot_angle = 17;
//  const int check_pin = 12;
//#endif

#if defined(M5STACK_MYBUILD)
  #include <M5Stack.h>
  #include <SoftwareSerial.h>
  #define M5STACK_MPU6886
  const int right_cw = 16;
  const int right_ccw = 17;
  const int right_lock = 6;
  const int left_cw = 2;
  const int left_ccw = 5;
  const int left_lock = 7;
  const int upper_sv = 4;
  const int lower_sv = 5;
  const int pulse_pin = 14;
  const int temp_pin = 15;
  const int line_tracer = 12;
  const int rot_angle = 13;
  const int check_pin = 26;
#endif

#include <EEPROM.h>
#include <TimedAction.h>
#include "PrivateStep.h"
#include "PrivateServo.h"
#include "struct_and_union.h"
#include "Stepping.h"


const int debug = 0;
const int receive_data_size = 24;
const int transmit_data_size = 42;
int current_unique_id = 0;

#if defined(ARDUINO_MYBUILD)
MPU9250 IMU(Wire,0x68);
#endif

int status;
PrivateServo servo = PrivateServo();
PrivateStep Stepping2 = PrivateStep();
Stepping Step = Stepping();

#if defined(ARDUINO_MYBUILD)
  TimedAction getPacket = TimedAction(1,receivePacket);
  TimedAction takeAction = TimedAction(1,work);
#endif

void setup() {
  // put your setup code here, to run once:-
#if defined(M5STACK_MYBUILD)
  M5.begin();
#endif
  Serialsetup();
  pinAssign();
  Serial.println("before ltsetup");
  LTsetup();
//  IMUcheck();
//  IMUsetup();
  Serial.println("before motorsetup");
  //servo.setup(upper_sv, float(90), lower_sv, float(90));
  //Step.setup(right_cw, left_cw, right_ccw, left_ccw, right_lock, left_lock);
  Serial.println("before multitask");
  #if defined(M5STACK_MYBUILD)
    //xTaskCreatePinnedToCore(task0, "Task0", 4096, NULL, 1, NULL, 0);
    //xTaskCreatePinnedToCore(task1, "Task1", 4096, NULL, 1, NULL, 1);
  #endif
  Serial.println("setup complete");
}

#if defined(M5STACK_MYBUILD)
void task0(void* arg) {
  long now = micros();
  while (1) {
    if (micros() - now >= 1000) {
      receivePacket();
      now = micros();
    }
  }
}

void task1(void* arg) {
  long now = micros();
  while (1) {
    if (micros() - now >= 1000) {
      work();
      now = micros();
    }
  }
}
#endif

void Serialsetup() {
#if defined(ARDUINO_MYBUILD)
  Serial.begin(9600);
#endif
#if defined(M5STACK_MYBUILD)
  Serial.begin(115200);
#endif
  while(!Serial) {}
  Serial.println("Transmission Start");
  Serial.flush();
  byte trush;
  while(Serial.available() > 0){
    trush = Serial.read();
  }
}

void pinAssign() {
  pinMode(right_cw, OUTPUT);
  pinMode(right_ccw, OUTPUT);
  pinMode(right_lock, OUTPUT);
  pinMode(left_cw, OUTPUT);
  pinMode(left_ccw, OUTPUT);
  pinMode(left_lock, OUTPUT);
  pinMode(upper_sv, OUTPUT);
  pinMode(lower_sv, OUTPUT);
  pinMode(pulse_pin, INPUT);
  pinMode(temp_pin, INPUT);
  pinMode(line_tracer, OUTPUT);
  pinMode(rot_angle, OUTPUT);
  pinMode(check_pin,INPUT);
}

//void IMUcheck() {
//  status = IMU.begin();
//  if (status > 0) {
//    Serial.println(">IMU initialization unsuccessful");
//    Serial.flush();
//    Serial.println(">Check IMU wiring or try cycling power");
//    Serial.flush();
//    Serial.print(">Status: ");
//    Serial.flush();
//    Serial.println(status);
//    Serial.flush();
//    while(1) {}
//  }
//}
//
//void IMUsetup(){
//  IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
//  IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
//  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
//  IMU.setSrd(19);
//}

void LTsetup() {
  if(digitalRead(check_pin) == HIGH) {
    Serial.println(">Line tracer Setup Start");
    Serial.flush();
    Serial.println(">pls connect check pin and GND to get the brightness on the line");
    Serial.flush();
    while(digitalRead(check_pin) == HIGH){
      delay(100);
    }
    delay(1000);
    int onLine = analogRead(line_tracer);
    Serial.println(">OK. pls connect check pin and VCC to get the brightness on the load");
    Serial.flush();
    while(digitalRead(check_pin) == LOW) {
      delay(100);
    }
    delay(1000);
    int onLoad = analogRead(line_tracer);
    int bright_thresh = (onLine + onLoad) /2;
    EEPROM.write(0,bright_thresh);
    Serial.println(">Setup Complete");
    Serial.flush();
    Serial.print(">Brightness threshold : ");
    Serial.flush();
    Serial.println(bright_thresh);
    Serial.flush();
  }
}

void loop() {
#if defined(ARDUINO_MYBUILD)
  getPacket.check();
  takeAction.check();
#endif
#if defined(M5STACK_MYBUILD)
  delay(1000);
#endif
}

Transmit_Packet *t_head;
Receive_Packet *r_head;
