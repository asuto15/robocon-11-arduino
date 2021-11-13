#include <EEPROM.h>
#include <MPU9250.h>
#include <TimedAction.h>
#include "PrivateStep.h"
#include "PrivateServo.h"
#define right_cw 4
#define right_ccw 5
#define right_lock 10
#define left_cw 6
#define left_ccw 7
#define left_lock 11
#define upper_sv 8
#define lower_sv 9
#define pulse_pin 14
#define temp_pin 15
#define line_tracer 16
#define rot_angle 17
#define check_pin 12

const int debug = 0;
const int receive_data_size = 24;
const int transmit_data_size = 42;
int current_unique_id = 0;
MPU9250 IMU(Wire,0x68);
int status;
PrivateServo servo = PrivateServo();
PrivateStep Stepping = PrivateStep(); 

void setup() {
  // put your setup code here, to run once:-
  Serial.begin(9600);
  pinAssign();
  if(digitalRead(check_pin) == HIGH) {
    LTsetup();
  }
  IMUcheck();
  IMUsetup();
  while(!Serial) {}
  Serial.println("Transmission Start");
  Serial.flush();
  servo.setup(upper_sv, float(90), lower_sv, float(90));
  Stepping.setup(right_cw, right_ccw, right_lock, left_cw, left_ccw, left_lock);
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

void IMUcheck() {
  status = IMU.begin();
  if (status > 0) {
    Serial.println(">IMU initialization unsuccessful");
    Serial.flush();
    Serial.println(">Check IMU wiring or try cycling power");
    Serial.flush();
    Serial.print(">Status: ");
    Serial.flush();
    Serial.println(status);
    Serial.flush();
    while(1) {}
  }
}

void IMUsetup(){
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  IMU.setSrd(19);
}

void LTsetup() {
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

void loop() {
  // put your main code here, to run repeatedly:
  int data0 = 0.05;
  int data1 = 360;
  int signaltype = 30;
  int rot_dir = 3;
  Stepping.setDegrees(data0, (float)data1, data0, (float)data1);
  Stepping.work(signaltype, rot_dir);
}
