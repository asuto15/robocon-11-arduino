#include <EEPROM.h>
#include <MPU9250.h>
#include <TimedAction.h>
#include "PrivateStep.h"
#include "PrivateServo.h"
#include "struct_and_union.h"
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

const int debug = 1;
const int receive_data_size = 24;
const int transmit_data_size = 42;
int current_unique_id = 0;
MPU9250 IMU(Wire,0x68);
int status;
TimedAction getPacket = TimedAction(10,receivePacket);
TimedAction takeAction = TimedAction(100,work);
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
  pinMode(check_pin,INPUT);
  pinMode(pulse_pin, INPUT);
  pinMode(temp_pin, INPUT);
  pinMode(right_cw, OUTPUT);
  pinMode(right_ccw, OUTPUT);
  pinMode(left_cw, OUTPUT);
  pinMode(left_ccw, OUTPUT);
  pinMode(upper_sv, OUTPUT);
  pinMode(lower_sv, OUTPUT);
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
  getPacket.check();
  takeAction.check();
}

Transmit_Packet *t_head;
Receive_Packet *r_head;


void receivePacket(){
  if (Serial.available() > 0) {
    Receive_Packet *r_packet;
    if ((r_packet = (Receive_Packet *) malloc(sizeof(Receive_Packet))) == NULL) {
      Serial.println(">malloc error with receive_packet");
      Serial.flush();
      return;
    }

    byte receive_data[receive_data_size];
    Serial.println(">Received");
    Serial.flush();
    //noInterrupts();
    byte received_data_size;
    if (debug == 2) {
      for (int i=0; i<24; i++) {
        receive_data[i] = 0;
      }
    } else {
      received_data_size = byte(Serial.readBytes(receive_data,receive_data_size));
    }
    Serial.println(">read");
    Serial.flush();
    if (received_data_size != byte(receive_data_size)) {
      Serial.write(received_data_size);
      Serial.flush();
      if (debug) {
        Serial.println(">Invalid data size");
        Serial.flush();
        Serial.print(">Datasize : ");
        Serial.flush();
        Serial.println(Serial.available());
        Serial.flush();
      }
      //noInterrupts();
      byte trush;
      while (Serial.available() > 0){
        trush = Serial.read();
        if (debug > 1) {
          Serial.print(char(trush));
          Serial.flush();
        }
      }
      Serial.println(">Buffer cleared");
      Serial.flush();
      //interrupts();
      free(r_packet);
      return;  
    }
    union byte2int B2I;
    for (int i = 0; i<2; i++){
      B2I.b[i] = receive_data[i];
    }
    r_packet->signaltype = B2I.integar;
    union byte2long B2L;
    for (int i = 0; i<4; i++){
      B2L.b[i] = receive_data[i+2];
    }
    r_packet->unique_id = B2L.number;
    r_packet->rot_dir = receive_data[6];
    r_packet->datatype = receive_data[7];
    union byte2float B2F1;
    union byte2float B2F2;
    union byte2float B2F3;
    union byte2float B2F4;
    for (int i = 0; i<4; i++){
      B2F1.b[i] = receive_data[i+8];
      B2F2.b[i] = receive_data[i+12];
      B2F3.b[i] = receive_data[i+16];
      B2F4.b[i] = receive_data[i+20];
    }
    r_packet->data[0] = B2F1.decimal;
    r_packet->data[1] = B2F2.decimal;
    r_packet->data[2] = B2F3.decimal;
    r_packet->data[3] = B2F4.decimal;
    if (r_packet->unique_id != 0x04d20000) { //if (r_packet->unique_id != current_unique_id) {
      if (debug) {
        Serial.print(">Invalid unique_id : ");
        Serial.flush();
        Serial.print(r_packet->unique_id);
        Serial.flush();
        Serial.write(r_packet->unique_id);
        Serial.flush();
        for (int i = 0; i < receive_data_size; i++) {
 //         receive_data[i] = byteSwap(receive_data[i]);
          Serial.write(receive_data[i]);
          Serial.flush();
        }
        union byte2long uniqueb2l2 = {0,0,0,0};
        uniqueb2l2.number = r_packet->unique_id;
        // uniqueb2l2.b[0] = byteSwap(uniqueb2l2.b[0]);
        // uniqueb2l2.b[1] = byteSwap(uniqueb2l2.b[1]);
        // uniqueb2l2.b[2] = byteSwap(uniqueb2l2.b[2]);
        // uniqueb2l2.b[3] = byteSwap(uniqueb2l2.b[3]);
        Serial.write(uniqueb2l2.b,4);
        Serial.flush();
      }
      free(r_packet);
      return;
    }
    current_unique_id += 1;
    if (debug) {
      Serial.print(">unique_id : ");
      Serial.flush();
      Serial.println(r_packet->unique_id);
      Serial.flush();
    }
    union byte2long uniqueb2l = {0,0,0,0};
    uniqueb2l.number = r_packet->unique_id;
    // uniqueb2l.b[0] = byteSwap(uniqueb2l.b[0]);
    // uniqueb2l.b[1] = byteSwap(uniqueb2l.b[1]);
    // uniqueb2l.b[2] = byteSwap(uniqueb2l.b[2]);
    // uniqueb2l.b[3] = byteSwap(uniqueb2l.b[3]);
    Serial.write(uniqueb2l.b,4);
    Serial.flush();
    Receive_Packet *r_tail;
    if (r_head == NULL) {
      r_head = r_packet;
    } else {
      r_tail = r_head;
      while (r_tail->next != NULL) {
        r_tail = r_tail->next;
      }
      r_tail->next = r_packet;
    }
    return;

  } else if (Serial.available() == 0){
    //receivePacket();
    return;

  } else {
    
  }
}

void work(){
  if (r_head == NULL) {
    return;
  } else {
    Transmit_Packet *t_packet;
    if ((t_packet = (Transmit_Packet *) malloc(sizeof(Transmit_Packet))) == NULL) {
        Serial.println(">malloc error with transmit_packet");
        Serial.flush();
        return;
    }
    t_packet->signaltype = r_head->signaltype;
    Serial.print(t_packet->signaltype);
    if (t_packet->signaltype == 80) {
      Serial.println("80 received");
      Serial.flush();
    }
    t_packet->unique_id = r_head->unique_id;
    int datatype = r_head->datatype;
    int rot_dir = r_head->rot_dir;
    for (int i=0; i<4; i++) {
      t_packet->data[i] = r_head->data[i];
    }
    for (int i=4; i<9; i++) {
      t_packet->data[i] = 0;
    }
    Receive_Packet *buf = r_head;
    r_head = r_head->next;
    free(buf);
    switch (t_packet->signaltype) {
      case 10:
        switch(datatype) {
          case 0:
            Stepping.setDegree(t_packet->data[0], t_packet->data[1]);
            Stepping.work(t_packet->signaltype, rot_dir);
            break;
          case 1:
            Stepping.setTime(t_packet->data[0], t_packet->data[1]);
            Stepping.work(t_packet->signaltype, rot_dir);
            break;
          case 2:
            Stepping.rot_p1(rot_dir);
            break;
          case 3:
            Stepping.stop_p1();
          default:
            break;
        }
        break;
      case 20:
        switch(datatype) {
          case 0:
            Stepping.setDegree(t_packet->data[0], t_packet->data[1]);
            Stepping.work(t_packet->signaltype, rot_dir);
            break;
          case 1:
            Stepping.setTime(t_packet->data[0], t_packet->data[1]);
            Stepping.work(t_packet->signaltype, rot_dir);
            break;
          case 2:
            Stepping.rot_p2(rot_dir);
            break;
          case 3:
            Stepping.stop_p2();
          default:
            break;
        }
        break;
      case 30:
        switch(datatype) {
          case 0:
            Stepping.setDegrees(t_packet->data[0], t_packet->data[1], t_packet->data[2], t_packet->data[3]);
            Stepping.work(t_packet->signaltype, rot_dir);
            break;
          case 1:
            Stepping.setTimes(t_packet->data[0], t_packet->data[1], t_packet->data[2], t_packet->data[3]);
            Stepping.work(t_packet->signaltype, rot_dir);
            break;
          case 2:
            Stepping.rot_p_both(rot_dir);
            break;
          case 3:
            Stepping.stop_p_both();
          default:
            break;
        }
        break;   
      case 40:
        IMU.readSensor();
        if (status) {
          t_packet->data[0] = get_dist(IMU.getTemperature_C());
        } else if (t_packet->data[0] > 10 & t_packet->data[0] < 40) {
          t_packet->data[0] = get_dist(t_packet->data[0]);
        } else {
          t_packet->data[0] = -1;
        }
        break;
      case 50:
        float bright_thresh = EEPROM.read(0);
        t_packet->data[0] = (digitalRead(line_tracer) >= bright_thresh)?1:0;
        break;
      case 60:
        servo.setDegree1(t_packet->data[0]);
        servo.work();
        break;
      case 70:
        servo.setDegree2(t_packet->data[0]);
        servo.work();
        break;
      case 80:
        IMU.readSensor();
        t_packet->data[0] = IMU.getAccelX_mss();
        t_packet->data[1] = IMU.getAccelY_mss();
        t_packet->data[2] = IMU.getAccelZ_mss();
        t_packet->data[3] = IMU.getGyroX_rads();
        t_packet->data[4] = IMU.getGyroY_rads();
        t_packet->data[5] = IMU.getGyroZ_rads();
        t_packet->data[6] = IMU.getMagX_uT();
        t_packet->data[7] = IMU.getMagY_uT();
        t_packet->data[8] = IMU.getMagZ_uT();
        break;
      default:
        break;
    }
    serialTransmit(t_packet);
  }
}

void serialTransmit(Transmit_Packet *t_packet){
  if (t_packet != NULL) {
    union byte2int signalb2i = {0,0};
    signalb2i.integar = t_packet->signaltype;
    // byteSwap(signalb2i.b[0]);
    // byteSwap(signalb2i.b[1]);
    Serial.write(signalb2i.b,2);
    Serial.flush();
    union byte2long uniqueb2l2 = {0,0,0,0};
    uniqueb2l2.number = t_packet->unique_id;
    // byteSwap(uniqueb2l2.b[0]);
    // byteSwap(uniqueb2l2.b[1]);
    // byteSwap(uniqueb2l2.b[2]);
    // byteSwap(uniqueb2l2.b[3]);
    Serial.write(uniqueb2l2.b,4);
    Serial.flush();
    union byte2float B2F = {0,0,0,0};
    for (int i = 0; i < 9; i++) {
      B2F.decimal = t_packet->data[i];
      // byteSwap(B2F.b[0]);
      // byteSwap(B2F.b[1]);
      // byteSwap(B2F.b[2]);
      // byteSwap(B2F.b[3]);
      Serial.write(B2F.b,4);
      Serial.flush();
    }
    free(t_packet);  
  }
}

int get_temp(){
  int analogIn = analogRead(temp_pin);
  int sensorVout = map(analogIn, 0, 1023, 0, 4600);
  int temperature = map(sensorVout, 350, 1450, -25, 80);
  return temperature;//℃
}

float get_dist(float temperature){
  float velocity = 331.5 + 0.61 * (float)temperature;//m/s
  int pulse_time = pulseIn(pulse_pin, HIGH, 20000);//us
  float distance = velocity * 1000 * (pulse_time / 2) / 1000000; //mm 
  return distance;
}

byte byteSwap(byte x){
  byte y = 0;
  for (int i = 0; i < 8; i++){
    bitWrite(y,i,bitRead(x,8-i));
  }
  x = y;
  return x;
}
