#define M5STACK_MPU6886

#include <M5Stack.h>
#include <EEPROM.h>
#include <MPU9250.h>
#include "PrivateStep.h"
#include "PrivateServo.h"
#include "struct_and_union.h"

#define RAD_TO_DEG 57.324
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
//PrivateStep Stepping = PrivateStep();
Stepping Step = Stepping();
servo.setup(upper_sv, float(90), lower_sv, float(90));
Step.setup(right_cw, right_ccw, right_lock, left_cw, left_ccw, left_lock);
  
void setup() {
  // put your setup code here, to run once:-
  pinAssign();
  LTsetup();
  Serialsetup();
  IMUsetup();
  M5setup();
  xTaskCreatePinnedToCore(task0, "Task0", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(task1, "Task1", 4096, NULL, 1, NULL, 1);
}

void task0(void* arg) {
  while (1) {
    long now = millis();
    if (millis() - now >= 10) {
      receivePacket();
    }
  }
}

void task1(void* arg) {
  while (1) {
    long now = millis();
    if (millis() - now >= 10) {
      work();
    }
  }
}

void Serialsetup() {
  Serial.begin(9600);
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

void IMUsetup() {
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
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  IMU.setSrd(19);
}

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

void M5setup() {
  M5.begin();
  M5.Power.begin();
  M5.IMU.Init();
}

void loop() {
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
    if (debug == 3) {
      for (int i=0; i<24; i++) {
        receive_data[i] = 0;
      }
    } else {
      received_data_size = byte(Serial.readBytes(receive_data,receive_data_size));
      Serial.print(">Data");
      for (int i=0; i<receive_data_size; i++){
        Serial.print(receive_data[i]);
      }
    }
//    Serial.println(">read");
//    Serial.flush();
    if (received_data_size != byte(receive_data_size)) {
      Serial.write(received_data_size);
      Serial.flush();
      if (debug) {
        Serial.println(">Invalid data size");
        Serial.flush();
//        Serial.print(">Datasize : ");
//        Serial.flush();
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
//      Serial.println(">Buffer cleared");
//      Serial.flush();
      //interrupts();
      free(r_packet);
      return;  
    }
    r_packet->signaltype = ((receive_data[0] << 8)+(receive_data[1]));
    r_packet->unique_id = ((receive_data[2] << 24)+(receive_data[3] << 16)+(receive_data[4] << 8)+(receive_data[5]));
    r_packet->rot_dir = receive_data[6];
    r_packet->datatype = receive_data[7];
    for (int i = 0; i < 4; i++){
      r_packet->data[i] = ((receive_data[4*i+8] << 24)+(receive_data[4*i+9] << 16)+(receive_data[4*i+10] << 8)+(receive_data[4*i+11]));
    }
    if (r_packet->unique_id != current_unique_id) { //if (r_packet->unique_id != 0x00000000) {
      if (debug) {
        Serial.print(">Invalid unique_id : ");
        Serial.flush();
        Serial.print(r_packet->unique_id);
        Serial.flush();
//        for (int i=0; i<receive_data_size; i++) {
//          Serial.print(receive_data[i]);
//          Serial.write(0x2E);
//        }
//        Serial.print((byte)(r_packet->signaltype >> 8) & 0xFF);
//        Serial.write(0x3A);
//        Serial.print((byte)(r_packet->signaltype & 0xFF));
//        Serial.write(0x3A);
//        Serial.print((byte)(r_packet->unique_id >> 24) & 0xFF);
//        Serial.write(0x3A);
//        Serial.print((byte)(r_packet->unique_id >> 16) & 0xFF);
//        Serial.write(0x3A);
//        Serial.print((byte)(r_packet->unique_id >> 8) & 0xFF);
//        Serial.write(0x3A);
//        Serial.print((byte)(r_packet->unique_id & 0xFF));
//        Serial.write(0x3A);
//        for (int i=0; i<4; i++) {
//          Serial.print((byte)((long)r_packet->data[i] >> 24) & 0xFF);
//          Serial.write(0x3A);
//          Serial.print((byte)((long)r_packet->data[i] >> 16) & 0xFF);
//          Serial.write(0x3A);
//          Serial.print((byte)((long)r_packet->data[i] >> 8) & 0xFF);
//          Serial.write(0x3A);
//          Serial.print((byte)((long)r_packet->data[i] & 0xFF));
//          Serial.write(0x3A);
//        }
//        Serial.write(0x3D);
//        for (int i = 0; i < receive_data_size; i++) {
//          receive_data[i] = byteSwap(receive_data[i]);
//          Serial.write(receive_data[i]);
//          Serial.flush();
//        }
      }
      free(r_packet);
      return;
    }
    current_unique_id += 1;
    if (debug) {
      Serial.print(">unique_id : ");
      Serial.flush();
    }
    Serial.write((byte)(r_packet->unique_id >> 24) & 0xFF);
    Serial.flush();
    Serial.write((byte)(r_packet->unique_id >> 16) & 0xFF);
    Serial.flush();
    Serial.write((byte)(r_packet->unique_id >> 8) & 0xFF);
    Serial.flush();
    Serial.write((byte)(r_packet->unique_id & 0xFF));
    Serial.flush();
    Serial.println("");
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
    int signaltype = r_head->signaltype;
    int datatype = r_head->datatype;
    int rot_dir = r_head->rot_dir;
    float bright_thresh;
    float accX, accY, accZ;
    float gyroX, gyroY, gyroZ;
    float pitch, roll, yaw;
    float Temp;
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
        Serial.print(">30willwork");
        Serial.println(datatype);
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
            Serial.print(">rot");
            Stepping.rot_p_both(rot_dir);
            Serial.print(">rotted");
            break;
          case 3:
            Stepping.stop_p_both();
          default:
            break;
        }
        Serial.println(">30worked");   
        break;
      case 40:
        M5.IMU.getTempData(&Temp);
        if (status) {
          t_packet->data[0] = get_dist(Temp);
        } else if (t_packet->data[0] > 10 & t_packet->data[0] < 40) {
          t_packet->data[0] = get_dist(t_packet->data[0]);
        } else {
          t_packet->data[0] = -1;
        }
        break;
      case 50:
        bright_thresh = EEPROM.read(0);
        t_packet->data[0] = (digitalRead(line_tracer) >= bright_thresh)?1:0;
        Serial.println(">50worked");
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
        M5.IMU.getGyroData(&gyroX,&gyroY,&gyroZ);
        M5.IMU.getAccelData(&accX,&accY,&accZ);
        M5.IMU.getAhrsData(&pitch,&roll,&yaw);
        t_packet->data[0] = accX;
        t_packet->data[1] = accY;
        t_packet->data[2] = accZ;
        t_packet->data[3] = gyroX;
        t_packet->data[4] = gyroY;
        t_packet->data[5] = gyroZ;
        t_packet->data[6] = pitch;
        t_packet->data[7] = roll;
        t_packet->data[8] = yaw;
        break;
      default:
        break;
    }
    Transmit_Packet *t_packet;
    if ((t_packet = (Transmit_Packet *) malloc(sizeof(Transmit_Packet))) == NULL) {
        Serial.println(">malloc error with transmit_packet");
        Serial.flush();
        return;
    }
    t_packet->signaltype = r_head->signaltype;
    t_packet->unique_id = r_head->unique_id;
    for (int i=0; i<4; i++) {
      t_packet->data[i] = r_head->data[i];
    }
    for (int i=4; i<9; i++) {
      t_packet->data[i] = 0;
    }
    Receive_Packet *buf = r_head->next;
    free(r_head);
    r_head = buf;
    
    serialTransmit(t_packet);
  }
}

void serialTransmit(Transmit_Packet *t_packet){
  if (t_packet != NULL) {
    Serial.write((byte)(t_packet->signaltype >> 8) & 0xFF);
    Serial.flush();
    Serial.write((byte)(t_packet->signaltype & 0xFF));
    Serial.flush();
    Serial.write((byte)(t_packet->unique_id >> 24) & 0xFF);
    Serial.flush();
    Serial.write((byte)(t_packet->unique_id >> 16) & 0xFF);
    Serial.flush();
    Serial.write((byte)(t_packet->unique_id >> 8) & 0xFF);
    Serial.flush();
    Serial.write((byte)(t_packet->unique_id & 0xFF));
    Serial.flush();
    for (int i=0; i<9; i++) {
      Serial.write((byte)((long)t_packet->data[i] >> 24) & 0xFF);
      Serial.flush();
      Serial.write((byte)((long)t_packet->data[i] >> 16) & 0xFF);
      Serial.flush();
      Serial.write((byte)((long)t_packet->data[i] >> 8) & 0xFF);
      Serial.flush();
      Serial.write((byte)((long)t_packet->data[i] & 0xFF));
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
