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
int r_erasable = 0;
int t_erasable = 0;
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
  LTsetup();
//  IMUcheck();
//  IMUsetup();
  servo.setup(upper_sv, float(90), lower_sv, float(90));
  Step.setup(right_cw, left_cw, right_ccw, left_ccw, right_lock, left_lock);
  #if defined(M5STACK_MYBUILD)
    xTaskCreatePinnedToCore(task0, "Task0", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(task1, "Task1", 4096, NULL, 1, NULL, 1);
  #endif
}

#if defined(M5STACK_MYBUILD)
void task0(void* arg) {
  while (1) {
    long now = millis();
    if (millis() - now >= 100) {
      receivePacket();
    }
  }
}

void task1(void* arg) {
  while (1) {
    long now = millis();
    if (millis() - now >= 100) {
      work();
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
Transmit_Packet *t_packet;
Transmit_Packet *t_tail;
Receive_Packet *r_head;
Receive_Packet *r_packet;
Receive_Packet *r_current;
Receive_Packet *r_tail;
Receive_Packet *buf;
r_head = NULL;
r_current = NULL;

void receivePacket(){
  while (r_erasable > 0){
    free(r_head);
    r_head = buf;
    r_erasable--;
  }
  if (t_erasable == 0){
    free(t_packet);
    t_erasable = false;
  }
  if (Serial.available() > 0) {
    Receive_Packet *r_packet;
    if ((r_packet = (Receive_Packet *) malloc(sizeof(Receive_Packet))) == NULL) {
      Serial.println(">malloc error with receive_packet");
      Serial.flush();
      return;
    }
    if (debug) {
      Serial.print(">rs : ");
      Serial.flush();
      Serial.println(micros());
      Serial.flush();  
    }
    byte receive_data[receive_data_size];
    //noInterrupts();
    byte received_data_size;
    if (debug == 2) {
      for (int i=0; i<24; i++) {
        receive_data[i] = 0;
      }
    } else {
      received_data_size = byte(Serial.readBytes(receive_data,receive_data_size));
//      Serial.print(">Data");
//      for (int i=0; i<receive_data_size; i++){
//        Serial.print(":");
//        Serial.print(receive_data[i]);
//      }      
//      Serial.println("");
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
      r_packet->data[i] = *(float *)(receive_data+4*i+8);
    }
    if (r_packet->unique_id != current_unique_id) { //if (r_packet->unique_id != 0x00000000) {
      if (debug) {
        Serial.print(">Invalid unique_id : ");
        Serial.flush();
        Serial.print(r_packet->unique_id);
        Serial.flush();
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
    Serial.flush();
    if (r_head == NULL) {
      r_head = r_packet;
    } else {
      r_tail = r_head;
      while (r_tail->next != NULL) {
        r_tail = r_tail->next;
      }
      r_tail->next = r_packet;
    }
    if (debug) {
      Serial.print(">re : ");
      Serial.flush();
      Serial.println(micros());
      Serial.flush();
    }
    return;

  } else if (Serial.available() == 0){
    //receivePacket();
    return;

  } else {
    return;
  }
}

void work(){
  if (r_erasable == true) {
    return;
  }
  if (r_head == NULL) {
    return;
  } else {
    if (r_current == NULL){
      r_current = r_head;
    } else {
      r_current = r_current->next;
    }
    Transmit_Packet *t_packet;
    if ((t_packet = (Transmit_Packet *) malloc(sizeof(Transmit_Packet))) == NULL) {
        Serial.println(">malloc error with transmit_packet");
        Serial.flush();
        return;
    }
    if (debug) {
      Serial.print(">ws : ");
      Serial.flush();
      Serial.println(micros());
      Serial.flush();
    }
    t_packet->signaltype = r_current->signaltype;
    t_packet->unique_id = r_current->unique_id;
    int signaltype = r_current->signaltype;
    int datatype = r_current->datatype;
    int rot_dir = r_current->rot_dir;
    float bright_thresh;
    float accX, accY, accZ;
    float gyroX, gyroY, gyroZ;
    float pitch, roll, yaw;
    float Temp;
    float data[4];

    for (int i=0; i<4; i++) {
      t_packet->data[i] = r_current->data[i];
      data[i] = r_current->data[i];
    }
    for (int i=4; i<9; i++) {
      t_packet->data[i] = 0;
    }
    buf = r_current->next;
    r_erasable = true;
    if (t_head == NULL) {
      t_head = t_packet;
    } else {
      t_tail = t_head;
      while (t_tail->next != NULL) {
        t_tail = t_tail->next;
      }
      t_tail->next = t_packet;
    }
    switch (signaltype) {
      case 10:
      case 20:
      case 30:
      case 60:
      case 70:
        serialTransmit(t_packet);
        break;
    }
    switch (signaltype) {
      case 10:
        switch(rot_dir) {
          case 0:
            switch(datatype) {
              case 0:
                Step.right_f(data[0],data[1]);
                break;
              case 1:
                Step.right_f(data[0],data[1]);
                break;
            }
            break;
          case 1:
            switch(datatype) {
              case 0:
                Step.right_b(data[0],data[1]);
                break;
              case 1:
                Step.right_b(data[0],data[0]*data[1]);
                break;
            }
            break;
          case 2:
            Step.lock_right();
            break;
        }
        break;
      case 20:
        switch(rot_dir) {
          case 0:
            switch(datatype) {
              case 0:
                Step.left_f(data[0],data[1]);
                break;
              case 1:
                Step.left_f(data[0],data[0]*data[1]);
                break;
            }
            break;
          case 1:
            switch(datatype) {
              case 0:
                Step.left_b(data[0],data[1]);
                break;
              case 1:
                Step.left_b(data[0],data[0]*data[1]);
                break;
            }
            break;
          case 2:
            Step.lock_left();
            break;
        }
        break;
      case 30:
        switch(rot_dir) {
          case 2:
            Step.lock_both();
          case 3:
            switch(datatype) {
              case 0:
                Step.move_forward(data[0], data[1]);
                break;
              case 1:
                Step.move_forward(data[0], data[0]*data[1]);
                break;
            }
            break;
          case 4:
            switch(datatype) {
              case 0:
                Step.move_backward(data[0], data[1]);
                break;
              case 1:
                Step.move_backward(data[0], data[0]*data[1]);
                break;
            }
            break;
          case 5:
            switch(datatype) {
              case 0:
                Step.move_right(data[0], data[1],data[2], data[3]);
                break;
              case 1:
                Step.move_right(data[0], data[0]*data[1],data[2], data[3]);
                break;
            }
            break;
          case 6:
            switch(datatype) {
              case 0:
                Step.move_left(data[0], data[1],data[2], data[3]);
                break;
              case 1:
                Step.move_left(data[0], data[0]*data[1],data[2], data[3]);
                break;
            }
            break;
        }
        break;
      case 40:
        if (status) {
          t_packet->data[0] = get_dist(20);
        } else if (t_packet->data[0] > 10 & t_packet->data[0] < 40) {
          t_packet->data[0] = get_dist(t_packet->data[0]);
        } else {
          t_packet->data[0] = -1;
        }
        serialTransmit(t_packet);
        break;
      case 50:
        bright_thresh = EEPROM.read(0);
        t_packet->data[0] = (digitalRead(line_tracer) >= bright_thresh)?1:0;
        serialTransmit(t_packet);
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
        t_packet->data[0] = accX;
        t_packet->data[1] = accY;
        t_packet->data[2] = accZ;
        t_packet->data[3] = gyroX;
        t_packet->data[4] = gyroY;
        t_packet->data[5] = gyroZ;
        t_packet->data[6] = pitch;
        t_packet->data[7] = roll;
        t_packet->data[8] = yaw;
        serialTransmit(t_packet);
        break;
    }
    if (debug) {
      Serial.print(">we : ");
      Serial.flush();
      Serial.println(micros());
      Serial.flush();
    }
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
    Serial.println("");
    Serial.flush();
    t_erasable = true;  
  }
}

int get_temp(){
  int analogIn = analogRead(temp_pin);
  int sensorVout = map(analogIn, 0, 1023, 0, 4600);
  int temperature = map(sensorVout, 350, 1450, -25, 80);
  return temperature;//�?
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
