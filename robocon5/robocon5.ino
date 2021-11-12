#include <EEPROM.h>
//#include <MPU9250.h>
#include <TimedAction.h>
#include <PrivateStep.h>
#include <PrivateServo.h>

const int receive_data_size = 24;
const int transmit_data_size = 42;
int current_t_packet_id = 0;
int current_r_packet_id = 0;
#define right_cw 4
#define right_ccw 5
#define left_cw 6
#define left_ccw 7
#define upper_sv 8
#define lower_sv 9
#define right_lock 10
#define left_lock 11
#define check_pin 12
#define pulse_pin 14
#define temp_pin 15
#define line_tracer 16
#define rot_angle 17

//MPU9250 IMU(Wire,0x68);
int status;
long previous_unique_id = 0;

typedef struct r_packet_t Receive_Packet;//24bytes
struct r_packet_t{
  int signaltype;//2bytes
  long unique_id;//4bytes
  byte rot_dir;//1byte
  byte datatype;//1byte
  float data[4];//4*4=16bytes
  Receive_Packet *next;
};

typedef struct t_packet_t Transmit_Packet;//42bytes
struct t_packet_t{
  int signaltype;//2bytes
  long unique_id;//4bytes
  float data[9];//4*9=36bytes
  Transmit_Packet *next;
};

Receive_Packet *receivePacket();

void work();

void serialTransmit(Transmit_Packet *t_packet);

Transmit_Packet *t_head;
Receive_Packet *r_head;

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
  //IMUcheck();
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
  status = 1;//IMU.begin();
  if (status > 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.flush();
    Serial.println("Check IMU wiring or try cycling power");
    Serial.flush();
    Serial.print("Status: ");
    Serial.flush();
    Serial.println(status);
    Serial.flush();
    while(1) {}
  }
}

void LTsetup() {
  Serial.println("Line tracer Setup Start");
  Serial.flush();
  Serial.println("pls connect check pin and GND to get the brightness on the line");
  Serial.flush();
  while(digitalRead(check_pin) == HIGH){
    delay(100);
  }
  int onLine = analogRead(line_tracer);
  Serial.println("OK. pls connect check pin and VCC to get the brightness on the load");
  Serial.flush();
  while(digitalRead(check_pin) == LOW) {
    delay(100);
  }
  int onLoad = analogRead(line_tracer);
  int bright_thresh = (onLine + onLoad) /2;
  EEPROM.write(0,bright_thresh);
  Serial.println("Setup Complete");
  Serial.flush();
  Serial.print("Brightness threshold : ");
  Serial.flush();
  Serial.println(bright_thresh);
  Serial.flush();
}

void loop() {
  getPacket.check();
  takeAction.check();
}

Receive_Packet *receivePacket(){
  if (Serial.available() > 0) {
    Receive_Packet *r_packet;
    if ((r_packet = (Receive_Packet *) malloc(sizeof(Receive_Packet))) == NULL) {
      Serial.println("malloc error");
      Serial.flush();
      return 0;
    }
    Receive_Packet *r_tail;
    r_tail = r_head;
    byte receive_data[receive_data_size];
    Serial.println("Stop");
    Serial.flush();
    //noInterrupts();
    int received_data_size = Serial.readBytes(receive_data,receive_data_size);
    r_packet->signaltype = byte2int(receive_data[0]);
    r_packet->unique_id = byte2long(receive_data[2]);
    r_packet->rot_dir = receive_data[6];
    r_packet->datatype = receive_data[7];
    r_packet->data[0] = byte2float(receive_data[8]);
    r_packet->data[1] = byte2float(receive_data[12]);
    r_packet->data[2] = byte2float(receive_data[16]);
    r_packet->data[3] = byte2float(receive_data[20]);
    current_r_packet_id += 1;
    //Serial.println("");
    //interrupts();
    Serial.print("unique_id : ");
    Serial.flush();
    Serial.println(r_packet->unique_id);
    Serial.flush();
    //Serial.write(r_packet->unique_id);
    while (r_tail->next != NULL) {
      r_tail = r_tail->next;
    }
    r_tail->next = r_packet;
    return 0; 
  } else if (Serial.available() == 0){
    receivePacket();
  } else {
    Serial.print("Invalid data size : ");
    Serial.flush();
    Serial.println(Serial.available());
    Serial.flush();
    //noInterrupts();
      byte trush;
      while (Serial.available() > 0){
        trush = Serial.read();
        Serial.print(char(trush));
        Serial.flush();
      }
      Serial.println("Buffer cleared");
      Serial.flush();
    //interrupts();
      return 0;  
  }
}

void work(){
  if (r_head == NULL){
    return 0;
  }
  Receive_Packet *r_tail;
  r_tail = r_head;
  while (r_tail->next != NULL) {
    r_tail = r_tail->next;
  }      
  Transmit_Packet *t_packet = (Transmit_Packet *) malloc(sizeof(Transmit_Packet));
  t_packet->packet_id = current_t_packet_id;
  t_packet->signaltype = r_head->signaltype;
  t_packet->unique_id = r_head->unique_id;
  for (int i=0; i<4; i++) {
    t_packet->data[i] = r_head->data[i];
  }
  switch (t_packet->signaltype) {
    case 10:
      switch(r_head->datatype) {
        case 0:
          Stepping.setDegree(t_packet->data[0], t_packet->data[1]);
          Stepping.work(t_packet->signaltype, r_head->rot_dir);
          break;
        case 1:
          Stepping.setTime(t_packet->data[0], t_packet->data[1]);
          Stepping.work(t_packet->signaltype, r_head->rot_dir);
          break;
        case 2:
          Stepping.rot_p1(int(r_head->rot_dir));
          break;
        case 3:
          Stepping.stop_p1();
        default:
          break;
      }
      break;
    case 20:
      switch(r_head->datatype) {
        case 0:
          Stepping.setDegree(t_packet->data[0], t_packet->data[1]);
          Stepping.work(t_packet->signaltype, r_head->rot_dir);
          break;
        case 1:
          Stepping.setTime(t_packet->data[0], t_packet->data[1]);
          Stepping.work(t_packet->signaltype, r_head->rot_dir);
          break;
        case 2:
          Stepping.rot_p2(int(r_head->rot_dir));
          break;
        case 3:
          Stepping.stop_p2();
        default:
          break;
      }
      break;
    case 30:
      switch(r_head->datatype) {
        case 0:
          Stepping.setDegrees(t_packet->data[0], t_packet->data[1], t_packet->data[2], t_packet->data[3]);
          Stepping.work(t_packet->signaltype, r_head->rot_dir);
          break;
        case 1:
          Stepping.setTimes(t_packet->data[0], t_packet->data[1], t_packet->data[2], t_packet->data[3]);
          Stepping.work(t_packet->signaltype, r_head->rot_dir);
          break;
        case 2:
          Stepping.rot_p_both(r_head->rot_dir);
          break;
        case 3:
          Stepping.stop_p_both();
        default:
          break;
      }
      break;   
    case 40:
      //IMU.readSensor();
      t_packet->data[0] = get_dist(t_packet->data[0]);
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
      // IMU.readSensor();
      // t_packet.data[0] = IMU.getAccelX_mss();
      // t_packet.data[1] = IMU.getAccelY_mss();
      // t_packet.data[2] = IMU.getAccelZ_mss();
      // t_packet.data[3] = IMU.getGyroX_rads();
      // t_packet.data[4] = IMU.getGyroY_rads();
      // t_packet.data[5] = IMU.getGyroZ_rads();
      // t_packet.data[6] = IMU.getMagX_uT();
      // t_packet.data[7] = IMU.getMagY_uT();
      // t_packet.data[8] = IMU.getMagZ_uT();
      break;
    default:
      for (int i=0; i<2; i++) {
        t_packet->data[i] = r_head->data[i];
      }
      break;
  }
  Receive_Packet *buf = r_head;
  r_head = r_head->next;
  free(buf);
  serialTransmit(t_packet);
}

void serialTransmit(Transmit_Packet *t_packet){
  //Serial.print("signaltype : ");
  Serial.print(t_packet->signaltype);
  Serial.flush();
  //Serial.print("unique_id : ");
  Serial.print(t_packet->unique_id);
  Serial.flush();
    for (int i = 0; i < 9; i++) {
      //Serial.print("data : ");
      Serial.print(t_packet->data[i]);
      Serial.flush();
    }
  free(t_packet);
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

int byte2int(byte *data){
  int *ret = (int*)data;
  return *ret;
}

long byte2long(byte *data){
  long *ret = (long*)data;
  return *ret;
}

float byte2float(byte *data){
  float *ret = (float*)data;
  return *ret;
}
