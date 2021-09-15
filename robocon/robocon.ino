#include <Wire.h>
#include <MPU9250.h>
const int receive_data_size = 17;
const int transmit_data_size = 42;
#define right_alert 3
#define right_feedback 4
#define left_alert 5
#define left_feedback 6
#define line_tracer 7
#define bright_thresh 8
#define pulse_pin 9
#define temp_pin 10

bool enable = true;
byte receive_data[receive_data_size];
byte transmit_data[transmit_data_size];
float IMU_data[9] = {0,0,0,0,0,0,0,0,0};
MPU9250 IMU(Wire,0x68);
int status;

typedef struct {
  int signaltype;
  long rand_id;
  byte rot_dir;
  byte angle_time;
  float data[2];
} Receive_Packet;

typedef struct {
  int signaltype;
  long rand_id;
  float data[9];
} Transmit_Packet;

void serial_transmit(Transmit_Packet t_packet){
}

void setup() {
  // put your setup code here, to run once:-
  Wire.begin();
  Serial.begin(9600);
  while(!Serial) {}

  status = IMU.begin();
  if (status > 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }
  
  Serial.println("Transmission Start");
  pinMode(right_alert, INPUT);
  pinMode(right_feedback, INPUT);
  pinMode(left_alert, INPUT);
  pinMode(left_feedback, INPUT);
  pinMode(pulse_pin, INPUT);
  pinMode(temp_pin, INPUT);
}

void loop() {
  //attachInterrupt(0,sw, RISING);
  byte *add = serial_receive();
  
}

byte *serial_receive() {
  byte receive_data[receive_data_size];
  byte *ret = &receive_data[0];
  if (Serial.available() == receive_data_size) {
    Serial.println("Stop");
    noInterrupts();
      //static int data_size = Serial.available();      
      //Serial.println(data_size);
      for (int n = 0; n < receive_data_size; n++){
        receive_data[n] = Serial.read();
        Serial.print((char)receive_data[n]);
      }
      //Serial.println("");
    interrupts();
      //data_size -= max_data_size;
      for (int n = 0; n < 4; n++){
        Serial.print((char)receive_data[n+2]);
      }
      Serial.println("");
      return ret;  
  } else if (Serial.available() > 0){
    Serial.println("Invalid data size");
    noInterrupts();
      int trush = 0;
      int len = Serial.available();
      for (int n = 0; n < len; n++){
        trush = Serial.read();
        Serial.print((char)trush);
      }
      Serial.println("");
    interrupts();
      //return ret;
  } else {
    delay(100);
    serial_receive();
  }
}

byte *receive_to_transmit(byte *add) {
  byte *receive_data = add;
  byte transmit_data[transmit_data_size];
  byte *ret = &transmit_data[0];
  switch (receive_data[0]) {
    case 0:
      transmit_data[0] = 60;
      break;
    case 1:
      transmit_data[0] = 60;
      break;
    case 2:
      transmit_data[0] = (digitalRead(right_alert) == HIGH)? 0:10;
      break;
    case 3:
      transmit_data[0] = (digitalRead(left_alert) == HIGH)? 20:30;
      break;
    case 4:
      transmit_data[0] = 40;
      break;
    case 5:
      transmit_data[0] = 50;
      break;
    case 6:
      transmit_data[0] = 70;
      break;
    default:
      break;
  }
  for (int i=0; i<4; i++) {
    transmit_data[i+2] = receive_data[i+1];
  }
  return ret;
}

/*
Receive_Packet decode_receive(int *add) {
  Receive_Packet r_packet;
  int len_data = typeof(add)/typeof(add[0]);
  if (len_data == receice_data_size){
    r_packet.signaltype = add[0];
    r_packet.rand_id = add[1] * pow(2,24) + add[2] * pow(2,16) + add[3] * pow(2,8) + add[4];
    r_packet.lock_tire = (add[5] == 1)? true:false;
    r_packet.rot_dir = (add[6] == 1)? true:false;
    r_packet.angle_time = (add[7] == 1)? true:false;
    r_packet.data[0] = add[8];
    r_packet.data[1] = add[9];
    return r_packet;
  } else {
    return 0;
  }

Transmit_Packet decode_transmit(int *add) {
  Transmit_Packet t_packet;
  int len_data = typeof(add)/typeof(add[0]);
  if (len_data == transmit_data_size){
    t_packet.signaltype = add[0];
    t_packet.rand_id = add[1] * pow(2,24) + add[2] * pow(2,16) + add[3] * pow(2,8) + add[4];
    t_packet.data[0] = add[8];
    t_packet.data[1] = add[9];
    return t_packet;
  } else {
    return 0;
  }
}

void serial_transmit2(Transmit_Packet t_packet){
  for (int i = 0; i < 9; i++) {
    t_packet.data[i] = 0;
  }
  switch (t_packet.signaltype) {
    case 0:
      t_packet.data[8] = (digitalRead(right_alert) == HIGH)? (float)1:(float)0;
      break;
    case 10:
      t_packet.data[8] = (digitalRead(right_feedback) == HIGH)? (float)1:(float)0;
      break;
    case 20:
      t_packet.data[8] = (digitalRead(left_alert) == HIGH)? (float)1:(float)0;
      break;
    case 30:
      t_packet.data[8] = (digitalRead(left_feedback) == HIGH)? (float)1:(float)0;
      break;
    case 40:
      IMU.readSensor();
      t_packet.data[8] = get_dist(IMU.getTemperature_C());
      break;
    case 50:
      t_packet.data[8] = (analogRead(line_tracer) >= bright_thresh)? (float)1:(float)0;
      break;
    case 60:
      break;
    case 70:
      IMU.readSensor();
      t_packet.data[0] = IMU.getAccelX_mss();
      t_packet.data[1] = IMU.getAccelY_mss();
      t_packet.data[2] = IMU.getAccelZ_mss();
      t_packet.data[3] = IMU.getGyroX_rads();
      t_packet.data[4] = IMU.getGyroY_rads();
      t_packet.data[5] = IMU.getGyroZ_rads();
      t_packet.data[6] = IMU.getMagX_uT();
      t_packet.data[7] = IMU.getMagY_uT();
      t_packet.data[8] = IMU.getMagZ_uT();
      break;
    default:
      break;
  }
  Serial.print(t_packet.signaltype);
  Serial.print(t_packet.rand_id);
    for (int i = 0; i < 9; i++) {
      Serial.print(t_packet.data[i]);
}*/

void serial_transmit(byte *add) {
  byte datatype = add[0];
  long rand_id = add[1] * pow(2,24) + add[2] * pow(2,16) + add[3] * pow(2,8) + add[4];
  for (int i = 0; i < 9; i++) {
    add[i+5] = 0;
  }
  switch (datatype) {
    case 0:
      IMU_data[8] = (digitalRead(right_alert) == HIGH)? (float)1:(float)0;
      break;
    case 10:
      IMU_data[8] = (digitalRead(right_feedback) == HIGH)? (float)1:(float)0;
      break;
    case 20:
      IMU_data[8] = (digitalRead(left_alert) == HIGH)? (float)1:(float)0;
      break;
    case 30:
      IMU_data[8] = (digitalRead(left_feedback) == HIGH)? (float)1:(float)0;
      break;
    case 40:
      IMU.readSensor();
      IMU_data[8] = get_dist(IMU.getTemperature_C());
      break;
    case 50:
      IMU_data[8] = (analogRead(line_tracer) >= bright_thresh)? (float)1:(float)0;
      break;
    case 60:
      break;
    case 70:
      IMU.readSensor();
      IMU_data[0] = IMU.getAccelX_mss();
      IMU_data[1] = IMU.getAccelY_mss();
      IMU_data[2] = IMU.getAccelZ_mss();
      IMU_data[3] = IMU.getGyroX_rads();
      IMU_data[4] = IMU.getGyroY_rads();
      IMU_data[5] = IMU.getGyroZ_rads();
      IMU_data[6] = IMU.getMagX_uT();
      IMU_data[7] = IMU.getMagY_uT();
      IMU_data[8] = IMU.getMagZ_uT();
      break;
    default:
      break;
    Serial.print(datatype);
    Serial.print(rand_id);
    for (int i = 0; i < 9; i++) {
      Serial.print(IMU_data[i]);
    }
  }
}

/*
void i2c_write(int address, int reg_id, long data, byte stop_mode) {
  Wire.beginTransmission(address);
  Wire.write(reg_id);
  Wire.write(data);
  Wire.endTransmission(stop_mode);
}

void i2c_read(int address, int reg_id, long data)
  Wire.beginTransmission(address);
  Wire.write(reg_id);
  Wire.endTransmission();
  Wire.requestFrom(address, data);
  while(Wire.available()) {
    b1 = Wire.read();
  }
}
*/

int get_temp() {
  int analogIn = analogRead(temp_pin);
  int sensorVout = map(analogIn, 0, 1023, 0, 4600);
  int temperature = map(sensorVout, 350, 1450, -25, 80);
  return temperature;//℃
}

double get_dist(float temperature) {
  float velocity = 331.5 + 0.61 * (float)temperature;//m/s
  int pulse_time = pulseIn(pulse_pin, HIGH, 20000);//us
  float distance = velocity * 1000 * (pulse_time / 2) / 1000000; //mm 
  return distance;
}

/*
void servo() {
}

void get_9axis() {
}
*/
