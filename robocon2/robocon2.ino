#include <Wire.h>
#include <EEPROM.h>
#include <MPU9250.h>
const int receive_data_size = 25;
const int transmit_data_size = 43;
#define right_alert 3
#define right_feedback 4
#define left_alert 5
#define left_feedback 6
#define line_tracer 7
#define pulse_pin 9
#define temp_pin 10
#define right_cw
#define right_ccw
#define left_cw
#define left_ccw
#define sv_upper
#define sv_lower
#define rot_angle
#define check
#define check1

byte receive_data[receive_data_size];
byte transmit_data[transmit_data_size];
float IMU_data[9] = {0,0,0,0,0,0,0,0,0};
//MPU9250 IMU(Wire,0x68);
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

enum dir {
  right,
  left,
  lock
};

enum move {
  right,
  left,
  both
};

Transmit_Packet receive_to_transmit2(Receive_Packet r_packet) {
}

Transmit_Packet work(Transmit_Packet t_packet1){
}

void serial_transmit2(Transmit_Packet t_packet){
}

void setup() {
  // put your setup code here, to run once:-
  pinMode(check,INPUT)
  pinMode(check1,INPUT)

  if(digitalRead(check) == HIGH) {
    LTsetup(check1);
  }

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
  pinMode(right_cw, OUTPUT);
  pinMode(right_ccw, OUTPUT);
  pinMode(left_cw, OUTPUT);
  pinMode(left_ccw, OUTPUT);
  pinMode(sv_upper, OUTPUT);
  pinMode(sv_lower, OUTPUT);
  
}

void LTsetup(int check1) {
  Serial.println("Line tracer Setup Start");
  Serial.println("pls connect check1 pin and VCC to get the brightness on the line")
  while(digitalRead(check1) == LOW){
    delay(1000);
  }
  int onLine = analogRead(line_tracer);
  Serial.println("OK. pls connect check1 pin and GND to get the brightness on the load")
  while(digitalRead(check1)) == HIGH) {
    delay(1000);
  }
  int onLoad = analogRead(line_tracer);
  int bright_thresh = (onLine + onLoad) /2;
  EEPROM.write(0,bright_thresh);
  Serial.println("Setup Complete");
  Serial.print("Brightness threshold : ");
  Serial.println(bright_thresh);
}

void loop() {
  //attachInterrupt(0,sw, RISING);
  Receive_Packet r_packet = serial_receive2();
  Transmit_Packet t_packet = receive_to_transmit(add);

}

Receive_Packet serial_receive2() {
  Receive_Packet r_packet;
  byte receive_data[receive_data_size];
  if (Serial.available() == receive_data_size) {
    Serial.println("NO!");
    noInterrupts();
      for (int n = 0; n < receive_data_size; n++){
        receive_data[n] = Serial.read();
      }
    interrupts();
      Serial.println("OK!");
      r_packet.signaltype = receive_data[0];
      r_packet.rand_id = receive_data[1] * pow(2,24) + receive_data[2] * pow(2,16) + receive_data[3] * pow(2,8) + receive_data[4];
      r_packet.data[0] = receive_data[5];
      r_packet.data[1] = receive_data[6];
      return r_packet;  
  } else if (Serial.available() > 0){
    Serial.println("Invalid data size");
    noInterrupts();
      int trush = 0;
      int len = Serial.available();
      for (int n = 0; n < len; n++){
        trush = Serial.read();
        Serial.print(trush);
      }
      Serial.println("");
    interrupts();
      r_packet.signaltype = -1;
      return r;
  } else {
    delay(100);
    serial_receive();
  }
}

Transmit_Packet receive_to_transmit2(Receive_Packet r_packet) {
  Transmit_Packet t_packet;
  switch (r_packet.signaltype) {
    case 10:
      t_packet.data[0] = (digitalRead(right_alert) == HIGH)? 10:11;
      break;
    case 20:
      t_packet.data[0] = (digitalRead(left_alert) == HIGH)? 20:21;
      break;
    case 30:
      t_packet.data[0] = (digitalRead(right_alert) == HIGH | digitalRead(left_alert) == HIGH)? 30:31;
      break;
    case 40:
      t_packet.data[0] = 40;
      break;
    case 50:
      t_packet.data[0] = 50;
      break;
    case 60:
      t_packet.data[0] = 60;
      break;
    case 70:
      t_packet.data[0] = 70;
      break;
    case 80:
      t_packet.data[0] = 80;
      break;
    default:
      break;
  }
  for (int i=0; i<2; i++) {
    t_packet.data[i] = r_packetdata[i];
  }
  return t_packet;
}

Transmit_Packet work(Transmit_Packet t_packet1){
  Transmit_Packet t_packet2;
  t_packet2.signaltype = t_packet1.signaltype;
  t_packet2.rand_id = t_packet1.rand_id;
  switch (t_packet1.signaltype) {
    case 10:
      step();
      break;
    case 20:
      step();
      break;
    case 30:
      step();
      break;
    case 40:
      step();
      break;
    case 50:
      IMU.readSensor();
      break;
    case 60:
      servo();
      break;
    case 70:
      servo();
      break;
    case 80:
      IMU.readSensor();
      break;
    default:
      break;
    }
}

void servo(int mode,float *data){
  switch (mode) {
    case 0:
      digitalWrite(sv_upper, HIGH);
      delay(1000);
      digitalWrite(sv_upper, LOW);
    case 1:
      digitalWrite(sv_lower, HIGH);
      delay(1000);
      digitalWrite(sv_lower, LOW);
    case 2:
      digitalWrite(sv_lower, HIGH);
      digitalWrite(sv_upper, HIGH);
      delay(1000);
      digitalWrite(sv_lower, LOW);
      digitalWrite(sv_upper, LOW);
  }
}

void step(){

}


void serial_transmit2(Transmit_Packet t_packet){
  for (int i = 0; i < 9; i++) {
    t_packet.data[i] = 0;
  }
  switch (t_packet.signaltype) {
    case 10:
      t_packet.data[0] = (digitalRead(right_alert) == HIGH)? (float)1:(float)0;
      break;
    case 20:
      t_packet.data[0] = (digitalRead(left_alert) == HIGH)? (float)1:(float)0;
      break;
    case 30:
      t_packet.data[0] = (digitalRead(left_alert) == HIGH)? (float)1:(float)0;
      break;
    case 40:
      IMU.readSensor();
      t_packet.data[0] = get_dist(IMU.getTemperature_C());
      break;
    case 50:
      break;
    case 60:
      bright_thresh = EEPROM.read(0);
      t_packet.data[0] = (analogRead(line_tracer) <= bright_thresh)? (float)1:(float)0;
      break;
    case 70:
      break;
    case 80:
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
}

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
byte *serial_receive() {
  byte receive_data[receive_data_size];
  byte *ret = &receive_data[0];
  if (Serial.available() == receive_data_size) {
    Serial.println("NO!");
    noInterrupts();
      //static int data_size = Serial.available();      
      //Serial.println(data_size);
      for (int n = 0; n < receive_data_size; n++){
        receive_data[n] = Serial.read();
        //Serial.print((char)receive_data[n]);
      }
      //Serial.println("");
    interrupts();
      //data_size -= max_data_size;
      Serial.println("OK!");
      return ret;  
  } else if (Serial.available() > 0){
    Serial.println("Invalid data size");
    noInterrupts();
      int trush = 0;
      int len = Serial.available();
      for (int n = 0; n < len; n++){
        trush = Serial.read();
        Serial.print(trush);
      }
      Serial.println("");
    interrupts();
      //return ret;
  } else {
    delay(100);
    serial_receive();
  }
}
*/

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
*/

/*
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
*/

/*
void serial_transmit(byte *add) {
  byte datatype = add[0];
  long rand_id = add[1] * pow(2,24) + add[2] * pow(2,16) + add[3] * pow(2,8) + add[4];
  for (int i = 0; i < 9; i++) {
    add[i+5] = 0;
  }
  work(byte *add, byte datatype, float *IMU_data);
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
*/


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

/*
void servo() {
}

void get_9axis() {
}
*/
