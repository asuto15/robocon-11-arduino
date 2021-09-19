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
  Receive_Packet r_packet = serial_receive();
  Transmit_Packet t_packet = work(r_packet);

}

Receive_Packet serial_receive() {
  Receive_Packet r_packet;
  byte receive_data[receive_data_size];
  if (Serial.available() == receive_data_size) {
    Serial.println("Stop");
    noInterrupts();
      for (int n = 0; n < receive_data_size; n++){
        receive_data[n] = Serial.read();
      }
    interrupts();
      r_packet.signaltype = receive_data[0];
      r_packet.rand_id = receive_data[1] * pow(2,24) + receive_data[2] * pow(2,16) + receive_data[3] * pow(2,8) + receive_data[4];
      r_packet.data[0] = receive_data[5];
      r_packet.data[1] = receive_data[6];
      Serial.println(r_packet.rand_id);
      return r_packet;  
  } else if (Serial.available() > 0){
    Serial.println("Invalid data size");
    noInterrupts();
      int trush = 0;
      int len = Serial.available();
      for (int n = 0; n < len; n++){
        trush = Serial.read();
        //Serial.print(trush);
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

Transmit_Packet work(Receive_Packet r_packet){
  Transmit_Packet t_packet;
  t_packet.signaltype = r_packet.signaltype;
  t_packet.rand_id = r_packet.rand_id;
  for (int i=0; i<2; i++) {
    t_packet.data[i] = r_packet.data[i];
  }
  switch (r_packet.signaltype) {
    case 10:
      t_packet.data[0] = (digitalRead(right_alert) == HIGH)? 10:11;
      step();
      break;
    case 20:
      t_packet.data[0] = (digitalRead(left_alert) == HIGH)? 20:21;
      step();
      break;
    case 30:
      t_packet.data[0] = (digitalRead(right_alert) == HIGH | digitalRead(left_alert) == HIGH)? 30:31;
      step();
      break;
    case 40:
      t_packet.data[0] = 40;
      step();
      break;
    case 50:
      t_packet.data[0] = 50;
      IMU.readSensor();
      break;
    case 60:
      t_packet.data[0] = 60;
      servo();
      break;
    case 70:
      t_packet.data[0] = 70;
      servo();
      break;
    case 80:
      t_packet.data[0] = 80;
      IMU.readSensor();
      break;
    default:
      t_packet.signaltype = -1;
      break;
  }
  return t_packet;
}

void servo(int mode,int signal_type, float *data){
  float angular_velocity = data[0];//deg/ms
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


void serial_transmit(Transmit_Packet t_packet){
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
