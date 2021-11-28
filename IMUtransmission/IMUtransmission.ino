#define M5STACK_MPU6886
#define M5STACK_MYBUILD
#include <M5Stack.h>
#include "struct_and_union.h"

float accX = 0.0F;  // Define variables for storing inertial sensor data
float accY = 0.0F;
float accZ = 0.0F;
float gyroX = 0.0F;
float gyroY = 0.0F;
float gyroZ = 0.0F;
float pitch = 0.0F;
float roll  = 0.0F;
float yaw   = 0.0F;
float temp = 0.0F;
const int debug = 0;
const int receive_data_size = 24;
const int transmit_data_size = 42;
int current_unique_id = 0;

void setup(){
  M5.begin(); //Init M5Core.
  M5.Power.begin(); //Init Power module.
  M5.IMU.Init();  //Init IMU sensor.
//  M5.Lcd.fillScreen(BLACK); //Set the screen background color to black.
//  M5.Lcd.setTextColor(GREEN , BLACK); //Sets the foreground color and background color of the displayed text.
//  M5.Lcd.setTextSize(2);  //Set the font size.
  Serialsetup();
  xTaskCreatePinnedToCore(task0, "Task0", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(task1, "Task1", 4096, NULL, 1, NULL, 1);  
}

void loop() {
  
}

void task0(void* arg){
  long now = micros();
  while(1){
    if(micros()-now >= 10000){
      receivePacket();
    }
    //LCDprint();
  }
}

void task1(void* arg){
  long now = micros();
  while(1){
    if(micros()-now >= 10000){
      work();
    }
    //LCDprint();
  }
}

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

void IMUgetdata(){
  M5.IMU.getGyroData(&gyroX,&gyroY,&gyroZ);
  M5.IMU.getAccelData(&accX,&accY,&accZ); //Stores the triaxial accelerometer.
  M5.IMU.getAhrsData(&pitch,&roll,&yaw);  //Stores the inertial sensor attitude.
  M5.IMU.getTempData(&temp);  //Stores the inertial sensor temperature to temp.
}

void LCDprint(){
  M5.Lcd.setCursor(0, 20);  //Move the cursor position to (x,y).
  M5.Lcd.printf("gyroX,  gyroY, gyroZ"); //Screen printingformatted string.
  M5.Lcd.setCursor(0, 42);
  M5.Lcd.printf("%6.2f %6.2f%6.2f o/s", gyroX, gyroY, gyroZ);
  M5.Lcd.setCursor(0, 120);
  M5.Lcd.printf("pitch,  roll,  yaw");
  M5.Lcd.setCursor(0, 142);
  M5.Lcd.printf("%5.2f  %5.2f  %5.2f deg", pitch, roll, yaw);
  M5.Lcd.setCursor(0, 175);
  M5.Lcd.printf("Temperature : %.2f C", temp);
}

Transmit_Packet *t_head;
Transmit_Packet *t_packet;
Transmit_Packet *t_tail;
Receive_Packet *r_head;
Receive_Packet *r_packet;
Receive_Packet *r_current;
Receive_Packet *r_tail;
Receive_Packet *buf;

void receivePacket(){
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
  if (r_head == NULL) {
    return;
  } else {
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
    t_packet->signaltype = r_head->signaltype;
    t_packet->unique_id = r_head->unique_id;
    int signaltype = r_head->signaltype;
    int datatype = r_head->datatype;
    int rot_dir = r_head->rot_dir;
    float bright_thresh;
    float accX, accY, accZ;
    float gyroX, gyroY, gyroZ;
    float pitch, roll, yaw;
    float Temp;
    float data[4];

    for (int i=0; i<4; i++) {
      t_packet->data[i] = r_head->data[i];
      data[i] = r_head->data[i];
    }
    for (int i=4; i<9; i++) {
      t_packet->data[i] = 0;
    }
    Receive_Packet *buf = r_head->next;
    free(r_head);
    r_head = buf;
    
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
      /*case 10:
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
        break;*/
      case 80:
        float t1 = 0.721;
        float t2 = 11.4514;
        float t3 = 45.45;
        M5.IMU.getGyroData(&gyroX,&gyroY,&gyroZ);
        M5.IMU.getAccelData(&accX,&accY,&accZ); //Stores the triaxial accelerometer.
        M5.IMU.getAhrsData(&pitch,&roll,&yaw);  //Stores the inertial sensor attitude.
        M5.IMU.getTempData(&temp);  //Stores the inertial sensor temperature to temp.
        t_packet->data[0] = accX;
        t_packet->data[1] = accY;
        t_packet->data[2] = accZ;
        t_packet->data[3] = gyroX;
        t_packet->data[4] = gyroY;
        t_packet->data[5] = gyroZ;
        t_packet->data[6] = t1;
        t_packet->data[7] = t2;
        t_packet->data[8] = t3;
        break;
    }
    if (debug) {
      Serial.print(">we : ");
      Serial.flush();
      Serial.println(micros());
      Serial.flush();
    }
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
    Serial.println("");
    Serial.flush();
    free(t_packet);  
  }
}
