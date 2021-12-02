#define M5STACK_MPU6886
#define M5STACK_MYBUILD
#include <Arduino.h>
#include <EEPROM.h>
#include <M5Stack.h>
#include <M5StackUpdater.h>  // version 0.5.2
#include <utility/quaternionFilters.h>

#include "BMM150class.h"
#include "struct_and_union.h"
// M5Stack.hより後ろにLovyanGFXを書く
#include <LovyanGFX.hpp>

#define MAHONY
//#define MADGWICK

static LGFX lcd;
static LGFX_Sprite compass(&lcd);  // オフスクリーン描画用バッファ
static LGFX_Sprite base(&compass);  // オフスクリーン描画用バッファ

float accX = 0.0F;
float accY = 0.0F;
float accZ = 0.0F;
float gyroX = 0.0;
float gyroY = 0.0F;
float gyroZ = 0.0F;
int AVERAGENUM_INIT = 256;
float init_gyroX = 0.0F;
float init_gyroY = 0.0F;
float init_gyroZ = 0.0F;
float magnetX = 0.0F;
float magnetY = 0.0F;
float magnetZ = 0.0F;
float magnetaX = 0.0F;
float magnetaY = 0.0F;
float magnetaZ = 0.0F;
// for hard iron correction
float magoffsetX = 0.0F;
float magoffsetY = 0.0F;
float magoffsetZ = 0.0F;
// for soft iron correction
float magscaleX = 0.0F;
float magscaleY = 0.0F;
float magscaleZ = 0.0F;
float pitch = 0.0F;
float roll = 0.0F;
float yaw = 0.0F;
BMM150class bmm150;
uint32_t Now_w = 0;
uint32_t lastUpdate_w = 0;
float deltat_w = 0.0f;
uint32_t Now = 0;
uint32_t lastUpdate = 0;
float deltat = 0.0f;
float p_head_dir = 0, p_yaw = 0, p_deltat;

struct bmm150_dev dev;
bmm150_mag_data mag_offset;  // Compensation magnetometer float data storage
bmm150_mag_data mag_max;
bmm150_mag_data mag_min;
int mag_compensate_x = 455;
int mag_compensate_y = 760;
int mag_compensate_z = 470;
float temp = 0.0F;
const int debug = 0;
const int mag_pin = 16;
const int echo_pin = 17;
const int line_tracer = 2;
const int lt_pin = 5;
const int receive_data_size = 24;
const int transmit_data_size = 42;
int current_signaltype = 0;
int current_unique_id = 0;
int bright_thresh = EEPROM.read(0);

void setup() {
  enableCore1WDT();
  Serial.begin(115200);
  M5.begin(true, false, true,
           false);             // Init M5Core(Initialize LCD, serial port).
  M5.Power.begin();            // Init Power module.
  M5.IMU.Init();               // Init IMU sensor.
  Wire.begin(21, 22, 400000);  // Set the frequency of the SDA SCL.
  Serialsetup();
  lcd.init();
  // 回転方向を 0～3
  // の4方向から設定します。(4～7を使用すると上下反転になります。)
  lcd.setRotation(1);
  initGyro();
  Serial.println("1");
  bmm150.Init();
  Serial.println("2");
  bmm150.getMagnetOffset(&magoffsetX, &magoffsetY, &magoffsetZ);
  Serial.println("3");
  bmm150.getMagnetScale(&magscaleX, &magscaleY, &magscaleZ);
  Serial.println("4");
  // 16の方がSPI通信量が少なく高速に動作しますが、赤と青の諧調が5bitになります。
  lcd.setColorDepth(16);
  Serial.println("5");
  // clearまたはfillScreenで画面全体を塗り潰します。
  lcd.clear();  // 黒で塗り潰し
                // スプライト（オフスクリーン）への描画も同様の描画関数が使えます。
                // 最初にスプライトの色深度をsetColorDepthで指定します。（省略した場合は16として扱われます。）
  Serial.println("6");
  compass.setColorDepth(16);
  Serial.println("7");
  base.setColorDepth(16);
  // createSpriteで幅と高さを指定してメモリを確保します。
  // 消費するメモリは色深度と面積に比例します。大きすぎるとメモリ確保に失敗しますので注意してください。
  Serial.println("8");
  compass.createSprite(180, 180);
  Serial.println("9");
  base.createSprite(120, 120);

  // base airplane
  Serial.println("10");
  base.drawLine(52, 36, 52, 14, TFT_WHITE);   // front L
  base.drawLine(2, 62, 52, 36, TFT_WHITE);    // wing L
  base.drawLine(4, 76, 2, 62, TFT_WHITE);     // wing L
  base.drawLine(52, 64, 4, 76, TFT_WHITE);    // wing L
  base.drawLine(52, 82, 52, 64, TFT_WHITE);   // body L
  base.drawLine(36, 104, 54, 96, TFT_WHITE);  //
  base.drawLine(36, 114, 36, 104, TFT_WHITE);
  base.drawLine(58, 110, 36, 114, TFT_WHITE);
  base.drawLine(60, 116, 58, 110, TFT_WHITE);
  base.drawLine(62, 110, 60, 116, TFT_WHITE);
  base.drawLine(84, 114, 62, 110, TFT_WHITE);
  base.drawLine(84, 104, 84, 114, TFT_WHITE);
  base.drawLine(66, 96, 84, 104, TFT_WHITE);
  base.drawLine(68, 64, 68, 82, TFT_WHITE);    // body R
  base.drawLine(116, 76, 68, 64, TFT_WHITE);   // wing R
  base.drawLine(118, 62, 116, 76, TFT_WHITE);  // wing R
  base.drawLine(68, 36, 118, 62, TFT_WHITE);   // wing R
  base.drawLine(68, 14, 68, 36, TFT_WHITE);
  base.drawLine(54, 96, 52, 82, TFT_WHITE);
  base.drawLine(66, 96, 68, 82, TFT_WHITE);
  base.drawLine(52, 14, 54, 4, TFT_WHITE);
  base.drawLine(68, 14, 66, 4, TFT_WHITE);
  base.drawLine(66, 4, 60, 0, TFT_WHITE);  // head
  base.drawLine(54, 4, 60, 0, TFT_WHITE);  // head

  // 作成したスプライトはpushSpriteで任意の座標に出力できます。
  //  base.pushSprite(100,60); // (x,y)=((320-120)/2,(240-120)/2) lcdに対して
  Serial.println("11");
  xTaskCreatePinnedToCore(task0, "Task0", 4096, NULL, 1, NULL, 0);
  Serial.println("12");
}

void mag_calibrate() {
  pinMode(mag_pin, INPUT);
  if (digitalRead(mag_pin) == LOW) {
    Serial.println(">Calibration Start");
    bmm150.bmm150_calibrate(10000);
    Serial.println(">Calibration finished");
  }
}

void initGyro() {
  
  lcd.clear();        // 黒で塗り潰し
  lcd.setCursor(0, 0);
  lcd.print("begin gyro calibration");

  for (int i = 0;i < AVERAGENUM_INIT;i++) {
    M5.IMU.getGyroData(&gyroX,&gyroY,&gyroZ);
    init_gyroX += gyroX;
    init_gyroY += gyroY;
    init_gyroZ += gyroZ;
    delay(5);
  }
  init_gyroX /= AVERAGENUM_INIT;
  init_gyroY /= AVERAGENUM_INIT;
  init_gyroZ /= AVERAGENUM_INIT;
}

void compassplot(float a) {
  int ang;
  a = 360.0 - a;
  compass.setTextDatum(middle_center);
  compass.setFont(&fonts::Font2);
  compass.fillScreen(0);  // fill black
  for (ang = 0; ang < 36; ang++) {
    compass.drawLine(90 + 80 * sin((a + ang * 10) / 180.0 * PI),
                     90 - 80 * cos((a + ang * 10) / 180.0 * PI),
                     90 + 90 * sin((a + ang * 10) / 180.0 * PI),
                     90 - 90 * cos((a + ang * 10) / 180.0 * PI),
                     TFT_WHITE);  // 0
    compass.setTextSize(1.5);
    if (ang == 0) {
      compass.drawString("N", 90 + 72 * sin((a + 0) / 180.0 * PI),
                         90 - 72 * cos((a + 0) / 180.0 * PI));  // 0
    } else if (ang == 9) {
      compass.drawString("E", 90 + 72 * sin((a + 90) / 180.0 * PI),
                         90 - 72 * cos((a + 90) / 180.0 * PI));  // 90
    } else if (ang == 18) {
      compass.drawString("S", 90 + 72 * sin((a + 180) / 180.0 * PI),
                         90 - 72 * cos((a + 180) / 180.0 * PI));  // 180
    } else if (ang == 27) {
      compass.drawString("W", 90 + 72 * sin((a + 270) / 180.0 * PI),
                         90 - 72 * cos((a + 270) / 180.0 * PI));  // 270
    } else if ((ang % 3) == 0) {
      compass.setTextSize(1);
      compass.drawNumber(ang, 90 + 72 * sin((a + ang * 10) / 180.0 * PI),
                         90 - 72 * cos((a + ang * 10) / 180.0 * PI));
    }
  }
  compass.fillTriangle(224 - 70, 56 - 30, 228 - 70, 47 - 30, 233 - 70, 52 - 30,
                       TFT_WHITE);  // 45
  compass.fillTriangle(224 - 70, 184 - 30, 233 - 70, 188 - 30, 228 - 70,
                       193 - 30, TFT_WHITE);  // 135
  compass.fillTriangle(96 - 70, 184 - 30, 92 - 70, 193 - 30, 87 - 70, 188 - 30,
                       TFT_WHITE);  // 225
  compass.fillTriangle(96 - 70, 56 - 30, 87 - 70, 52 - 30, 92 - 70, 47 - 30,
                       TFT_WHITE);  // 315

  // 作成したスプライトはpushSpriteで任意の座標に出力できます。
  base.pushSprite(
      30, 30,
      0);  // (x,y)=((180-120)/2,(180-120)/2)  compassに出力する。透過色あり
  compass.pushSprite(
      70, 30);  // (x,y)=((320-180)/2,(240-180)/2) lcdに出力する。透過色なし
}

void loop() {
  // put your main code here, to run repeatedly:
  M5.update();
  M5.IMU.getGyroData(&gyroX, &gyroY, &gyroZ);
  M5.IMU.getAccelData(&accX, &accY, &accZ);
  bmm150.getMagnetData(&magnetX, &magnetY, &magnetZ);

  gyroX -= init_gyroX;
  gyroY -= init_gyroY;
  gyroZ -= init_gyroZ;
  magnetX = (magnetX - magoffsetX) * magscaleX;
  magnetY = (magnetY - magoffsetY) * magscaleY;
  magnetZ = (magnetZ - magoffsetZ) * magscaleZ;

  float head_dir = atan2(magnetX, magnetY);
  if (head_dir < 0) head_dir += 2 * PI;
  if (head_dir > 2 * PI) head_dir -= 2 * PI;

  head_dir *= RAD_TO_DEG;

  Now = micros();
  deltat = ((Now - lastUpdate) / 1000000.0f);  // 0.09
  lastUpdate = Now;

#ifdef MADGWICK
  MadgwickQuaternionUpdate(accX, accY, accZ, gyroX * DEG_TO_RAD,
                           gyroY * DEG_TO_RAD, gyroZ * DEG_TO_RAD, -magnetX,
                           magnetY, -magnetZ, deltat);
#endif

#ifdef MAHONY
  MahonyQuaternionUpdate(accX, accY, accZ, gyroX * DEG_TO_RAD,
                         gyroY * DEG_TO_RAD, gyroZ * DEG_TO_RAD, -magnetX,
                         magnetY, -magnetZ, deltat);
  // delay(10); // adjust sampleFreq = 50Hz
#endif

  yaw =
      atan2(2.0f * (*(getQ() + 1) * *(getQ() + 2) + *getQ() * *(getQ() + 3)),
            *getQ() * *getQ() + *(getQ() + 1) * *(getQ() + 1) -
                *(getQ() + 2) * *(getQ() + 2) - *(getQ() + 3) * *(getQ() + 3));
  pitch =
      -asin(2.0f * (*(getQ() + 1) * *(getQ() + 3) - *getQ() * *(getQ() + 2)));
  roll =
      atan2(2.0f * (*getQ() * *(getQ() + 1) + *(getQ() + 2) * *(getQ() + 3)),
            *getQ() * *getQ() - *(getQ() + 1) * *(getQ() + 1) -
                *(getQ() + 2) * *(getQ() + 2) + *(getQ() + 3) * *(getQ() + 3));
  yaw = -0.5 * M_PI - yaw;
  if (yaw < 0) yaw += 2 * M_PI;
  if (yaw > 2 * M_PI) yaw -= 2 * M_PI;
  pitch *= RAD_TO_DEG;
  yaw *= RAD_TO_DEG;
  roll *= RAD_TO_DEG;

  delay(1);

#ifdef MAG
  Serial.printf(" %5.2f,  %5.2f,  %5.2f  \r\n", magnetX, magnetY, magnetZ);
#endif
#ifdef ANGLE
  Serial.printf(" %5.2f  \r\n", head_dir);
#endif

  compassplot(yaw);
  // for processing display
  Serial.printf(" %5.2f,  %5.2f,  %5.2f  \r\n", roll, pitch,
                yaw);  // to processing

  lcd.fillTriangle(160, 29, 157, 20, 163, 20, TFT_WHITE);     //  0
  lcd.fillTriangle(251, 120, 260, 117, 260, 123, TFT_WHITE);  // 90
  lcd.fillTriangle(160, 211, 163, 220, 157, 220, TFT_WHITE);  // 180
  lcd.fillTriangle(69, 120, 60, 123, 60, 117, TFT_WHITE);     // 270

  lcd.setTextColor(TFT_BLACK, TFT_BLACK);
  lcd.drawString(String(p_head_dir), 30, 50);
  lcd.drawString(String(p_yaw), 30, 80);
  Serial.printf("p_deltat : %f\n",p_deltat);
  lcd.drawString(String(1 / p_deltat), 270, 215);

  lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  lcd.drawString("MAG X-Y", 20, 35);
  lcd.drawString(String(head_dir), 30, 50);
  lcd.drawString("Heading", 20, 65);
  lcd.drawString(String(yaw), 30, 80);
  lcd.drawString("sampleFreq", 250, 200);
  Serial.printf("deltat : %f\n", deltat);
  lcd.drawString(String(1 / deltat), 270, 215);

  p_head_dir = head_dir;
  p_yaw = yaw;
  p_deltat = deltat;

  lcd.setCursor(40, 230);
  lcd.printf("BTN_A:CAL ");
}

void M5calibration() {
  if (M5.BtnA.wasPressed()) {
    lcd.clear();  // 黒で塗り潰し
    lcd.setCursor(0, 0);
    lcd.print("begin calibration in 3 seconds");
    delay(3000);
    lcd.setCursor(0, 10);
    lcd.print("Flip + rotate core calibration");
    bmm150.bmm150_calibrate(10000);
    delay(100);

    bmm150.Init();
    bmm150.getMagnetOffset(&magoffsetX, &magoffsetY, &magoffsetZ);
    bmm150.getMagnetScale(&magscaleX, &magscaleY, &magscaleZ);
    lcd.clear();  // 黒で塗り潰し
  }
}

void task0(void *arg) {
  long now = micros();
  while (1) {
    if (micros() - now >= 1000) {
      if (Serial.available() > 0) {
       receivePacket(); 
      }
      work();
      if (analogRead(line_tracer) >= bright_thresh) {
        lt_event();
      }
      now = micros();
    }
  }
}

void lt_event() {
  Transmit_Packet *t_packet;
  if ((t_packet = (Transmit_Packet *)malloc(sizeof(Transmit_Packet))) == NULL) {
    Serial.println(">malloc error with transmit_packet");
    Serial.flush();
    return;
  }
  M5.IMU.getGyroData(&gyroX, &gyroY, &gyroZ);
  M5.IMU.getAccelData(&accX, &accY, &accZ);  // Stores the triaxial accelerometer.
  M5.IMU.getAhrsData(&pitch, &roll, &yaw);  // Stores the inertial sensor attitude.
  M5.IMU.getTempData(&temp);
  bmm150_read_mag_data(&dev);

  bmm150.getMagnetData(&magnetX, &magnetY, &magnetZ);
  gyroX -= init_gyroX;
  gyroY -= init_gyroY;
  gyroZ -= init_gyroZ;
  magnetX = (magnetX - magoffsetX) * magscaleX;
  magnetY = (magnetY - magoffsetY) * magscaleY;
  magnetZ = (magnetZ - magoffsetZ) * magscaleZ;
  float head_dir = atan2(magnetX, magnetY);
  if (head_dir < 0) head_dir += 2 * PI;
  if (head_dir > 2 * PI) head_dir -= 2 * PI;
  head_dir *= RAD_TO_DEG;
  Now = micros();
  deltat = ((Now - lastUpdate) / 1000000.0f);  // 0.09
  lastUpdate = Now;
#ifdef MADGWICK
  MadgwickQuaternionUpdate(accX, accY, accZ, gyroX * DEG_TO_RAD,
                            gyroY * DEG_TO_RAD, gyroZ * DEG_TO_RAD,
                            -magnetX, magnetY, -magnetZ, deltat);
#endif
#ifdef MAHONY
  MahonyQuaternionUpdate(accX, accY, accZ, gyroX * DEG_TO_RAD,
                          gyroY * DEG_TO_RAD, gyroZ * DEG_TO_RAD, -magnetX,
                          magnetY, -magnetZ, deltat);
  // delay(10); // adjust sampleFreq = 50Hz
#endif
  yaw = atan2(
      2.0f * (*(getQ() + 1) * *(getQ() + 2) + *getQ() * *(getQ() + 3)),
      *getQ() * *getQ() + *(getQ() + 1) * *(getQ() + 1) -
          *(getQ() + 2) * *(getQ() + 2) - *(getQ() + 3) * *(getQ() + 3));
  pitch = -asin(
      2.0f * (*(getQ() + 1) * *(getQ() + 3) - *getQ() * *(getQ() + 2)));
  roll = atan2(
      2.0f * (*getQ() * *(getQ() + 1) + *(getQ() + 2) * *(getQ() + 3)),
      *getQ() * *getQ() - *(getQ() + 1) * *(getQ() + 1) -
          *(getQ() + 2) * *(getQ() + 2) + *(getQ() + 3) * *(getQ() + 3));
  yaw = -0.5 * M_PI - yaw;
  if (yaw < 0) yaw += 2 * M_PI;
  if (yaw > 2 * M_PI) yaw -= 2 * M_PI;
  pitch *= RAD_TO_DEG;
  yaw *= RAD_TO_DEG;
  roll *= RAD_TO_DEG;
  
  t_packet->data[0] = accX;
  t_packet->data[1] = accY;
  t_packet->data[2] = accZ;
  t_packet->data[3] = gyroX;
  t_packet->data[4] = gyroY;
  t_packet->data[5] = gyroZ;
  t_packet->data[6] = yaw;
  t_packet->data[7] = temp;
  t_packet->data[8] = 1.0;

  Serial.printf(">#{\"SignalType\" : %d, \"UniqueID\" : %d, \"AccelX\" : %.2f, \"AccelY\" : %.2f, \"AccelZ\" : %.2f, \
    \"GyroX\" : %.2f, \"GyroY\" : %.2f, \"GyroZ\" : %.2f, \"Direction\" : %.2f, \"Temperature\" : %.2f, \"LineTracer\" : %d}\n",\
    (int)90, (int)current_unique_id+1, t_packet->data[0],\
    t_packet->data[1], t_packet->data[2], t_packet->data[3],\
    t_packet->data[4], t_packet->data[5], t_packet->data[6],\
    t_packet->data[7], (int)t_packet->data[8]);
  free(t_packet);
}

void Serialsetup() {
#if defined(ARDUINO_MYBUILD)
  Serial.begin(9600);
#endif
#if defined(M5STACK_MYBUILD)
  Serial.begin(115200);
#endif
  while (!Serial) {
  }
  Serial.println("Transmission Start");
  Serial.flush();
  byte trush;
  while (Serial.available() > 0) {
    trush = Serial.read();
  }
}

void IMUgetdata() {
  M5.IMU.getGyroData(&gyroX, &gyroY, &gyroZ);
  M5.IMU.getAccelData(&accX, &accY,
                      &accZ);  // Stores the triaxial accelerometer.
  M5.IMU.getAhrsData(&pitch, &roll,
                     &yaw);   // Stores the inertial sensor attitude.
  M5.IMU.getTempData(&temp);  // Stores the inertial sensor temperature to temp.
}

void LTsetup() {
  pinMode(lt_pin, INPUT);
  pinMode(line_tracer, INPUT);
  if (digitalRead(lt_pin) == HIGH) {
    Serial.println(">Line tracer Setup Start");
    Serial.flush();
    Serial.println(
        ">pls connect check pin and GND to get the brightness on the line");
    Serial.flush();
    while (digitalRead(lt_pin) == HIGH) {
      delay(100);
    }
    delay(1000);
    int onLine = analogRead(line_tracer);
    Serial.println(
        ">OK. pls connect check pin and VCC to get the brightness on the load");
    Serial.flush();
    while (digitalRead(lt_pin) == LOW) {
      delay(100);
    }
    delay(1000);
    int onLoad = analogRead(line_tracer);
    bright_thresh = (onLine + onLoad) / 2;
    EEPROM.write(0, bright_thresh);
    Serial.println(">Setup Complete");
    Serial.flush();
    Serial.print(">Brightness threshold : ");
    Serial.flush();
    Serial.println(bright_thresh);
    Serial.flush();
  }
}

Transmit_Packet *t_head;
Transmit_Packet *t_packet;
Transmit_Packet *t_tail;
Receive_Packet *r_head;
Receive_Packet *r_packet;
Receive_Packet *r_current;
Receive_Packet *r_tail;
Receive_Packet *buf;

void receivePacket() {
  if (Serial.available() > 0) {
    Receive_Packet *r_packet;
    if ((r_packet = (Receive_Packet *)malloc(sizeof(Receive_Packet))) == NULL) {
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
    // noInterrupts();
    byte received_data_size;
    if (debug == 2) {
      for (int i = 0; i < 24; i++) {
        receive_data[i] = 0;
      }
    } else {
      received_data_size =
          byte(Serial.readBytes(receive_data, receive_data_size));
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
      // noInterrupts();
      byte trush;
      while (Serial.available() > 0) {
        trush = Serial.read();
        if (debug > 1) {
          Serial.print(char(trush));
          Serial.flush();
        }
      }
      //      Serial.println(">Buffer cleared");
      //      Serial.flush();
      // interrupts();
      free(r_packet);
      return;
    }
    r_packet->signaltype = ((receive_data[0] << 8) + (receive_data[1]));
    r_packet->unique_id = ((receive_data[2] << 24) + (receive_data[3] << 16) +
                           (receive_data[4] << 8) + (receive_data[5]));
    r_packet->rot_dir = receive_data[6];
    r_packet->datatype = receive_data[7];
    for (int i = 0; i < 4; i++) {
      r_packet->data[i] = *(float *)(receive_data + 4 * i + 8);
    }
    if (r_packet->unique_id !=
        current_unique_id) {  // if (r_packet->unique_id != 0x00000000) {
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
  } else if (Serial.available() == 0) {
    // receivePacket();
    return;
  } else {
    return;
  }
}

void work() {
  if (r_head == NULL) {
    return;
  } else {
    Transmit_Packet *t_packet;
    if ((t_packet = (Transmit_Packet *)malloc(sizeof(Transmit_Packet))) == NULL) {
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
    current_signaltype = signaltype;
    float bright_thresh;
    float accX, accY, accZ;
    float gyroX, gyroY, gyroZ;
    float pitch, roll, yaw;
    float temp;
    float data[4];

    for (int i = 0; i < 4; i++) {
      t_packet->data[i] = r_head->data[i];
      data[i] = r_head->data[i];
    }
    for (int i = 4; i < 9; i++) {
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
      case 40:
        M5.IMU.getTempData(&temp);  // Stores the inertial sensor temperature to temp.
        if (temp >= -10 & temp < 40) {
          t_packet->data[0] = get_dist(temp);
        } else if (t_packet->data[0] >= -10 & t_packet->data[0] < 40) {
          t_packet->data[0] = get_dist(t_packet->data[0]);
        } else {
          t_packet->data[0] = get_dist(20);
        }
        break;
      case 50:
        bright_thresh = EEPROM.read(0);
        t_packet->data[0] = (digitalRead(line_tracer) >= bright_thresh) ? 1 : 0;
        break;
      case 80:
        M5.IMU.getGyroData(&gyroX, &gyroY, &gyroZ);
        M5.IMU.getAccelData(&accX, &accY, &accZ);  // Stores the triaxial accelerometer.
        M5.IMU.getAhrsData(&pitch, &roll, &yaw);  // Stores the inertial sensor attitude.
        M5.IMU.getTempData(&temp);
        bmm150.getMagnetData(&magnetX, &magnetY, &magnetZ);
        gyroX -= init_gyroX;
        gyroY -= init_gyroY;
        gyroZ -= init_gyroZ;
        magnetX = (magnetX - magoffsetX) * magscaleX;
        magnetY = (magnetY - magoffsetY) * magscaleY;
        magnetZ = (magnetZ - magoffsetZ) * magscaleZ;
        float head_dir = atan2(magnetX, magnetY);
        if (head_dir < 0) head_dir += 2 * PI;
        if (head_dir > 2 * PI) head_dir -= 2 * PI;
        head_dir *= RAD_TO_DEG;
        Now_w = micros();
        deltat_w = ((Now_w - lastUpdate_w) / 1000000.0f);  // 0.09
        lastUpdate_w = Now_w;
#ifdef MADGWICK
        MadgwickQuaternionUpdate(accX, accY, accZ, gyroX * DEG_TO_RAD,
                                 gyroY * DEG_TO_RAD, gyroZ * DEG_TO_RAD,
                                 -magnetX, magnetY, -magnetZ, deltat_w);
#endif
#ifdef MAHONY
        MahonyQuaternionUpdate(accX, accY, accZ, gyroX * DEG_TO_RAD,
                               gyroY * DEG_TO_RAD, gyroZ * DEG_TO_RAD, -magnetX,
                               magnetY, -magnetZ, deltat_w);
        // delay(10); // adjust sampleFreq = 50Hz
#endif
        yaw = atan2(
            2.0f * (*(getQ() + 1) * *(getQ() + 2) + *getQ() * *(getQ() + 3)),
            *getQ() * *getQ() + *(getQ() + 1) * *(getQ() + 1) -
                *(getQ() + 2) * *(getQ() + 2) - *(getQ() + 3) * *(getQ() + 3));
        pitch = -asin(
            2.0f * (*(getQ() + 1) * *(getQ() + 3) - *getQ() * *(getQ() + 2)));
        roll = atan2(
            2.0f * (*getQ() * *(getQ() + 1) + *(getQ() + 2) * *(getQ() + 3)),
            *getQ() * *getQ() - *(getQ() + 1) * *(getQ() + 1) -
                *(getQ() + 2) * *(getQ() + 2) + *(getQ() + 3) * *(getQ() + 3));
        yaw = -0.5 * M_PI - yaw;
        if (yaw < 0) yaw += 2 * M_PI;
        if (yaw > 2 * M_PI) yaw -= 2 * M_PI;
        pitch *= RAD_TO_DEG;
        yaw *= RAD_TO_DEG;
        roll *= RAD_TO_DEG;
        bright_thresh = EEPROM.read(0);
        
        t_packet->data[0] = accX;
        t_packet->data[1] = accY;
        t_packet->data[2] = accZ;
        t_packet->data[3] = gyroX;
        t_packet->data[4] = gyroY;
        t_packet->data[5] = gyroZ;
        t_packet->data[6] = yaw;
        t_packet->data[7] = temp;
        t_packet->data[8] = (analogRead(line_tracer) >= bright_thresh) ? 1.0 : 0.0;
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

void serialTransmit(Transmit_Packet *t_packet) {
  if (t_packet != NULL) {
    if (t_packet->signaltype == 80) {
      Serial.printf(">#{\"SignalType\" : %d, \"UniqueID\" : %d, \"AccelX\" : %.2f, \"AccelY\" : %.2f, \"AccelZ\" : %.2f, \
        \"GyroX\" : %.2f, \"GyroY\" : %.2f, \"GyroZ\" : %.2f, \"Direction\" : %.2f, \"Temperature\" : %.2f, \"LineTracer\" : %.2f}\n",
        t_packet->signaltype, t_packet->unique_id, t_packet->data[0],
        t_packet->data[1], t_packet->data[2], t_packet->data[3],
        t_packet->data[4], t_packet->data[5], t_packet->data[6],
        t_packet->data[7], t_packet->data[8]);
    } else {
      Serial.printf(">#{\"SignalType\" : %d, \"UniqueID\" : %d, \"Data1\" : %.2f, \"Data2\" : %.2f, \"Data3\" : %.2f, \
        \"Data4\" : %.2f, \"Data5\" : %.2f, \"Data6\" : %.2f, \"Data7\" : %.2f, \"Data8\" : %.2f, \"Data9\" : %.2f}\n",
        t_packet->signaltype, t_packet->unique_id, t_packet->data[0],
        t_packet->data[1], t_packet->data[2], t_packet->data[3],
        t_packet->data[4], t_packet->data[5], t_packet->data[6],
        t_packet->data[7], t_packet->data[8]);
    }
    free(t_packet);
  }
}

float get_dist(float temperature) {
  float velocity = 331.5 + 0.61 * (float)temperature;             // m/s
  int pulse_time = pulseIn(echo_pin, HIGH, 20000);                // us
  float distance = velocity * 1000 * (pulse_time / 2) / 1000000;  // mm
  return distance;
}
