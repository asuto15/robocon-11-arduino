// define must ahead #include <M5Stack.h>
#define M5STACK_MPU6886 
// #define M5STACK_MPU9250 
// #define M5STACK_MPU6050
// #define M5STACK_200Q

#include <LovyanGFX.hpp>
#define _M5DISPLAY_H_
class M5Display {};

#include <M5Stack.h>

static LGFX lcd;
static LGFX_Sprite sprite(&lcd);

#include "utility/MPU9250.h"
MPU9250 IMU;

// the setup routine runs once when M5Stack starts up
void setup(){

  // Initialize the M5Stack object
  M5.begin();
  M5.Power.begin();
  M5.IMU.Init();
  IMU.initMPU9250();
  delay(100);
  IMU.initAK8963(IMU.magCalibration);

  IMU.magbias[0] = -120;
  IMU.magbias[1] = -1475;
  IMU.magbias[2] = -120;

  lcd.init();
  lcd.setColorDepth(16);
  sprite.setColorDepth(8);
  sprite.createSprite(320, 240);  
  lcd.fillScreen(TFT_BLACK);
  sprite.setTextColor(TFT_WHITE);
}

// the loop routine runs over and over again forever
void loop() {
  if (IMU.readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01) {
    IMU.readMagData(IMU.magCount);
    IMU.getMres();

    float mg_0 = (float)IMU.magCount[0] * IMU.mRes * IMU.magCalibration[0] - IMU.magbias[0];
    float mg_1 = (float)IMU.magCount[1] * IMU.mRes * IMU.magCalibration[1] - IMU.magbias[1];
    float mg_2 = (float)IMU.magCount[2] * IMU.mRes * IMU.magCalibration[2] - IMU.magbias[2];

    float at = atan(mg_0 / mg_1);
    float x = cos(PI / 2 - at) * 20;
    float y = sin(PI / 2 - at) * 20;

    sprite.clear();
    sprite.fillTriangle(160 - x, 120 - y, 160 + x, 120 + y, 160 + mg_1 / 3, 120 - mg_0 / 3, TFT_RED);

    sprite.setTextSize(1);
    sprite.setCursor(0, 0);
    sprite.printf("Bias:%f %f %f", IMU.magbias[0], IMU.magbias[1], IMU.magbias[2]);
    sprite.setCursor(0, 10);
    sprite.printf("Calb:%f %f %f", IMU.magCalibration[0], IMU.magCalibration[1], IMU.magCalibration[2]);
    sprite.setCursor(0, 20);
    sprite.printf("mres:%f", IMU.mRes);
    sprite.setCursor(0, 40);
    sprite.printf("coun:%6d %6d %6d", (int16_t)IMU.magCount[0], (int16_t)IMU.magCount[1], (int16_t)IMU.magCount[2]);

    sprite.setTextSize(2);
    sprite.setCursor(0, 190);
    sprite.printf(" %6.1f %6.1f %6.1f", mg_0, mg_1, mg_2);

    sprite.pushSprite(0, 0);
  }

  delay(100);
}