// define must ahead #include <M5Stack.h>
#define M5STACK_MPU6886
// #define M5STACK_MPU9250
// #define M5STACK_MPU6050
// #define M5STACK_200Q

#include "MovingAverage.hpp"
#include <LovyanGFX.hpp>
#define _M5DISPLAY_H_
class M5Display
{
};

#include <M5Stack.h>

static LGFX lcd;
static LGFX_Sprite sprite(&lcd);

#include "utility/MPU9250.h"
MPU9250 IMU;

MovingAverage<int16_t> mg_0(1000);
MovingAverage<int16_t> mg_1(1000);
MovingAverage<int16_t> mg_2(1000);

float temp = 0.0F;

// the setup routine runs once when M5Stack starts up
void setup()
{

    // Initialize the M5Stack object
    M5.begin();
    M5.Power.begin();
    M5.IMU.Init();
    IMU.initMPU9250();
    delay(100);
    IMU.initAK8963(IMU.magCalibration);

    lcd.init();
    lcd.setColorDepth(16);
    sprite.setColorDepth(8);
    sprite.createSprite(320, 240);
    sprite.setTextColor(TFT_WHITE);
}

// the loop routine runs over and over again forever
void loop()
{
    if (IMU.readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01)
    {
        IMU.readMagData(IMU.magCount);
        IMU.getMres();

        int count = mg_0.setValue(IMU.magCount[0]);
        mg_1.setValue(IMU.magCount[1]);
        mg_2.setValue(IMU.magCount[2]);

        IMU.magCount[0] = mg_0.getAverage(10);
        IMU.magCount[1] = mg_1.getAverage(10);
        IMU.magCount[2] = mg_2.getAverage(10);

        float mg[3];
        mg[0] = (float)IMU.magCount[0] * IMU.mRes * IMU.magCalibration[0] - IMU.magbias[0];
        mg[1] = (float)IMU.magCount[1] * IMU.mRes * IMU.magCalibration[1] - IMU.magbias[1];
        mg[2] = (float)IMU.magCount[2] * IMU.mRes * IMU.magCalibration[2] - IMU.magbias[2];

        float at = atan(mg[0] / mg[1]);
        float x = cos(PI / 2 - at) * 20;
        float y = sin(PI / 2 - at) * 20;

        sprite.clear();
        sprite.setTextSize(1);

        sprite.fillTriangle(160 - x, 120 - y, 160 + x, 120 + y, 160 + mg[1] / 4, 120 - mg[0] / 4, TFT_RED);

        sprite.setFont(&fonts::Font2);
        sprite.setCursor(0, 0);
        sprite.println("From now on,");
        sprite.println("it finds the appropriate correction factor.");
        sprite.println();
        sprite.println("You should tilt the main unit left and right,");
        sprite.println("up and down, and in various directions,");
        sprite.println("and look for the direction in which each value of ");
        sprite.println("the three axes indicates the minimum and maximum.");

        sprite.setCursor(200, 120);
        sprite.printf("remaining time: %03d", 100 - count / 10);

        sprite.setFont(&fonts::Font2);
        sprite.setCursor(0, 120);
        sprite.printf("now: %6d %6d %6d", (int16_t)IMU.magCount[0], (int16_t)IMU.magCount[1], (int16_t)IMU.magCount[2]);
        sprite.setCursor(0, 150);
        sprite.printf("min: %6d %6d %6d", mg_0.getMin(), mg_1.getMin(), mg_2.getMin());
        sprite.setCursor(0, 170);
        sprite.printf("max: %6d %6d %6d", mg_0.getMax(), mg_1.getMax(), mg_2.getMax());

        IMU.magbias[0] = IMU.mRes * IMU.magCalibration[0] * (mg_0.getMax() + mg_0.getMin()) / 2;
        IMU.magbias[1] = IMU.mRes * IMU.magCalibration[1] * (mg_1.getMax() + mg_1.getMin()) / 2;
        IMU.magbias[2] = IMU.mRes * IMU.magCalibration[2] * (mg_2.getMax() + mg_2.getMin()) / 2;

        sprite.setCursor(0, 220);
        sprite.printf("Bias:%d %d %d", (int16_t)IMU.magbias[0], (int16_t)IMU.magbias[1], (int16_t)IMU.magbias[2]);

        sprite.setFont(&fonts::Font0);
        sprite.setCursor(180, 226);
        sprite.printf(" %6.1f %6.1f %6.1f", mg[0], mg[1], mg[2]);

        sprite.pushSprite(0, 0);

        if (count == 1000)
        {
            lcd.clear();
            lcd.setFont(&fonts::Font2);
            lcd.setCursor(0, 0);
            lcd.printf("IMU.magbias[0] = %d;", (int16_t)IMU.magbias[0]);
            lcd.println();
            lcd.printf("IMU.magbias[1] = %d;", (int16_t)IMU.magbias[1]);
            lcd.println();
            lcd.printf("IMU.magbias[2] = %d;", (int16_t)IMU.magbias[2]);
            lcd.println();

            Serial.printf("IMU.magbias[0] = %d;", (int16_t)IMU.magbias[0]);
            Serial.println();
            Serial.printf("IMU.magbias[1] = %d;", (int16_t)IMU.magbias[1]);
            Serial.println();
            Serial.printf("IMU.magbias[2] = %d;", (int16_t)IMU.magbias[2]);
            Serial.println();

            while (1)
                ;
        }
    }

    delay(100);
}
