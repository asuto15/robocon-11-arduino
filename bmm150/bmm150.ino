#define M5STACK_MPU6886

#include <Arduino.h>
#include "Preferences.h"
#include "M5Stack.h"
#include "math.h"
#include "M5_BMM150.h"
#include "M5_BMM150_DEFS.h"

Preferences prefs;

struct bmm150_dev dev;
bmm150_mag_data mag_offset; // Compensation magnetometer float data storage
bmm150_mag_data mag_max;
bmm150_mag_data mag_min;
int mag_compensate_x = 455;
int mag_compensate_y = 760;
int mag_compensate_z = 470;

int8_t i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *read_data, uint16_t len){
    if(M5.I2C.readBytes(dev_id, reg_addr, len, read_data)){ // Check whether the device ID, address, data exist.
        return BMM150_OK;
    }else{
        return BMM150_E_DEV_NOT_FOUND;
    }
}

int8_t i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *read_data, uint16_t len){
    if(M5.I2C.writeBytes(dev_id, reg_addr, read_data, len)){    //Writes data of length len to the specified device address.
        return BMM150_OK;
    }else{
        return BMM150_E_DEV_NOT_FOUND;
    }
}

int8_t bmm150_initialization(){
    int8_t rslt = BMM150_OK;

    dev.dev_id = 0x10;  //Device address setting.
    dev.intf = BMM150_I2C_INTF; //SPI or I2C interface setup.
    dev.read = i2c_read;    //Read the bus pointer.
    dev.write = i2c_write;  //Write the bus pointer.
    dev.delay_ms = delay;

    // Set the maximum range range
    mag_max.x = -2000;
    mag_max.y = -2000;
    mag_max.z = -2000;

    // Set the minimum range
    mag_min.x = 2000;
    mag_min.y = 2000;
    mag_min.z = 2000;

    rslt = bmm150_init(&dev);   //Memory chip ID.
    dev.settings.pwr_mode = BMM150_NORMAL_MODE;
    rslt |= bmm150_set_op_mode(&dev);   //Set the sensor power mode.
    dev.settings.preset_mode = BMM150_PRESETMODE_ENHANCED;
    rslt |= bmm150_set_presetmode(&dev);    //Set the preset mode of .
    return rslt;
}

void bmm150_offset_save(){  //Store the data.
    prefs.begin("bmm150", false);
    prefs.putBytes("offset", (uint8_t *)&mag_offset, sizeof(bmm150_mag_data));
    prefs.end();
}

void bmm150_offset_load(){  //load the data.
    if(prefs.begin("bmm150", true)){
        prefs.getBytes("offset", (uint8_t *)&mag_offset, sizeof(bmm150_mag_data));
        prefs.end();
        Serial.println(">bmm150 load offset finish....");
    }else{
        Serial.println(">bmm150 load offset failed....");
    }
}

void setup() {
    M5.begin(true, false, true, false); //Init M5Core(Initialize LCD, serial port).
    M5.Power.begin();   //Init Power module.
    Wire.begin(21, 22, 400000); //Set the frequency of the SDA SCL.

    while(bmm150_initialization() != BMM150_OK){
        Serial.println(">BMM150 init failed");
        delay(2000);
    }
    bmm150_offset_load();
}

void bmm150_calibrate(uint32_t calibrate_time){ //bbm150 data calibrate.
    uint32_t calibrate_timeout = 0;

    calibrate_timeout = millis() + calibrate_time;
    Serial.printf(">Go calibrate, use %d ms \r\n", calibrate_time);  //The serial port outputs formatting characters.  串口输出格式化字符
    Serial.printf(">running ...");

    while (calibrate_timeout > millis()){
        bmm150_read_mag_data(&dev); //read the magnetometer data from registers.
        if(dev.data.x){
            mag_min.x = (dev.data.x < mag_min.x) ? dev.data.x : mag_min.x;
            mag_max.x = (dev.data.x > mag_max.x) ? dev.data.x : mag_max.x;
        }
        if(dev.data.y){
            mag_max.y = (dev.data.y > mag_max.y) ? dev.data.y : mag_max.y;
            mag_min.y = (dev.data.y < mag_min.y) ? dev.data.y : mag_min.y;
        }
        if(dev.data.z){
            mag_min.z = (dev.data.z < mag_min.z) ? dev.data.z : mag_min.z;
            mag_max.z = (dev.data.z > mag_max.z) ? dev.data.z : mag_max.z;
        }
        delay(100);
    }

    mag_offset.x = (mag_max.x + mag_min.x) / 2;
    mag_offset.y = (mag_max.y + mag_min.y) / 2;
    mag_offset.z = (mag_max.z + mag_min.z) / 2;
    bmm150_offset_save();

    Serial.printf(">calibrate finish ... \n");
    Serial.printf(">mag_max.x: %.2f x_min: %.2f \n", mag_max.x, mag_min.x);
    Serial.printf(">y_max: %.2f y_min: %.2f \n", mag_max.y, mag_min.y);
    Serial.printf(">z_max: %.2f z_min: %.2f \n", mag_max.z, mag_min.z);
}
//dev.data.x, dev.data.y, dev.data.z, head_dir
void loop() {
    M5.update();    //Read the press state of the key.
    bmm150_read_mag_data(&dev);
    dev.data.x += mag_compensate_x;
    dev.data.y += mag_compensate_y;
    dev.data.z += mag_compensate_z;
    float head_dir = atan2(dev.data.x -  mag_offset.x, dev.data.y - mag_offset.y) * 180.0 / M_PI;
    Serial.printf(">Magnetometer data, heading %.2f\n", head_dir);
    Serial.printf(">MAG X : %.2f \t MAG Y : %.2f \t MAG Z : %.2f \n", dev.data.x, dev.data.y, dev.data.z);
    Serial.printf(">MID X : %.2f \t MID Y : %.2f \t MID Z : %.2f \n", mag_offset.x, mag_offset.y, mag_offset.z);

    if(M5.BtnA.wasPressed()){
        bmm150_calibrate(10000);
    }
    delay(100);
}
