#include <M5Stack.h>
#include <TinyGPS++.h>

HardwareSerial GPS_s(2);
TinyGPSPlus gps;

void setup() {
    M5.begin();
    M5.Power.begin();
    GPS_s.begin(9600);
    termInit();
    Serial.println("hello");
    M5.Lcd.setTextFont(4);
    M5.Lcd.println(("GPS Raw Example"));
}

void loop() {
    while (!gps.location.isUpdated()) {
        while (GPS_s.available() > 0) {
            if (gps.encode(GPS_s.read())) {
                break;
            }
        }
    }
    float gps_lat = gps.location.lat();
    float gps_lng = gps.location.lng();
    Serial.printf("lat: %f, lng: %f\r\n", gps_lat, gps_lng);
    M5.Lcd.println(("lat: %f, lng: %f\r\n", gps_lat, gps_lng));
}
