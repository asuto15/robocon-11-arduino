#include <M5Stack.h>
#include <TinyGPS++.h>

HardwareSerial GPS_s(2);
TinyGPSPlus gps;

void setup() {
    M5.begin();
    GPS_s.begin(9600);
}

void loop() {
    while (!gps.location.isUpdated()) {
        while (GPS_s.available() > 0) {
            if (gps.encode(GPS_s.read())) {
                break;
            }
        }
    }
    Serial.printf("lat: %f, lng: %f\r\n", gps.location.lat(), gps.location.lng());
}
