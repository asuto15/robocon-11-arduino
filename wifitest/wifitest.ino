#include <WiFi.h>
#include <WiFiUDP.h>
const char ssid[] = "elecom-c39b1f"; // SSID
const char pass[] = "2zt3s4e7";  // password
static WiFiUDP wifiUdp; 
static const char *raspberryIP = "192.168.10.3";
static const int raspberryPort = 9000; //送信先のポート

IPAddress remoteIP; // 相手のIPアドレス
int port;

static void WiFi_setup()
{
 static const int kLocalPort = 4321;  //自身のポート
 WiFi.begin(ssid, pass);
 while( WiFi.status() != WL_CONNECTED) {
   delay(500);  
 }  
 wifiUdp.begin(kLocalPort);
}

static void Serial_setup()
{
 Serial.begin(115200);
 Serial.println(""); // to separate line  
}

void setup() {
 Serial_setup();
 WiFi_setup();
}

void loop() 
{
 int count=0;
 char i[64];
 
 while(1){
   count++;
   
  //パケットの送信 
 wifiUdp.beginPacket(raspberryIP, raspberryPort);
 wifiUdp.printf("ESP32Dev Bord: %d", count);
 wifiUdp.endPacket();
 
 //パケットの受信
  if (wifiUdp.parsePacket()) {
   wifiUdp.read(i,64);
   remoteIP = wifiUdp.remoteIP();
   port = wifiUdp.remotePort();    
   Serial.print(remoteIP);
   Serial.print(" / ");
   Serial.print(port);
   Serial.print(" / ");
   Serial.println(i); // UDP通信で来た値を表示
 }
  
 delay(3000);
 }
}
