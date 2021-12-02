#include <M5Stack.h>

void setup() {
  // put your setup code here, to run once:
  pinMode(2,INPUT);
  pinMode(5,INPUT);
  Serial.begin(115200);
  M5.begin(); //Init M5Core.
  M5.Power.begin(); //Init Power module.
}

void loop() {
  // put your main code here, to run repeatedly:
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setCursor(10, 10); //Move the cursor position to (x,y).
  M5.Lcd.setTextColor(WHITE); //Set the font color to white.
  M5.Lcd.setTextSize(4);  //Set the font size.
  int t = analogRead(2);
  M5.Lcd.printf("%d",t);  //Serial output format string.
  Serial.println(t);
  delay(100);
}
