const int sw_out = 9;
int target_value = 1023;
void setup(){
  pinMode(sw_out,OUTPUT);
}

void loop(){
  uint16_t data = analogRead(A0);
//  uint8_t duty = (target_value < data) ? 0 : ((target_value - data) >> 2);
  uint16_t duty = ((target_value - data) >> 2);
  //analogWrite(sw_out, duty);
  digitalWrite(sw_out, HIGH);
  delayMicroseconds(8000);
  //delay(3);
  digitalWrite(sw_out, LOW);
  delayMicroseconds(2000);
  //delay(3);
}
