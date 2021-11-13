union byte2int {
  byte b[2];
  int integar;
};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  byte x = 64;
  byte y = byteSwitch(x);
  Serial.print(x);
  Serial.print(" : ");
  Serial.println(y);
  
  union byte2int b2i;
  b2i.b[1] = 0;
  b2i.b[0] = 0x50;
  Serial.println(b2i.integar);
}

void loop() {
  // put your main code here, to run repeatedly:
  }

byte byteSwitch(byte x){
  byte y = 0;
  for (int i = 0; i < 8; i++){
    bitWrite(y,i,bitRead(x,8-i));
  }
  return y;
}
