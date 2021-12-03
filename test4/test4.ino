void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Transmission Start");
  int i = 0;
  int source = 0;
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0) {
    //Serial.print("Stop!");
    int size = Serial.available();
    byte *data;

    if ((data = (byte *) malloc(sizeof(byte) * size)) == NULL) {
      Serial.println("malloc error");
    } else {
      for (int i = 0; i < size; i++) {
        data[i] = Serial.read();
        if (data[i] != 10) {
         Serial.println(char(data[i]));
        }
      }
      //Serial.print("OK!");
    }
  }
}
