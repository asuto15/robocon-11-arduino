


typedef struct {
  int signaltype;
  long rand_id;
  bool lock_tire;
  bool rot_dir;
  bool angle_time;
  double data[2];
} Receive_Packet;

Receive_Packet receive_packet;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  //receive_packet = {1,2,True,False,True,{1,2}}
  receive_packet.signaltype = 10;
  receive_packet.rand_id = 1234;
  receive_packet.lock_tire = true;
  receive_packet.rot_dir = false;
  receive_packet.angle_time = true;
  receive_packet.data[0] = 12;
  receive_packet.data[1] = 13;
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("OK!");
  delay(1000);
  Serial.println(receive_packet.signaltype);
  Serial.println(receive_packet.data[0]);
}



