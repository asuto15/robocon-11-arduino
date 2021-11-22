int count = 0;
long count4 = 0;
long count5 = 0;
long previous = 0;
long t1 = 33820;
long t2 = 13500;
//long t1 = 200;
//long t2 = 200;
long t3 = 2665;
float step_degree = 0.18;

void setup() {
  // put your setup code here, to run once:
  pinMode(5,OUTPUT);
  pinMode(4,OUTPUT);
  pinMode(12,INPUT);
}



void loop() {
  // put your main code here, to run repeatedly:
  golong();
}

void golong() {
  while (millis() - previous <= t1){
    go();
    go();
    go();
    go();
  }
  previous = millis();
}

void goshort() {
  while (millis() - previous <= t2){
    go();
    go();
    go();
    go();
  }
  previous = millis();
}

void turn() {
  while (millis() - previous <= t3){
    curve();
    curve();
    curve();
    curve();
  }
  previous = millis();
}

void go() {
  digitalWrite(5,HIGH);
  digitalWrite(4,HIGH);
  delayMicroseconds(25);
  digitalWrite(5,LOW);
  digitalWrite(4,LOW);
  delayMicroseconds(25);
  count4++;
  count5++;
}

void curve() {
  digitalWrite(5,LOW);
  digitalWrite(4,HIGH);
  delayMicroseconds(250);
  digitalWrite(5,LOW);
  digitalWrite(4,LOW);
  delayMicroseconds(250);
  count4++;
}
