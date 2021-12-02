#define TrigPin 2 // D1
#define EchoPin 5 // D2
 
double speedSound = 331.5 + 0.61 * 20; // 20は現在の気温
double distance = 0;


void setup() {
  Serial.begin(9600);
  pinMode(TrigPin, OUTPUT);
  pinMode(EchoPin, INPUT);
  Serial.println("test start");
}

int n = 0;

void loop() {
  trigger();

  double t = pulseIn(EchoPin, HIGH); // μS
  Serial.print(n);
  Serial.print("\t");
  if (t > 0) {
    t = t / 2; //往復距離なので半分の時間
    distance = t * speedSound * 100 / 1000000; // 距離（cm）を計算
    Serial.println(distance);
  } else {
    Serial.println("failed");
  }
  n++;
  delay(500);
}

void trigger() {
  digitalWrite(TrigPin, LOW); 
  delayMicroseconds(2);
  digitalWrite(TrigPin, HIGH );
  delayMicroseconds( 10 );
  digitalWrite(TrigPin, LOW );
}
