#include <M5Stack.h>
void task0(void* arg) {
  int cnt = 0;
  while (1) {
    printf(">task2 thread_cnt=%ld\n", cnt++);
    delay(1000);
  }
} 
void task1(void* arg) {
  int cnt = 0;
  while (1) {
    printf(">task1 thread_cnt=%ld\n", cnt++);
    delay(1500);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Transmission Start");
  xTaskCreatePinnedToCore(task0, "Task0", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(task1, "Task1", 4096, NULL, 1, NULL, 1);
}

void loop() {
  static int cnt = 0;
  printf(">Maintask thread_cnt=%ld\n", cnt++);
  delay(1200);
}
