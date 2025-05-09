# Iot25_HW1

아두이노 Ide 설치하고, 보드 세팅 후 Led가 켜지는 실습
![image](https://github.com/user-attachments/assets/1096187e-3b93-415e-a20d-9f15baecf2d2)

### 사용한 코드
#include <Arduino.h>

#define LED 2

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(LED, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(LED, HIGH);
  Serial.println("LED is on");
  delay(1000);
  digitalWrite(LED, LOW);
  Serial.println("LED is off");
  delay(1000);
}


영상 : https://youtu.be/2f5vyQUgy2c
