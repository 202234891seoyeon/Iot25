#Iot25_HW3
아두이노로 아날로그 값 읽기

사용 코드 
const int potPin = 25;

// variable for storing the potentiometer value
int potValue = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
}

void loop() {
  // Reading potentiometer value
  potValue = analogRead(potPin);
  Serial.println(potValue);
  delay(500);
}

영상 : https://youtu.be/CZlIerRS9hI
