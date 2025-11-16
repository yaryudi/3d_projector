// Arduino Uno - A0 빠른 스트리밍 (정수 ADC 값 전송, 지연 없음)
const int analogPin = A0;

void setup() {
  Serial.begin(230400); // 120 Hz UI 갱신을 위해 여유 있는 보레이트 권장
}

void loop() {
  int sensorValue = analogRead(analogPin); // 0~1023
  Serial.println(sensorValue);             // 정수 + 개행
  // delay 없음a
}