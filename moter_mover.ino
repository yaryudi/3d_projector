#include <AccelStepper.h>

// ==========================
// 1. 핀 및 변수 정의
// ==========================
const int irSensor1 = A1; 
const int irSensor2 = A2; 
int threshold = 500;

// 모터 핀
const int STEP_PIN = 2;
const int DIR_PIN  = 3;
const int ENA_PIN  = 4; 
const int ALM_PIN  = 6;   

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// 기본값
float motorSpeed = 0.0;
float motorAccel = 400.0; 

// 센서 상태
bool s1Triggered = false;
bool s2Triggered = false;
int lastTriggered = 0; 
unsigned long lastSensorCheckTime = 0;
const long sensorInterval = 10; 

// [추가됨] 위치 전송 타이머
unsigned long lastPosTime = 0;
const long posInterval = 50; // 0.05초마다 위치 보고 (통신 과부하 방지)

String inputString = "";
bool stringComplete = false;

void setup() {
  Serial.begin(115200); 
  inputString.reserve(200);

  pinMode(irSensor1, INPUT);
  pinMode(irSensor2, INPUT);
  pinMode(ENA_PIN, OUTPUT);
  digitalWrite(ENA_PIN, LOW); 
  pinMode(ALM_PIN, INPUT_PULLUP);

  // 초기 상태 0
  stepper.setMaxSpeed(0);     
  stepper.setAcceleration(motorAccel);  
  stepper.setCurrentPosition(0); 
}

void loop() {
  // 1. 명령 처리
  if (stringComplete) {
    parseCommand(inputString);
    inputString = "";
    stringComplete = false;
  }

  // 2. 모터 구동 (무한 회전 로직 유지)
  if (stepper.distanceToGo() == 0) {
      // 멈춰있지 않도록 계속 타겟 갱신 (속도가 0이 아닐 때)
      if (stepper.speed() != 0) {
        long dir = (stepper.speed() > 0) ? 1 : -1;
        stepper.moveTo(stepper.currentPosition() + (1000000000 * dir));
      }
  }
  stepper.run(); 
  
  checkAlarm();

  unsigned long currentMillis = millis();

  // 3. 센서 감지
  if (currentMillis - lastSensorCheckTime >= sensorInterval) {
    lastSensorCheckTime = currentMillis; 
    readSensors(); 
  }

  // [추가됨] 4. 현재 위치(Step) 보고
  if (currentMillis - lastPosTime >= posInterval) {
    lastPosTime = currentMillis;
    // 포맷: "POS:12345"
    Serial.print("POS:");
    Serial.println(stepper.currentPosition());
  }
}

// === 명령 해석 ===
void parseCommand(String cmd) {
  cmd.trim();
  char type = cmd.charAt(0);
  float val = cmd.substring(1).toFloat();

  if (type == 'v') { // 속도 (음수면 역회전)
    float targetSpeed = val;
    if (targetSpeed == 0) {
      stepper.setMaxSpeed(0);
      stepper.stop();
    } else {
      stepper.setMaxSpeed(abs(targetSpeed));
      long infiniteTarget = (targetSpeed > 0) ? 1000000000L : -1000000000L;
      stepper.moveTo(stepper.currentPosition() + infiniteTarget);
    }
  } 
  else if (type == 'a') { // 가속도
    stepper.setAcceleration(abs(val));
  }
  else if (type == 'e') { digitalWrite(ENA_PIN, LOW); }
  else if (type == 'd') { digitalWrite(ENA_PIN, HIGH); }
}

void readSensors() {
  int val1 = analogRead(irSensor1);
  int val2 = analogRead(irSensor2);

  if (val1 < threshold) {
    if (!s1Triggered && lastTriggered != 1) {
      Serial.println("1"); 
      s1Triggered = true; lastTriggered = 1;   
    }
  } else { s1Triggered = false; }

  if (val2 < threshold) {
    if (!s2Triggered && lastTriggered != 2) {
      Serial.println("2"); 
      s2Triggered = true; lastTriggered = 2;   
    }
  } else { s2Triggered = false; }
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') stringComplete = true;
  }
}

void checkAlarm() {
  if (digitalRead(ALM_PIN) == LOW) digitalWrite(ENA_PIN, HIGH);
}