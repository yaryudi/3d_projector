/*
 * CS-D808 드라이브 시리얼 제어 및 상태 모니터링
 * * [배선]
 * - Arduino D2 -> P1 PUL+ (Step)
 * - Arduino D3 -> P1 DIR+ (Direction)
 * - Arduino D4 -> P1 ENA+ (Enable)
 * - Arduino D5 -> P2 Pend+ (In-Position)
 * - Arduino D6 -> P2 ALM+ (Alarm)
 * - Arduino GND -> P1 (PUL-, DIR-, ENA-) 및 P2 (Pend-, ALM-)
 * * [시리얼 명령어] (시리얼 모니터에서 입력)
 * m[숫자] : [숫자] 만큼 스텝 이동 (예: m1600, m-3200)
 * e        : 모터 동력 켜기 (Enable)
 * d        : 모터 동력 끄기 (Disable)
 * s        : 현재 상태 (In-Position, Alarm) 보고
 */

#include <AccelStepper.h>

// 1. 핀 정의
const int STEP_PIN = 2;
const int DIR_PIN  = 3;
const int ENA_PIN  = 4;
const int IN_POS_PIN = 5; // In-Position (Pend+)
const int ALM_PIN  = 6; // Alarm (ALM+)

// 2. AccelStepper 객체 생성 (DRIVER 모드)
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// 3. 상태 변수
long targetPosition = 0; // 목표 위치 저장

void setup() {
  // 4. 시리얼 통신 시작
  Serial.begin(9600);
  Serial.println("--- CS-D808 시리얼 제어기 ---");
  Serial.println("명령어: m[스텝], e, d, s");

  // 5. 제어핀 (P1) 설정
  pinMode(ENA_PIN, OUTPUT);
  digitalWrite(ENA_PIN, LOW); // 기본상태: 모터 동력 OFF (Disabled)
  
  // 6. 상태핀 (P2) 설정 (INPUT_PULLUP)
  // 드라이브의 출력은 '오픈 컬렉터' 방식입니다.
  // 아두이노의 내부 풀업저항을 사용해야 신호를 읽을 수 있습니다.
  pinMode(IN_POS_PIN, INPUT_PULLUP);
  pinMode(ALM_PIN, INPUT_PULLUP);

  // 7. 모터 초기 설정
  stepper.setMaxSpeed(4000);     // 최대 속도 (스텝/초)
  stepper.setAcceleration(2000); // 가속도 (스텝/초^2)
}

void loop() {
  // 1. 시리얼 명령이 있는지 확인
  parseSerial();

  // 2. 모터가 움직여야 한다면 구동
  if (stepper.distanceToGo() != 0) {
    stepper.run();
  }
  
  // 3. 알람 상태 실시간 확인
  checkAlarm();
}

// 시리얼 명령을 해석하는 함수
void parseSerial() {
  if (Serial.available() > 0) {
    char cmd = Serial.read(); // 첫 글자(명령어) 읽기

    if (cmd == 'm') { // 'm' (Move)
      long steps = Serial.parseInt(); // 숫자(스텝) 읽기
      Serial.print("명령: ");
      Serial.print(steps);
      Serial.println(" 스텝 이동");
      
      stepper.move(steps); // AccelStepper에 상대 이동 명령
      // 참고: move()는 상대 이동, moveTo()는 절대 위치 이동입니다.

    } else if (cmd == 'e') { // 'e' (Enable)
      digitalWrite(ENA_PIN, HIGH); // ENA+ 핀에 5V 인가 (활성화)
      Serial.println("상태: 모터 활성화 (Enable)");
      
    } else if (cmd == 'd') { // 'd' (Disable)
      digitalWrite(ENA_PIN, LOW); // ENA+ 핀에 0V 인가 (비활성화)
      Serial.println("상태: 모터 비활성화 (Disable)");
      
    } else if (cmd == 's') { // 's' (Status)
      reportStatus(); // 현재 상태 보고
    }
    
    // 시리얼 버퍼 비우기 (다음 명령을 위해)
    while(Serial.available() > 0) Serial.read();
  }
}

// 현재 상태를 시리얼 모니터에 출력하는 함수
void reportStatus() {
  Serial.println("--- 현재 상태 보고 ---");
  
  // In-Position 상태 확인
  // INPUT_PULLUP을 사용했으므로, LOW 신호가 "정상(도착)" 입니다.
  if (digitalRead(IN_POS_PIN) == LOW) {
    Serial.println("In-Position: YES (목표 위치에 안정적으로 도착)");
  } else {
    Serial.println("In-Position: NO (이동 중이거나 목표 위치 이탈)");
  }

  // 알람 상태 확인
  // INPUT_PULLUP을 사용했으므로, LOW 신호가 "알람 발생" 입니다.
  if (digitalRead(ALM_PIN) == LOW) {
    Serial.println("Alarm: YES (!!! 드라이브 에러 발생 !!!)");
  } else {
    Serial.println("Alarm: NO (정상)");
  }
  
  Serial.print("AccelStepper 목표: ");
  Serial.println(stepper.distanceToGo());
  Serial.println("---------------------");
}

// 실시간 알람 확인 함수
void checkAlarm() {
  // 알람이 발생하면 (LOW 신호)
  if (digitalRead(ALM_PIN) == LOW) {
    // 즉시 모터 동력을 차단하고 모든 목표를 정지
    digitalWrite(ENA_PIN, LOW); 
    stepper.stop();
    stepper.setCurrentPosition(0); // 위치 리셋
    
    Serial.println("***********************************");
    Serial.println("!!! 비상 정지: 드라이브 알람 발생 !!!");
    Serial.println("배선, 전압, 모터 상태를 확인하세요.");
    Serial.println("***********************************");
    
    // 알람이 해결될 때까지 1초마다 메시지 출력
    while(digitalRead(ALM_PIN) == LOW) {
      delay(1000);
      Serial.println("...알람 상태... 드라이브 전원 확인...");
    }
  }
}