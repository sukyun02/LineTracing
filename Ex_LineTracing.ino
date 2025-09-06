#include <Arduino.h>

// 센서, 모터 핀
#define S1_PIN 13
#define S2_PIN 12
#define S3_PIN 11
#define S4_PIN 10
#define S5_PIN 9

#define L_ENA 2
#define L_DIR 3
#define L_PUL 4
#define R_ENA 5
#define R_DIR 6
#define R_PUL 7

// 초음파 핀
#define Trig 8
#define Echo A0

float Kp = 0.1, Ki = 0.0001, Kd = 0.01;
int basePulseDelay = 10;
int minPulseDelay = 5;
int maxPulseDelay = 30;
int numPulse = 10;

float error = 0, lastError = 0, integral = 0;
int weights[5] = { -3, -2, 0, 2, 4 };

long measureDistance() {
  digitalWrite(Trig, LOW);
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig, LOW);
  long duration = pulseIn(Echo, HIGH, 20000);
  long distance = duration / 58.0;
  return distance;
}

void readSensors(int* sensorArray) {
  sensorArray[0] = digitalRead(S1_PIN);
  sensorArray[1] = digitalRead(S2_PIN);
  sensorArray[2] = digitalRead(S3_PIN);
  sensorArray[3] = digitalRead(S4_PIN);
  sensorArray[4] = digitalRead(S5_PIN);
}

float calculateError(int* sensorArray) {
  int sum = 0;
  int count = 0;
  for(int i=0; i<5; i++) {
    if(sensorArray[i] == 0) {
      sum += weights[i];
      count++;
    }
  }
  if(count == 0) {
    if(lastError > 0) return 3.0;
    else return -3.0;
  }
  return (float)sum / (float)count;
}

void driveMotors(int leftDelay, int rightDelay, int leftDir, int rightDir, int numPulse) {
  digitalWrite(L_ENA, HIGH);
  digitalWrite(R_ENA, HIGH);

  digitalWrite(L_DIR, leftDir);
  digitalWrite(R_DIR, rightDir);

  for(int i = 0; i < numPulse; i++) {
    digitalWrite(L_PUL, HIGH); digitalWrite(R_PUL, HIGH);
    delayMicroseconds(2);
    digitalWrite(L_PUL, LOW); digitalWrite(R_PUL, LOW);

    int maxDelay = max(leftDelay, rightDelay);
    int minDelay = min(leftDelay, rightDelay);

    delayMicroseconds(minDelay);
    if(maxDelay > minDelay)
      delayMicroseconds(maxDelay - minDelay);
  }
}

void stopMotors() {
  digitalWrite(L_ENA, LOW);
  digitalWrite(R_ENA, LOW);
  digitalWrite(L_PUL, LOW);
  digitalWrite(R_PUL, LOW);
}

unsigned long startTime = 0;

void setup() {
  pinMode(S1_PIN, INPUT);
  pinMode(S2_PIN, INPUT);
  pinMode(S3_PIN, INPUT);
  pinMode(S4_PIN, INPUT);
  pinMode(S5_PIN, INPUT);

  pinMode(L_ENA, OUTPUT);
  pinMode(L_DIR, OUTPUT);
  pinMode(L_PUL, OUTPUT);
  pinMode(R_ENA, OUTPUT);
  pinMode(R_DIR, OUTPUT);
  pinMode(R_PUL, OUTPUT);

  pinMode(Trig, OUTPUT);
  pinMode(Echo, INPUT);

  digitalWrite(L_ENA, HIGH);
  digitalWrite(R_ENA, HIGH);

  startTime = millis();

  Serial.println("시스템 시작");
}

void loop() {
  static bool ultrasonicActive = false;
  static unsigned long lastUltrasonicTime = 0;
  const unsigned long ultrasonicInterval = 120;
  const unsigned long ultrasonicCooldown = 1500;
  static bool isStopped = false;
  static unsigned long stopStartTime = 0;
  static unsigned long cooldownStartTime = 0;
  static int stopCount = 0;
  const int maxStop = 4;

  unsigned long now = millis();

  // 1. 15초 경과 후 초음파 활성화
  if (!ultrasonicActive && (now - startTime >= 15000UL)) {
    ultrasonicActive = true;
    Serial.println("---- [15초 경과: 초음파 감지 시작] ----");
  }

  // 2. 정지 후 재출발
  if(isStopped) {
    if(now - stopStartTime >= 5000) {
      digitalWrite(L_ENA, HIGH);
      digitalWrite(R_ENA, HIGH);
      isStopped = false;
      cooldownStartTime = now;
      Serial.println(">>> 5초 정지 후 재출발!");
    } else {
      stopMotors();
      return;
    }
  }

  // 3. 쿨타임 체크
  static bool inCooldown = false;
  if(cooldownStartTime > 0 && (now - cooldownStartTime < ultrasonicCooldown)) {
    inCooldown = true;
  } else {
    inCooldown = false;
  }

  // 4. 초음파 감지
  if(ultrasonicActive && !isStopped && !inCooldown && stopCount < maxStop) {
    if(now - lastUltrasonicTime > ultrasonicInterval) {
      long distance = measureDistance();
      Serial.print("[초음파]: ");
      Serial.print(distance);
      Serial.println("cm");

      if(distance > 0 && distance <= 30) {
        Serial.print("*** 장애물 감지: ");
        Serial.print(distance);
        Serial.println("cm → 정지!");
        stopMotors();
        isStopped = true;
        stopStartTime = now;
        stopCount++;
        lastUltrasonicTime = now;
        return;
      }
      lastUltrasonicTime = now;
    }
  }

  // 5. 라인트레이싱 (동일)
  int sensors[5];
  readSensors(sensors);

  error = calculateError(sensors);

  integral += error;
  float derivative = error - lastError;
  float correction = Kp * error + Ki * integral + Kd * derivative;

  int leftPulseDelay = basePulseDelay + correction * 5;
  int rightPulseDelay = basePulseDelay - correction * 5;

  if(
    (sensors[0]==0 && sensors[1]==0 && sensors[2]==1 && sensors[3]==1 && sensors[4]==1) ||
    (sensors[0]==0 && sensors[1]==0 && sensors[2]==0 && sensors[3]==0 && sensors[4]==0) ||
    (error <= -2.0)
  ) {
    leftPulseDelay = basePulseDelay;
    rightPulseDelay = basePulseDelay;
    driveMotors(leftPulseDelay, rightPulseDelay, HIGH, HIGH, numPulse);
  }
  else if(
    (sensors[0]==1 && sensors[1]==1 && sensors[2]==1 && sensors[3]==0 && sensors[4]==0) ||
    (error >= 1.0)
  ) {
    leftPulseDelay = basePulseDelay;
    rightPulseDelay = basePulseDelay;
    driveMotors(leftPulseDelay, rightPulseDelay, LOW, LOW, numPulse);
  }
  else {
    leftPulseDelay = constrain(leftPulseDelay, minPulseDelay, maxPulseDelay);
    rightPulseDelay = constrain(rightPulseDelay, minPulseDelay, maxPulseDelay);
    driveMotors(leftPulseDelay, rightPulseDelay, HIGH, LOW, numPulse);
  }

  lastError = error;
  delay(1);
}