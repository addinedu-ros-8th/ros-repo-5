#include <WiFi.h>
#include <ESP32Servo.h>
#include <SPI.h>
#include <MFRC522.h>

// WiFi 설정
const char* ssid = "AIE_509_2.4G";
const char* password = "addinedu_class1";
const char* host = "192.168.0.56";
const int port = 9000;

// 핀 설정
const int fsrPin = 34;
const int taxiId = 2;

const int redPin = 25;
const int greenPin = 26;
const int bluePin = 27;
const int leftBlinkerPin = 32;
const int rightBlinkerPin = 33;

// 서보
Servo doorServo;
const int servoPin = 14;
int requestedServoAngle = -1;
int currentServoAngle = -1;
bool servoNeedsReset = false;

// RFID
#define RST_PIN 22
#define SS_PIN 5
MFRC522 mfrc522(SS_PIN, RST_PIN);

// WiFi
WiFiClient client;

// 상태
bool inZeroState = false;
bool eventSent = false;
bool rfidEventSent = false;
unsigned long zeroStartTime = 0;
unsigned long lastReconnectAttempt = 0;
unsigned long lastRFIDCheck = 0;
unsigned long lastBlinkTime = 0;

// Blink
int blinkerMode = 0;

// TCP 메시지 전송 함수
void sendMessage(int taxiId, int eventId, String data = "") {
  char message[15]; // 14문자 + null
  snprintf(message, sizeof(message), "%02d%02d%-10s", taxiId, eventId, data.c_str());  // 👈 9 → 10으로 수정
  client.write((const uint8_t*)message, 14);  // 👈 print() 대신 write() 사용
  Serial.print("📤 전송: ");
  Serial.println(message);
}

void connectToServer() {
  if (client.connect(host, port)) {
    Serial.println("📡 TCP 서버 연결 완료");
  } else {
    Serial.println("❌ TCP 서버 연결 실패");
    lastReconnectAttempt = millis();
  }
}

void resetBlinkers() {
  digitalWrite(leftBlinkerPin, LOW);
  digitalWrite(rightBlinkerPin, LOW);
}

void blinkBlinker() {
  static bool blinkerState = false;
  unsigned long now = millis();
  if (now - lastBlinkTime >= 500) {
    lastBlinkTime = now;
    blinkerState = !blinkerState;
    if (blinkerMode == 6) {
      digitalWrite(leftBlinkerPin, blinkerState ? HIGH : LOW);
      digitalWrite(rightBlinkerPin, LOW);
    }
    else if (blinkerMode == 7){
      digitalWrite(rightBlinkerPin, blinkerState ? HIGH : LOW);
      digitalWrite(leftBlinkerPin, LOW);
    }
    else if (blinkerMode == 8) {
      digitalWrite(leftBlinkerPin, blinkerState ? HIGH : LOW);
      digitalWrite(rightBlinkerPin, blinkerState ? HIGH : LOW);
    }
    else {
      digitalWrite(rightBlinkerPin, LOW);
      digitalWrite(leftBlinkerPin, LOW);
    }
  }
}

void handleEvent(int eventType) {
  switch (eventType) {
    case 3: digitalWrite(redPin, LOW); digitalWrite(greenPin, HIGH); digitalWrite(bluePin, LOW); break;
    case 4: digitalWrite(redPin, HIGH); digitalWrite(greenPin, LOW); digitalWrite(bluePin, LOW); break;
    case 5: digitalWrite(redPin, LOW); digitalWrite(greenPin, LOW); digitalWrite(bluePin, HIGH); break;
    case 6: blinkerMode = 6; break;
    case 7: blinkerMode = 7; break;
    case 8: blinkerMode = 8; break;
    case 9: blinkerMode = 0; resetBlinkers(); break;
    case 10: requestedServoAngle = 90; servoNeedsReset = true; break;
    case 11: requestedServoAngle = 180; servoNeedsReset = true; break;
  }
  

  // 응답 메시지: "이벤트ID + ACK + 공백 패딩"
  char ackMessage[15];
  snprintf(ackMessage, sizeof(ackMessage), "%02d%02dACK        ",taxiId ,eventType);
  client.print(ackMessage);
  Serial.print("📥 이벤트 처리 완료: ");
  Serial.println(ackMessage);
}

void checkIncomingEvent() {
  if (client.available() > 0) {
    char buffer[5] = {0};  // 14바이트 + null 종료
    client.readBytes(buffer, 5);
    String msg = String(buffer);
    Serial.print("📨 문자열 수신: ");
    Serial.println(msg);

    int eventType = msg.substring(2, 4).toInt();  // 0~2: taxiId, 2~4: event
    handleEvent(eventType);
  }
}

void checkRFID() {
  if (millis() - lastRFIDCheck >= 100) {
    lastRFIDCheck = millis();
    if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
      String uid = "";
      for (byte i = 0; i < mfrc522.uid.size; i++) {
        uid += String(mfrc522.uid.uidByte[i] < 0x10 ? "0" : "");
        uid += String(mfrc522.uid.uidByte[i], HEX);
      }
      uid.toUpperCase();

      if (!rfidEventSent) {
        sendMessage(taxiId, 12, uid);
        rfidEventSent = true;
      }
      mfrc522.PICC_HaltA();
    } else {
      rfidEventSent = false;
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(fsrPin, INPUT);
  pinMode(redPin, OUTPUT); pinMode(greenPin, OUTPUT); pinMode(bluePin, OUTPUT);
  pinMode(leftBlinkerPin, OUTPUT); pinMode(rightBlinkerPin, OUTPUT);
  resetBlinkers();

  SPI.begin();
  mfrc522.PCD_Init();

  ESP32PWM::allocateTimer(1);
  doorServo.setPeriodHertz(50);
  doorServo.attach(servoPin, 500, 2400);
  doorServo.write(180); currentServoAngle = 180;

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(500);
  Serial.println("✅ WiFi 연결됨");
  connectToServer();
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) ESP.restart();
  if (!client.connected() && millis() - lastReconnectAttempt >= 10000) connectToServer();

  checkIncomingEvent();
  checkRFID();
  if (blinkerMode) blinkBlinker();

  if (servoNeedsReset && requestedServoAngle != currentServoAngle) {
    doorServo.write(requestedServoAngle);
    currentServoAngle = requestedServoAngle;
    servoNeedsReset = false;
  }

  // FSR 감지
  int fsrValue = analogRead(fsrPin);
  if (fsrValue < 30) {
    if (!inZeroState) { inZeroState = true; zeroStartTime = millis(); eventSent = false; }
    else if (!eventSent && millis() - zeroStartTime >= 2000) {
      sendMessage(taxiId, 2);  // 하차
      eventSent = true;
    }
  } else {
    if (inZeroState) { inZeroState = false; eventSent = false; zeroStartTime = millis(); }
    else if (!eventSent && millis() - zeroStartTime >= 2000) {
      sendMessage(taxiId, 1);  // 승차
      eventSent = true;
    }
  }
}
