#include <WiFi.h>
#include <ESP32Servo.h>
#include <SPI.h>
#include <MFRC522.h>
#include <deque>

// WiFi 설정
// const char* ssid = "pinky_d7f5"; //taxi2
//const char* ssid = "pinky_d780"; //taxi1
// const char* password = "pinkyros2";
// const char* host = "192.168.4.1";

const char* ssid = "esp32";
const char* password = "12345678";

// const char* host = "192.168.1.10";
// const int vehicleId = 1;

const char* host = "192.168.1.6";
const int vehicleId = 2;

const int port = 9000;

// 핀 설정
const int fsrPin = 34;
const int redPin = 25, greenPin = 26, bluePin = 27;
const int leftBlinkerPin = 32, rightBlinkerPin = 33;
const int servoPin = 14;

#define RST_PIN 22
#define SS_PIN 5
MFRC522 mfrc522(SS_PIN, RST_PIN);

WiFiClient client;
Servo doorServo;

// 상태 변수
bool inZeroState = false, eventSent = false, rfidEventSent = false;
unsigned long zeroStartTime = 0, lastReconnectAttempt = 0, lastRFIDCheck = 0, lastBlinkTime = 0;
int blinkerMode = 0;
int requestedServoAngle = -1, currentServoAngle = -1;
bool servoNeedsReset = false;

struct AckEntry {
  String message;
  int retries;
  unsigned long lastSent;
};
std::deque<AckEntry> ackQueue;

void resetBlinkers() {
  digitalWrite(leftBlinkerPin, LOW);
  digitalWrite(rightBlinkerPin, LOW);
}

void blinkBlinker() {
  static bool blinkerState = false;
  if (millis() - lastBlinkTime >= 500) {
    lastBlinkTime = millis();
    blinkerState = !blinkerState;
    if (blinkerMode == 6) digitalWrite(leftBlinkerPin, blinkerState ? HIGH : LOW);
    else if (blinkerMode == 7) digitalWrite(rightBlinkerPin, blinkerState ? HIGH : LOW);
    else if (blinkerMode == 8) {
      digitalWrite(leftBlinkerPin, blinkerState ? HIGH : LOW);
      digitalWrite(rightBlinkerPin, blinkerState ? HIGH : LOW);
    } else resetBlinkers();
  }
}

void processCommandFromPi(int eventType) {
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
  char ackMessage[13];
  snprintf(ackMessage, sizeof(ackMessage), "%02d%02dACK     ", vehicleId, eventType);
  client.write((const uint8_t*)ackMessage, 12);
  Serial.printf("📥 ACK 응답 전송: '%s'\n", ackMessage);
}

void checkIncomingMessageFromPi() {
  if (client.available()) {
    char buffer[13] = {0};
    int len = client.readBytes(buffer, 12);
    if (len >= 4) {
      String msg = String(buffer);
      Serial.printf("📨 수신: '%s'\n", msg.c_str());
      if (msg.indexOf("ACK") != -1) {
        if (!ackQueue.empty() && msg.startsWith(ackQueue.front().message.substring(0, 4))) {
          Serial.println("✅ ACK 수신 - 큐에서 제거됨");
          ackQueue.pop_front();
        }
      } else {
        int eventType = msg.substring(2, 4).toInt();
        processCommandFromPi(eventType);
      }
    }
  }
}

void resendPendingEventToPi() {
  if (!client.connected()) return;
  if (!ackQueue.empty()) {
    AckEntry& front = ackQueue.front();
    if (millis() - front.lastSent >= 3000) {
      if (front.retries < 3) {
        client.write((const uint8_t*)front.message.c_str(), 12);
        front.lastSent = millis();
        front.retries++;
        Serial.printf("🔁 재전송: '%s'\n", front.message.c_str());
      } else {
        Serial.println("⛔ 재시도 초과, 제거됨");
        ackQueue.pop_front();
      }
    }
  }
}

void sendEventToPi(int vehicleId, int eventId, String data = "") {
  char message[13];
  snprintf(message, sizeof(message), "%02d%02d%-8s", vehicleId, eventId, data.c_str());
  client.write((const uint8_t*)message, 12);
  Serial.printf("📤 전송: '%s'\n", message);
  AckEntry entry = { String(message), 0, millis() };
  ackQueue.push_back(entry);
}

void checkRFID() {
  if (millis() - lastRFIDCheck >= 100) {
    lastRFIDCheck = millis();
    if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
      String uid = "";
      for (byte i = 0; i < mfrc522.uid.size; i++) {
        uid += (mfrc522.uid.uidByte[i] < 0x10 ? "0" : "") + String(mfrc522.uid.uidByte[i], HEX);
      }
      uid.toUpperCase();
      if (!rfidEventSent) {
        sendEventToPi(vehicleId, 12, uid);
        rfidEventSent = true;
      }
      mfrc522.PICC_HaltA();
    } else {
      rfidEventSent = false;
    }
  }
}

void connectToServer() {
  if (client.connect(host, port)) {
    Serial.println("📡 TCP 서버 연결 완료");
  } else {
    Serial.println("❌ TCP 서버 연결 실패");
    lastReconnectAttempt = millis();
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("🚀 ESP32 시작");

  pinMode(fsrPin, INPUT);
  pinMode(redPin, OUTPUT); pinMode(greenPin, OUTPUT); pinMode(bluePin, OUTPUT);
  pinMode(leftBlinkerPin, OUTPUT); pinMode(rightBlinkerPin, OUTPUT);
  resetBlinkers();

  SPI.begin();
  mfrc522.PCD_Init();

  ESP32PWM::allocateTimer(1);
  doorServo.setPeriodHertz(50);
  doorServo.attach(servoPin, 500, 2400);
  doorServo.write(180);
  currentServoAngle = 180;

  Serial.printf("📡 WiFi 연결 시도: SSID='%s'\n", ssid);
  WiFi.begin(ssid, password);

  int retry = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (++retry > 20) {
      Serial.println("❌ WiFi 연결 실패 (20초 초과)");
      return;
    }
  }

  Serial.println();
  Serial.println("✅ WiFi 연결 성공");
  Serial.println(WiFi.localIP());

  connectToServer();
}


void loop() {
  if (WiFi.status() != WL_CONNECTED) ESP.restart();
  if (!client.connected() && millis() - lastReconnectAttempt >= 10000) connectToServer();

  checkIncomingMessageFromPi();
  checkRFID();
  resendPendingEventToPi();
  if (blinkerMode) blinkBlinker();

  if (servoNeedsReset && requestedServoAngle != currentServoAngle) {
    doorServo.write(requestedServoAngle);
    currentServoAngle = requestedServoAngle;
    servoNeedsReset = false;
  }

  int fsrValue = analogRead(fsrPin);
  if (fsrValue < 30) {
    if (!inZeroState) { inZeroState = true; zeroStartTime = millis(); eventSent = false; }
    else if (!eventSent && millis() - zeroStartTime >= 2000) {
      sendEventToPi(vehicleId, 2); eventSent = true;
    }
  } else {
    if (inZeroState) { inZeroState = false; eventSent = false; zeroStartTime = millis(); }
    else if (!eventSent && millis() - zeroStartTime >= 2000) {
      sendEventToPi(vehicleId, 1); eventSent = true;
    }
  }
}