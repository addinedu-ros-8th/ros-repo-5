#include <WiFi.h>
#include <WebServer.h>

#define RED_PIN 16
#define YELLOW_PIN 17
#define GREEN_PIN 18
#define STATUS_LED 2  // 보통 내장 LED

const char* ssid = "AIE_509_2.4G";       // Wi-Fi SSID
const char* password = "addinedu_class1"; // Wi-Fi 비밀번호

WebServer server(80);

void handleRoot() {
  server.send(200, "text/plain", "ESP32 Traffic Light Ready");
}

void handleCommand() {
  String cmd = server.arg("cmd");

  // 모든 불 OFF
  digitalWrite(RED_PIN, LOW);
  digitalWrite(YELLOW_PIN, LOW);
  digitalWrite(GREEN_PIN, LOW);

  // 선택된 불 ON
  if (cmd == "R") digitalWrite(RED_PIN, HIGH);
  else if (cmd == "Y") digitalWrite(YELLOW_PIN, HIGH);
  else if (cmd == "G") digitalWrite(GREEN_PIN, HIGH);

  server.send(200, "text/plain", "OK");
}

void setup() {
  Serial.begin(115200);

  // 핀 모드 설정
  pinMode(RED_PIN, OUTPUT);
  pinMode(YELLOW_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(STATUS_LED, OUTPUT);

  // Wi-Fi 연결 시도
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(STATUS_LED, !digitalRead(STATUS_LED)); // 깜빡임
    delay(500);
  }

  // Wi-Fi 연결 성공
  digitalWrite(STATUS_LED, HIGH); 
  Serial.println(WiFi.localIP());

  // 서버 설정
  server.on("/", handleRoot);
  server.on("/command", handleCommand);
  server.begin();
}

void loop() {
  server.handleClient();
}

