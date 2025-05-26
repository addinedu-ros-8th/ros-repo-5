#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

// WiFi Configuration (Connect to your router)
const char* ssid = "esp32";           // your WiFi SSID
const char* password = "12345678";    // your WiFi password

AsyncWebServer server(80);

// Traffic light structure
struct TrafficLight {
  int greenPin;
  int yellowPin;
  int redPin;
};

TrafficLight trafficLights[4] = {
  {27, 26, 25}, // Set 1
  {12, 13, 14}, // Set 2
  {21, 22, 23}, // Set 3
  {15, 16, 17}  // Set 4
};

// State and control variables
bool autoMode = true;
unsigned long lastChangeTime = 0;

enum Phase { GREEN_12, YELLOW_12, GREEN_34, YELLOW_34 };
Phase currentPhase = GREEN_12;

const unsigned long GREEN_DURATION = 10000;   // 10 seconds
const unsigned long YELLOW_DURATION = 2000;   // 2 seconds

// Light control functions
void setLight(int index, int red, int yellow, int green) {
  digitalWrite(trafficLights[index].redPin, red);
  digitalWrite(trafficLights[index].yellowPin, yellow);
  digitalWrite(trafficLights[index].greenPin, green);
}

void setGreenGroup12() {
  setLight(0, LOW, LOW, HIGH);
  setLight(1, LOW, LOW, HIGH);
  setLight(2, HIGH, LOW, LOW);
  setLight(3, HIGH, LOW, LOW);
}

void setYellowGroup12() {
  setLight(0, LOW, HIGH, LOW);
  setLight(1, LOW, HIGH, LOW);
  setLight(2, HIGH, LOW, LOW);
  setLight(3, HIGH, LOW, LOW);
}

void setGreenGroup34() {
  setLight(0, HIGH, LOW, LOW);
  setLight(1, HIGH, LOW, LOW);
  setLight(2, LOW, LOW, HIGH);
  setLight(3, LOW, LOW, HIGH);
}

void setYellowGroup34() {
  setLight(0, HIGH, LOW, LOW);
  setLight(1, HIGH, LOW, LOW);
  setLight(2, LOW, HIGH, LOW);
  setLight(3, LOW, HIGH, LOW);
}

// Minimal HTML UI (English)
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Traffic Light</title></head><body>
<h3>Traffic Light Control</h3>
<button onclick="location.href='/green12'">Set 1/2 Green</button><br><br>
<button onclick="location.href='/green34'">Set 3/4 Green</button><br><br>
<button onclick="location.href='/auto'">Auto Mode</button><br><br>
<p>Current mode: <b>%MODE%</b></p>
</body></html>
)rawliteral";

// Placeholder replacement
String processor(const String& var) {
  if (var == "MODE") {
    return autoMode ? "Auto" : "Manual";
  }
  return "";
}

void setup() {
  Serial.begin(115200);

  // Pin initialization
  for (int i = 0; i < 4; i++) {
    pinMode(trafficLights[i].greenPin, OUTPUT);
    pinMode(trafficLights[i].yellowPin, OUTPUT);
    pinMode(trafficLights[i].redPin, OUTPUT);
  }
  setGreenGroup12();

  // WiFi connection
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected! IP Address: ");
  Serial.println(WiFi.localIP());

  // Web routes
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html, processor);
  });

  server.on("/green12", HTTP_GET, [](AsyncWebServerRequest *request){
    autoMode = false;
    setGreenGroup12();
    request->redirect("/");
  });

  server.on("/green34", HTTP_GET, [](AsyncWebServerRequest *request){
    autoMode = false;
    setGreenGroup34();
    request->redirect("/");
  });

  server.on("/auto", HTTP_GET, [](AsyncWebServerRequest *request){
    autoMode = true;
    lastChangeTime = millis();
    request->redirect("/");
  });

  server.begin();
  lastChangeTime = millis();
}

void loop() {
  if (!autoMode) return;

  unsigned long now = millis();
  unsigned long duration = (currentPhase % 2 == 0) ? GREEN_DURATION : YELLOW_DURATION;
  if (now - lastChangeTime < duration) return;

  switch (currentPhase) {
    case GREEN_12: setYellowGroup12(); currentPhase = YELLOW_12; break;
    case YELLOW_12: setGreenGroup34(); currentPhase = GREEN_34; break;
    case GREEN_34: setYellowGroup34(); currentPhase = YELLOW_34; break;
    case YELLOW_34: setGreenGroup12(); currentPhase = GREEN_12; break;
  }
  lastChangeTime = now;
}
