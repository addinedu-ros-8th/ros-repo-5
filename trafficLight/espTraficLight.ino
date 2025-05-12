struct TrafficLight {
    int greenPin;
    int yellowPin;
    int redPin;
  };
  
  TrafficLight trafficLights[4] = {
    {27, 26, 25},  // 세트 1
    {12, 13, 14},  // 세트 2
    {21, 22, 23},  // 세트 3
    {15, 16, 17}   // 세트 4
  };
  
  // 타이밍 변수
  const unsigned long GREEN_DURATION = 5000;   // 초록 지속 시간
  const unsigned long YELLOW_DURATION = 2000;  // 노랑 지속 시간
  
  enum Phase {
    GREEN_12,
    YELLOW_12,
    GREEN_34,
    YELLOW_34
  };
  
  Phase currentPhase = GREEN_12;
  unsigned long lastChangeTime = 0;
  
  void setup() {
    for (int i = 0; i < 4; i++) {
      pinMode(trafficLights[i].greenPin, OUTPUT);
      pinMode(trafficLights[i].yellowPin, OUTPUT);
      pinMode(trafficLights[i].redPin, OUTPUT);
    }
  
    setGreenGroup12();  // 초기 상태
    lastChangeTime = millis();
  }
  
  void loop() {
    unsigned long now = millis();
  
    switch (currentPhase) {
      case GREEN_12:
        if (now - lastChangeTime >= GREEN_DURATION) {
          setYellowGroup12();
          currentPhase = YELLOW_12;
          lastChangeTime = now;
        }
        break;
  
      case YELLOW_12:
        if (now - lastChangeTime >= YELLOW_DURATION) {
          setGreenGroup34();
          currentPhase = GREEN_34;
          lastChangeTime = now;
        }
        break;
  
      case GREEN_34:
        if (now - lastChangeTime >= GREEN_DURATION) {
          setYellowGroup34();
          currentPhase = YELLOW_34;
          lastChangeTime = now;
        }
        break;
  
      case YELLOW_34:
        if (now - lastChangeTime >= YELLOW_DURATION) {
          setGreenGroup12();
          currentPhase = GREEN_12;
          lastChangeTime = now;
        }
        break;
    }
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
  
  void setLight(int index, int red, int yellow, int green) {
    digitalWrite(trafficLights[index].redPin, red);
    digitalWrite(trafficLights[index].yellowPin, yellow);
    digitalWrite(trafficLights[index].greenPin, green);
  }
  