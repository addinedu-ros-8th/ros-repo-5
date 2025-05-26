
![ChatGPT Image May 26, 2025, 12_52_15 PM (2)](https://github.com/user-attachments/assets/c25695b5-e296-4e4e-bf4c-d24439e6fe5e)


# **Autonomous System: ADDIN 🚖 TAXI**



**자율주행 로봇** 택시 프로젝트 **ADDIN TAXI**



## 실행 환경 



### 개발 환경 및 사용 기술

| 분류                   | 내용                                                                 |
|------------------------|----------------------------------------------------------------------|
| 개발 환경 / 언어       | Ubuntu 24.04, ROS2 (Jazzy), Python, C++                              |
| 네트워크               | TCP/IP, ROS2, REST API, UDP                                          |
| 개발 기술 및 라이브러리 | YOLO, ROS2, ArUco Marker, PyQt6, OpenCV                              |
| 하드웨어               | Raspberry Pi 5, ESP32                                                |
| 협업 도구              | GitHub, Confluence, Jira, Slack                                      |



### 실행 


```bash
git clone https://github.com/addinedu-ros-8th/ros-repo-5.git
cd ros-repo-5
```



## 팀원 구성 



| 이름      |직책            | 역할                                                              |
|----------|--------------|----------------------------------------------------------------------|
|**송원준**     |  팀장         | 프로젝트 관리, AI 서버, 주행 알고리즘   |
|**김규환**    |  부팀장       | Controll 서버, 택시 하드웨어 개발     |
| **임승연** | 팀원       | GUI 설계 및 구현, 문서 작성           |
| **권빛**  | 팀원      |Admin GUI 설계 및 구현, PPT 제작       |





## 주요 기능 


| 분류       | 주요 기능 항목 |
|------------|----------------|
| **주행**    | - 주행 기능<br>- 장애물 인식 기능<br>- 신호 대응 기능<br>- 횡단보도 대응 기능<br>- 표지판 인식 기능 |
| **택시 서비스** | - 택시 호출<br>- 택시 배차<br>- 택시 상태 모니터링<br>- 승객 탑승 확인<br>- 결제 기능<br>- 목적지 이동<br>- 하차 확인 |
| **관제**    | - 로그인 기능<br>- 실시간 모니터링<br>- 경로 조회 기능<br>- 주행 이벤트 조회<br>- 운행 기록 조회<br>- 통계 기능 |






## 맵 구성 
![resized_new_map_image](https://github.com/user-attachments/assets/83374665-4ebb-4d8b-8f70-051cb18f922a)


## 하드웨어 구성


![image](https://github.com/user-attachments/assets/03cc7b37-cff1-4342-962a-4808ec283f3c)






## 시스템 아키텍처

![image](https://github.com/user-attachments/assets/92708ccb-ac1b-48c2-bd47-dd7078b3ebec)



###  ER Diagram

![image](https://github.com/user-attachments/assets/da6ef441-e304-48df-93ff-41911cb7e79d)



### State Diagram

![image](https://github.com/user-attachments/assets/15ec0a18-9d41-436a-becf-59848c2e5cb2)


### Scenario
![image](https://github.com/user-attachments/assets/365a0f81-5276-4045-b227-e21132cb264b)



## 딥러닝 자율 주행 기술 

### 차선 감지 모델 
![image](https://github.com/user-attachments/assets/4745f14b-aa7f-4d70-a0f9-a8142c6a4c42)


* **YOLOv8 Segmantation**
* 도로 속 흰색 실선, 중앙선, 흰색 점선 탐지
* 차선 인식 후 중앙점 기준으로 주행 


### 도로 객체 탐지 기능
![image](https://github.com/user-attachments/assets/055e1bd1-aa85-468d-87ac-d94e10864da7)


* **YOLOv8 detection** 활용
* 도로 속 정지선, 횡단보도 탐지
* 맵 위 신호등 표지판, 보행자 탐지 후 정지

 
### 주행 경로 탐색 알고리즘 
![image_720](https://github.com/user-attachments/assets/f7d6882e-94cb-436e-91e8-0e69036ddf39)


1. 맵 **Waypoint** 생성 후 **노드화** 및 연결
2. 천장의 웹캠이 보는 맵 속 **노드의 좌표** 생성
3. 택시 상단에 **AruCo 마커** 부착
4. 실시간으로 노드와 택시의 좌표를 송수신하며 경로 탐색
5. **A*알고리즘**으로 **최적경로** 탐색 



### 택시 배차 할당 방식 
#### **승객 인원**
**5명 이상** -> 승합차 
**4명 이하** -> 승용차 


#### 로봇 상태 
**배터리 > 60%** 


#### 거리 기반
출발지 **최근접** 택시



## 시연 

### 도로주행
![20250526_100820 (2) (1) (1)](https://github.com/user-attachments/assets/57b0e955-eec1-4318-acdf-24ab07268314)



## 전체 시나리오 

### 1. 로그인 
![image_720](https://github.com/user-attachments/assets/7f16303d-f904-47b1-a2f0-b71405cd185a)




### 2. 배차 




#### User GUI 



#### Admin GUI 






### 3. 승차 




#### User GUI 



#### Admin GUI 




### 4. 운행 


#### User GUI 


#### Admin GUI 



### 5. 하차 


#### User GUI 



#### Admin GUI 



------

## 레퍼런스 

### pinky_violet
```
https://github.com/pinklab-art/pinky_violet
```






------






