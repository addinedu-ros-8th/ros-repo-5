
![ChatGPT Image May 26, 2025, 12_52_15 PM](https://github.com/user-attachments/assets/8262538f-89b0-4234-9d2d-c3a225c73910)


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

![img_0598_720](https://github.com/user-attachments/assets/dcc011fb-e212-46aa-b757-7aa63b7eb8c5)

| 이름      |직책            | 역할                                                              |
|----------|--------------|----------------------------------------------------------------------|
|송원준     |  팀장         | 프로젝트 관리, AI 서버, 주행 알고리즘   |
| 김규환    |  부팀장       | Controll 서버, 택시 하드웨어 개발     |
| 임승연 | 팀원       | User GUI 설계 및 구현, 문서 작성           |
| 권빛  | 팀원      |Admin GUI 설계 및 구현, PPT 제작       |





## 주요 기능 


| 분류       | 주요 기능 항목 |
|------------|----------------|
| **주행**    | - 주행 기능<br>- 장애물 인식 기능<br>- 신호 대응 기능<br>- 횡단보도 대응 기능<br>- 표지판 인식 기능 |
| **택시 서비스** | - 택시 호출<br>- 택시 배차<br>- 택시 상태 모니터링<br>- 승객 탑승 확인<br>- 결제 기능<br>- 목적지 이동<br>- 하차 확인 |
| **관제**    | - 로그인 기능<br>- 실시간 모니터링<br>- 경로 조회 기능<br>- 주행 이벤트 조회<br>- 운행 기록 조회<br>- 통계 기능 |






## 맵 구성 
![resized_new_map_image](https://github.com/user-attachments/assets/83374665-4ebb-4d8b-8f70-051cb18f922a)






## 시스템 아키텍처

![image](https://github.com/user-attachments/assets/92708ccb-ac1b-48c2-bd47-dd7078b3ebec)



### SW 


![image](https://github.com/user-attachments/assets/78cee545-554b-4b81-a23f-ba409a018311)


### HW 


![image](https://github.com/user-attachments/assets/07c17994-c21f-47a7-bcdf-2979d1ece7a3)


###  ER Diagram

![image](https://github.com/user-attachments/assets/da6ef441-e304-48df-93ff-41911cb7e79d)



### State Diagram

![image](https://github.com/user-attachments/assets/ca1c4f47-835c-474c-9f4d-1099546a6280)


### Sequence Diagram


#### 배차 성공


![image](https://github.com/user-attachments/assets/5378cadc-bae1-491f-9b2d-605c61af8c19)



#### 배차 실패


![image](https://github.com/user-attachments/assets/79d5483a-8535-4126-bfa9-1f90f7d58f59)


#### 운행


![image](https://github.com/user-attachments/assets/01600b19-2cca-4052-bf67-6c62983b1302)




## GUI


### USER GUI 






## 시연 영상 








------






