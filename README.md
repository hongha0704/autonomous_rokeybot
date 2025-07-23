🚙 자율주행 ROKEYBOT (Simulation)
===
ROKEY B-1조 협동-3 Project (디지털 트윈 기반 서비스 로봇 운영 시스템 구성)
---

### 🔨 개발환경
본 프로젝트는 Ubuntu 22.04 (ROS2 humble) 환경에서 개발되었습니다.   
&nbsp;

### 🚥 작업공간
<img src="https://github.com/user-attachments/assets/ebe70713-0cb6-4f57-ad24-b2c55033ca71" width="75%" height="75%" title="px(픽셀) 크기 설정" alt="project workspace"></img>   
&nbsp;

### 💻 코드 실행

#### **TurtleBot3 Launch**
```bash
ros2 launch turtlebot3_gazebo turtlebot3_autorace_2020.launch.py
```

#### **Camera Calibration**
```bash
ros2 launch turtlebot3_autorace_camera intrinsic_camera_calibration.launch.py
```
```bash
ros2 launch turtlebot3_autorace_camera extrinsic_camera_calibration.launch.py
```

#### **Object Detection**
code: [detect_yolo_sign.py](turtlebot3_autorace/turtlebot3_autorace_detect/turtlebot3_autorace_detect/detect_yolo_sign.py)
```bash
ros2 launch turtlebot3_autorace_detect detect_sign.launch.py mission:=yolo
```

#### **Detect Lane**
code: [detect_lane.py](turtlebot3_autorace/turtlebot3_autorace_detect/turtlebot3_autorace_detect/detect_lane.py)
```bash
ros2 launch turtlebot3_autorace_detect detect_lane.launch.py
```

#### **Control Lane**
code: [control_lane.py](turtlebot3_autorace/turtlebot3_autorace_mission/turtlebot3_autorace_mission/control_lane.py)
```bash
ros2 launch turtlebot3_autorace_mission control_lane.launch.py
```

#### **Detect Traffic Light**
code: [detect_traffic_light.py](turtlebot3_autorace/turtlebot3_autorace_detect/turtlebot3_autorace_detect/detect_traffic_light.py)
```bash
ros2 launch turtlebot3_autorace_detect detect_traffic_light.launch.py
```

&nbsp;

### 📷 시연 영상
https://youtu.be/cOo7qpPeUjg (00:00 ~ 00:21)

---

&nbsp;

## 목차

#### [1. 📘 프로젝트 개요](#1--프로젝트-개요-1)   
#### [2. 👥 프로젝트 팀 구성 및 역할분담](#2--프로젝트-팀-구성-및-역할분담-1)   
#### [3. 🗓 프로젝트 구현 일정](#3--프로젝트-구현-일정-1)   
#### [4. 📌 SKILLS](#4--skills-1)   
#### [5. 🎬 System Flow](#5--system-flow-1)   
#### [6. ✨ 주요 기능](#6--주요-기능-1)   

---

&nbsp;

## 1. 📘 프로젝트 개요
TurtleBot3 Burger를 활용하여 실제 마을 환경을 묘사한 Gazebo 시뮬레이션에서 YOLO 기반 객체 인식과 적응형 주행 제어를 통한 종합적인 자율주행 시스템을 구현합니다.   
교통 표지판 인식, 속도 제어, 보행자 감지, 조명 적응형 차선 인식 등 5가지 핵심 기능을 통해 안전하고 지능적인 자율주행을 실현합니다.

### **핵심 기능**
1. YOLOv11 기반 방향 및 주차 표지판 인식
2. 과속방지턱 및 어린이 보호구역 속도 제어
3. 보행자 인식 및 정지/출발 제어
4. 속도 표지판 인식 및 자동 속도 조절
5. 조명 적응형 차선 인식 성능 향상(HSV 보정)

&nbsp;

## 2. 👥 프로젝트 팀 구성 및 역할분담
|이름|담당 업무|
|--|--|
|백홍하(팀장)| YOLO 커스텀 데이터셋 제작, 터틀봇 속도 제어 로직, 가제보 플러그인 |
|이하빈|차선 탐지 및 경로 로직, 터틀봇 속도 제어 로직|
|장연호|GUI 제작, ROS2 노드 설계 및 launch 파일 작성|
|정찬원|차선 탐지 및 경로 로직, 가제보 플러그인, 데이터 수집 및 전처리|

&nbsp;

## 3. 🗓 프로젝트 구현 일정
**진행 일자: 25.06.09(월) ~ 25.06.13(금) (5일)**
<img src="https://github.com/user-attachments/assets/532b479e-6d50-41f1-adbb-e5ec09ce2c69" width="75%" height="75%" title="px(픽셀) 크기 설정" alt="project_management"></img>

&nbsp;

## 4. 📌 SKILLS
### **Development Environment**
<div align=left>
  
  ![Ubuntu](https://img.shields.io/badge/Ubuntu-E95420?style=for-the-badge&logo=ubuntu&logoColor=white)
  ![Visual Studio Code](https://img.shields.io/badge/Visual%20Studio%20Code-0078d7.svg?style=for-the-badge&logo=visual-studio-code&logoColor=white)
</div>

[![My Skills](https://skillicons.dev/icons?i=ubuntu,vscode&theme=light)](https://skillicons.dev)

### **Robotics**
![ROS](https://img.shields.io/badge/ros-%230A0FF9.svg?style=for-the-badge&logo=ros&logoColor=white)   
[![My Skills](https://skillicons.dev/icons?i=ros&theme=light)](https://skillicons.dev)

### **Programming Languages**
![Python](https://img.shields.io/badge/python-3670A0?style=for-the-badge&logo=python&logoColor=ffdd54)   
[![My Skills](https://skillicons.dev/icons?i=python&theme=light)](https://skillicons.dev)

### **AI & Computer Vision**
<div align=left>
  
  ![OpenCV](https://img.shields.io/badge/opencv-%23white.svg?style=for-the-badge&logo=opencv&logoColor=white)
  ![YOLO](https://img.shields.io/badge/YOLO-111F68?style=for-the-badge&logo=YOLO&logoColor=white)
</div>

[![My Skills](https://skillicons.dev/icons?i=opencv&theme=light)](https://skillicons.dev) 

&nbsp;

## 5. 🎬 System Flow

1. **출발 및 신호 인식**
    - 출발선에서 대기하며 신호등을 인식
    - 초록불이 감지되면 주행을 시작함
2. **방향 표지판 인식 및 경로 결정**
    - 갈림길에서 일정 시간 간격으로 왼쪽/오른쪽 방향 표지판이 번갈아 나타남
    - TurtleBot은 현재 인식된 방향 표지판을 기준으로 해당 방향으로 이동
3. **과속 방지턱 대비**
    - 과속 방지턱 표지판을 인식하면 속도를 감속하여 방지턱 충격을 완화함
4. **어린이 보호구역 감속 주행**
    - 어린이 보호구역 표지판을 인식하면 지정된 저속으로 감속 주행함
5. **주차 표지판 인식 및 주차**
    - 주차 표지판 인식 시, 지정된 주차 구역에 정차
    - 일정 시간 정차 후, 다시 주행을 재개
6. **횡단보도에서 보행자 인식**
    - 횡단보도에 진입 시, 사람을 인식하면 일시 정지
    - 보행자가 지나간 뒤 안전을 확인하고 주행 재개
7. **속도 제한 표지판 인식**
    - 속도 제한 100 표지판 인식 시, 지정된 속도로 가속
    - 속도 제한 30 표지판 인식 시, 감속하여 안전 주행
8. **조명 변화 대응 및 차선 인식 보정**
    - 강한 조명이 도로에 비춰지는 환경을 구성
    - 조도 변화에 따른 차선 인식 정확도 저하에 대비하여 차선 인식 파라미터를 조정
    - 조명 변화에도 안정적인 차선 인식 및 주행이 가능하도록 구현

&nbsp;

## 6. ✨ 주요 기능
### 0. YOLO 커스텀 데이터셋 제작
- YOLO dataset class   
<img src="https://github.com/user-attachments/assets/5f48836e-ef29-4b50-997e-85b277340a2f" width="75%" height="75%" title="px(픽셀) 크기 설정" alt="image"></img>

- 라벨링 진행   
<img src="https://github.com/user-attachments/assets/c759e6c6-6915-46e2-9b71-44ac535445a4" width="75%" height="75%" title="px(픽셀) 크기 설정" alt="image"></img>
<img src="https://github.com/user-attachments/assets/a862e0b6-e9fe-4652-81cb-780e16063ff9" width="75%" height="75%" title="px(픽셀) 크기 설정" alt="image"></img>

- YOLO Object Detection
  - 표지판 인식 후 Publish → control_lane.py, detect_lane.py 에서 표지판 정보를 Subscribe하고 감속, 정지 등 동작 수행   
<img src="https://github.com/user-attachments/assets/ede79cab-3d1e-43ee-add8-724e3933c59d" width="75%" height="75%" title="px(픽셀) 크기 설정" alt="image"></img>

&nbsp;
### 1. 신호등 인식
- 이미지에서 3색 신호등을 각각 마스킹
- 원형 객체 탐지를 통해 신호등 상태를 인식 결과를 이미지에 표시하고 토픽으로 송신
- 특정 색상의 마스크 이미지에서 원형 blob을 검출
- 원형 blob이 있을 경우 해당 색상의 신호등 감지 판단
- 탐지된 신호등 색에 따라 publish 하는 ```twist.linear.x``` 값을 다르게 하여 터틀봇 속도 제어

<img src="https://github.com/user-attachments/assets/f6105cef-fdfe-41e6-a8bf-d4f2d7b51dc5" width="75%" height="75%" title="px(픽셀) 크기 설정" alt="image"></img>
<img width="565" height="71" alt="250613_신호등 터미널" src="https://github.com/user-attachments/assets/9d259d47-df18-4b6a-ac3a-6017d3d16d22" />
<img width="564" height="37" alt="250613_신호등 터미널2" src="https://github.com/user-attachments/assets/6191e820-c288-4de6-bf7a-28402b872a67" />

&nbsp;
### 2. 좌우 표지판 전환 플러그인
- 플러그인을 사용하여 교차로에서 좌우 표지판이 일정 시간 간격으로 전환되도록 구현   
<img src="https://github.com/user-attachments/assets/a9eba8ee-ca2b-4a2d-b1ff-00c1ce5e3cec" width="75%" height="75%" title="px(픽셀) 크기 설정" alt="image"></img>
- 표지판을 인식하면 해당 방향 경로로 이동   
<img src="https://github.com/user-attachments/assets/7bb33dab-5168-46fd-973d-a6d0039797ff" width="75%" height="75%" title="px(픽셀) 크기 설정" alt="image"></img>

| 왼쪽 이동 | 오른쪽 이동 |
|--|--|
| <img width="1063" height="461" alt="250612_follow left" src="https://github.com/user-attachments/assets/1f0f45e1-6d78-4e05-b534-b2efd7aff0a8" /> <img width="543" height="74" alt="250612_터미널 left" src="https://github.com/user-attachments/assets/92ec68e9-bae4-4c32-bb55-dfc3fd1e0bc2" /> | <img width="1061" height="463" alt="250612_follow right" src="https://github.com/user-attachments/assets/4a3a575b-7ff7-4dca-ac57-a7c9b2def2a1" /> <img width="543" height="74" alt="250612_터미널 right" src="https://github.com/user-attachments/assets/9f504ccd-2c93-4f38-b25d-69706b6e4b12" /> |

&nbsp;
### 3. 과속 방지턱 - 감속
- 과속 방지턱 가까이서 인식 시, 10초 간 터틀봇에 낮은 ```twist.linear.x``` 값을 퍼블리시 하여 감속
- 이후 다시 정상 속도로 주행   

<img src="https://github.com/user-attachments/assets/11055a9a-e424-40e0-9fb9-77d68da2459a" width="75%" height="75%" title="px(픽셀) 크기 설정" alt="image"></img>
<img width="563" height="71" alt="250612_과속방지턱 터미널" src="https://github.com/user-attachments/assets/981dc3d2-24e5-4930-b885-3dd903f614d1" />

&nbsp;
### 4. 어린이 보호구역 - 감속
- 어린이 보호구역 가까이서 인식 시, 10초 간 터틀봇에 낮은 twist.linear.x 값을 퍼블리시 하여 감속
- 이후 다시 정상 속도로 주행

<img src="https://github.com/user-attachments/assets/68443d70-373d-44fe-ade6-ee7afc2762fb" width="75%" height="75%" title="px(픽셀) 크기 설정" alt="image"></img>
<img width="557" height="72" alt="250613_school zone 터미널" src="https://github.com/user-attachments/assets/e3365701-168d-4a85-b6bf-3a1894d6167e" />

&nbsp;
### 5. 횡단보도 - 일시 정지
- 횡단보도와 보행자 동시에 인식 시, 10초 간 터틀봇에 ```twist.linear.x = 0``` 값을 퍼블리시 하여 정지
- 이후 다시 정상 속도로 주행

<img src="https://github.com/user-attachments/assets/3b88c067-6bda-4eed-8db0-7d61dfb941d0" width="75%" height="75%" title="px(픽셀) 크기 설정" alt="image"></img>
<img width="560" height="74" alt="250613_횡단보도 터미널" src="https://github.com/user-attachments/assets/64ef59d8-1a12-4643-9693-daa366d8f92d" />

&nbsp;
### 6. 속도 제한 표지판 100 - 가속
- 속도 제한 표지판 100 인식 시, 터틀봇에 높은 ```twist.linear.x``` 값을 퍼블리시 하여 가속

<img src="https://github.com/user-attachments/assets/b67ff2f2-2a59-43fe-bd29-765df2b9ab34" width="75%" height="75%" title="px(픽셀) 크기 설정" alt="image"></img>
<img width="559" height="72" alt="250613_속도 100 터미널" src="https://github.com/user-attachments/assets/e2d0af0f-c9e0-483e-8008-83fd25c65c67" />

&nbsp;
### 7. 속도 제한 표지판 30 - 감속
- 속도 제한 표지판 30 인식 시, 터틀봇에 낮은 ```twist.linear.x``` 값을 퍼블리시 하여 감속

<img src="https://github.com/user-attachments/assets/f87fa3b9-4294-4415-931d-8fab17b30594" width="75%" height="75%" title="px(픽셀) 크기 설정" alt="image"></img>
<img width="559" height="90" alt="250613_속도 30 터미널" src="https://github.com/user-attachments/assets/ee0b2348-f676-437a-8294-2f89403eaf56" />
