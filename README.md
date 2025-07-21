🚙 자율주행 ROKEYBOT (Simulation)
===
ROKEY B-1조 협동-3 Project (디지털 트윈 기반 서비스 로봇 운영 시스템 구성)
---

### 🔨 개발환경
본 프로젝트는 Ubuntu 22.04 (ROS2 humble) 환경에서 개발되었습니다.   
&nbsp;

### 🦾 작업공간
<img src="rokey_project/image/workspace/IMG_3175.jpg" width="50%" height="50%" title="px(픽셀) 크기 설정" alt="project_management"></img>   
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
https://youtu.be/FMOeqKwD2Ls

---

&nbsp;

## 목차

#### [1. 📘 프로젝트 개요](#1--프로젝트-개요-1)   
#### [2. 👥 프로젝트 팀 구성 및 역할분담](#2--프로젝트-팀-구성-및-역할분담-1)   
#### [3. 🗓 프로젝트 구현 일정](#3--프로젝트-구현-일정-1)   
#### [4. 📌 SKILLS](#4--skills-1)   
#### [5. 🎬 System Flow](#5--system-flow-1)   

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
<img src="rokey_project/image/notion/250717_project_management.png" width="100%" height="100%" title="px(픽셀) 크기 설정" alt="project_management"></img>

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
  
  ![PyTorch](https://img.shields.io/badge/PyTorch-%23EE4C2C.svg?style=for-the-badge&logo=PyTorch&logoColor=white)
  ![OpenCV](https://img.shields.io/badge/opencv-%23white.svg?style=for-the-badge&logo=opencv&logoColor=white)
</div>

[![My Skills](https://skillicons.dev/icons?i=pytorch,opencv&theme=light)](https://skillicons.dev) 

&nbsp;

## 5. 🎬 System Flow
<img src="rokey_project/image/system_flow/ROKEY_Pharmacy_detail.drawio.png" width="75%" height="75%" title="px(픽셀) 크기 설정" alt="system_flow"></img>

&nbsp;
