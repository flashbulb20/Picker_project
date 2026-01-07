# Picker project

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue) ![Python](https://img.shields.io/badge/Python-3.10-yellow) ![Turtlebot4](https://img.shields.io/badge/Platform-Turtlebot4-green) ![YOLO](https://img.shields.io/badge/AI-YOLOv8-red)

## 📖 프로젝트 개요
이 프로젝트는 **2대의 Turtlebot4 로봇**이 협업하여 물류 창고 내의 박스 재고를 자율적으로 파악하고 관리하는 시스템입니다.  
Nav2를 이용한 자율 주행, YOLOv8 기반의 객체 인식, 그리고 로봇 간 충돌 방지를 위한 **Smart Traffic Control** 시스템이 통합되어 있습니다.

## 🚀 주요 기능 (Key Features)

### 1. 🤖 멀티 로봇 협업 및 교통 정리 (Smart Traffic Control)
- 좁은 통로에서 로봇 간 충돌을 방지하기 위해 **이중 확인(Dual Check)** 로직을 적용했습니다.
- **Phase Sharing:** 각 로봇은 자신의 작업 단계(Phase 1~5)를 `/current_phase` 토픽으로 공유합니다.
- **ROI Occupancy:** 작업 구역(ROI)에 진입하면 `/ROI_robot_detected` (Bool) 신호를 송출합니다.
- **Wait Logic:** 후행 로봇은 선행 로봇이 작업 중일 경우, 지정된 **대기 장소**로 이동하여 안전거리를 확보하고 10초간 대기합니다.

### 2. 📦 딥러닝 기반 재고 파악
- **YOLOv8n** 모델을 커스텀 학습시켜 박스(Box)를 실시간으로 탐지합니다.
- OAK-D Lite 카메라를 통해 취득한 이미지를 분석하고, **Phase 3** 단계에서 정지 상태로 정확한 개수를 카운트합니다.
- 중복 전송 및 음수 방지 로직(`max(0, count - 1)`)이 적용된 데이터를 서버로 전송합니다.

### 3. 🛡️ Lidar 기반 안전 가드 (Safety Monitor)
- Nav2의 Costmap과는 별개로, Raw Lidar 데이터를 직접 구독하여 긴급 상황에 대처합니다.
- **Emergency Stop:** 전방 30도, 0.4m 이내 장애물 감지 시 즉시 정지.
- **Active Avoidance:** 정지 후 전방 100도 범위를 스캔하여 더 넓은 공간으로 회피 기동을 수행합니다.

### 4. 🔊 청각적 피드백 및 자동화
- 미션 완료(Phase 4 도착) 시 로봇에서 **"삐뽀삐뽀"** 알림음을 3초간 재생하여 작업자에게 알립니다.
- 작업 완료 후 자동으로 도킹 스테이션으로 복귀하여 충전을 시작합니다.

---

## 🔄 미션 수행 프로세스 (Phase System)

| 단계 | Phase Name | 설명 |
|:---:|:---:|:---|
| **1** | **Entry** | 대기 장소에서 창고 진입로로 이동 (Nav2) |
| **2** | **Approach** | 박스 적재 위치 바로 앞으로 정밀 접근 (Nudge Control) |
| **3** | **Detection** | 정지 후 YOLO를 통해 박스 개수 카운트 & ROI 점유 신호 발신 |
| **4** | **Delivery** | 데이터 전송 위치로 이동 후 DB 업데이트 & 알림음 재생 |
| **5** | **Return** | 도킹 스테이션으로 복귀 및 자동 도킹 (Auto Docking) |

---

## 🛠️ 기술적 해결 사례 (Troubleshooting)

개발 과정에서 발생한 주요 이슈와 해결 방법입니다.

### 1. Nav2 "Ghost Arrival" 현상
* **문제:** 이동 명령(`goToPose`) 직후 로봇이 움직이지 않았는데도 '도착 성공'으로 처리되는 현상.
* **해결:** 명령 전송 직후 `time.sleep(1.5)`를 강제로 부여하여 Nav2 상태가 초기화될 시간을 확보하고, `distance_remaining`을 이중 체크하는 로직 추가.

### 2. Initial Pose 오차 문제
* **문제:** 하드코딩된 초기 좌표와 실제 맵 간의 미세한 오차로 인해 주행 시작 시 로봇이 회전하거나 경로를 이탈함.
* **해결:** `setInitialPose` 강제 설정을 제거하고, **"도킹 스테이션 = (0,0) = North"**라는 물리적 기준을 활용하여 `Undock` 후 바로 주행하도록 로직 개선.

---

## 📦 설치 및 실행 방법

### 1. 의존성 패키지 설치
```bash
sudo apt install ros-humble-turtlebot4-navigation \
                 ros-humble-nav2-simple-commander \
                 ros-humble-irobot-create-msgs
pip install ultralytics opencv-python
```
### 2. 패키지 빌드
```bash
cd ~/rokey_ws
colcon build --symlink-install --packages-select Picker_project
source install/setup.bash
```

### 3. 실행
```bash
# 로봇 2대 각각 터미널에서 실행 
ros2 run Picker_project picker_main
```

---

## 📝 License
