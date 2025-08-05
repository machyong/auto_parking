# auto_parking
예시 입니다
## 📝 프로젝트 개요

이 프로젝트는 TurtleBot3 Burger 사방에 카메라를 장착하여 다양한 방향에서의 시각 정보를 활용하여 강화학습 시뮬레이션 환경을 구성합니다.  
Gazebo 기반 autorace 맵을 사용하며 주차에 대한 강화학습을 진행 할 예정압니다. 

---

## 📁 브랜치 설명

| 브랜치명 | 설명 |
|----------|------|
| `main` | 기본 TurtleBot3 및 autorace 환경 |
| `rear_camera_parkingline_detector` | 후방 카메라의 이미지를 필터링하여 주차라인 도착 여부 확인 |

---

## 📌 사용된 주요 ROS2 패키지
- `turtlebot3_gazebo`
- `gazebo_ros`
- `image_transport`
- `tf2_ros
- `sensor_msgs`

## 🔧 Version History

### 🚀 v0.0.01 - Gazebo Simulation
- turtlebot3 gazebo 환경 가져오기
- turtlebot3 gazebo 실행 테스트 완료

### 🚀 v0.0.02 - Gazebo Simulation
- turtlebot3 burgur rear camera 추가 및 TF 프레임 설정
- rear/image_raw, rear/camera_info 토픽 발행 확인
- rear_camera_link 기준 pose 및 카메라 각도 튜닝
- 맵 주차장 이미지, 크기 변경
- 주차장에 차량을 대신할 블록 설치
- autorace 맵에서 rear camera 작동 테스트 완료
  
### 🚀 v0.0.03 - Machine learning
- machine learning 환경 가져오기
- dqn 파일 일부 실행 테스트 진행
  
### 🚀 v0.0.04 - Burger Model
- left, right camear 추가 및 TF 프레임 설정
- left_camera/image_raw, left_camera/camera_info 토픽 발행 확인 (right_camera 또한 동일)
- left_camera_link 기준 pose 및 카메라 각도 튜닝 (right_camera 또한 동일)
  
### 🚀 v0.0.05 - Camera Integration
- 노란선 필터링을 위한 초기 Line Detector 노드 테스트 진행
- ROI 설정을 통한 아래 화면의 80% 사용 범위 설정
- 노란선 비율 0.6 이상일 시 노란선 인식 토픽 발행 (서비스 서버로 수정 예정)
> 브랜치: `main/rear_camera_parking_detector`

## ▶️ 실행 방법

```bash
# Launch Gazebo with rear camera
ros2 launch turtlebot3_gazebo turtlebot3_autorace_2020.launch.py

# Run line detector node (detect yellow line from rear camera)
ros2 run turtlebot3_dqn line_detector_node

