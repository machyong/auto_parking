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
| `parking_area_detect` | 우측 카메라의 이미지를 필터링하여 주차 가능구역 탐색 |

---

## 📌 사용된 주요 ROS2 패키지
- `turtlebot3_gazebo`
- `gazebo_ros`
- `image_transport`
- `tf2_ros
- `sensor_msgs`

## 주요 기능
- **다중 카메라 입력**: `rear`, `left`, `right` 카메라 이미지 토픽 구독
- **라인 검출(노란선/흰선)**: ROI 하단 80%에서 색상 기반 필터링 및 비율 산출
- **주차 가능 구역 검출**: ROI 사각형 컨투어 면적 비율 ≥ 40% 시 True
- **강화학습 환경/에이전트 분리**: `dqn_environment` ↔ `dqn_agent`
- **충돌 판정**: 충돌 센서 추가 및 충돌 토픽 구독
- **학습 로깅/가시화**: 스텝 로그 가독성 개선 및 결과 분석 스크립트(`result_analysis/`)
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
- ROI(Region of Interest) 설정을 통해 화면 하단 80% 영역만 사용
- 노란선과 흰색 이진화 이미지 생성
- 주차 구역 검출을 위한 Parking Zone Detector 노드 구현
- ROI 내에서 사각형(4변) 컨투어 중, 넓이가 전체 ROI의 40% 이상일 경우 "주차 가능 구역"으로 판별
- 결과에 따라 **주차 가능 구역이면 True, 아니면 False**를 /parking_zone_detected 토픽(std_msgs/Bool)으로 발행
- OpenCV를 통한 검출 결과 시각화 지원(디버깅용)
> 브랜치: `main/parking_area_detect`
- main에 병합 완료

### 🚀 v0.0.06 - env
- 주차구역 감지 전까지 직진 수행

### 🚀 v0.0.07 - burget
- 차량 충돌센서 추가
- 학습 실패조건 충돌 방식 : odom을 이용한 차량속도와 cmd_vel의 차이 -> 충돌 센서 입력값
  
### 🚀 v0.0.08 - agent
- reward방식 수정
- environment step 로그 가시성 개편
- 초기화시 차량 흔들림 수정 (2번 초기화)
- 충돌 시 초기화 문제 수정
> 브랜치: `main/test_of_train`

## 🎥 결과 영상
[![강화학습을 통한 자율주차 학습결과](https://img.youtube.com/vi/zjEXBvHMCus/0.jpg)](https://www.youtube.com/watch?v=zjEXBvHMCus)

## 시스템 요구사항
- **OS**: Ubuntu 22.04 LTS 권장  
- **ROS 2**: Humble Hawksbill  
- **Simulator**: Gazebo Classic  
- **Python**: 3.10+  
- **패키지**: `turtlebot3_gazebo`, `gazebo_ros`, `image_transport`, `tf2_ros`, `sensor_msgs`, OpenCV 등

## ▶️ 실행 방법


```bash
# bashrc
function MLmaplaunch() {
	cd ~/auto_parking
	. install/setup.bash
	ros2 launch turtlebot3_dqn gazebo.launch.py
	}

# Launch Gazebo with gazebo_init, rear camera, right camera
MLmaplaunch
# Run environment node
ros2 run turtlebot3_dqn dqn_environment

# Run agent node
ros2 run turtlebot3_dqn dqn_agent
```
## 학습(Training) & 평가(Evaluation)
# 보상 설계(요지)

- 성공: 성공 점수 + ROI 기반 성공 점수 + 거리/각도 점수
- 실패(충돌/스텝초과): 거리/각도 점수 − (충돌/초과 패널티)
- 진행중: 얇은 shaping 중심(진행도 − 이동 소패널티)
# 로그/결과 분석: result_analysis
