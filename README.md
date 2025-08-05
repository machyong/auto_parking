# auto_parking
## 📝 프로젝트 개요

이 프로젝트는 TurtleBot3 Burger에 전방 및 후방 카메라를 장착하여 다양한 방향에서의 시각 정보를 활용할 수 있도록 하는 시뮬레이션 환경을 구성합니다.  
Gazebo 기반 autorace 맵을 사용하며, 이후 주차, 라인 트래킹, 물체 탐지 등의 기능으로 확장될 예정입니다.

---

## 📁 브랜치 설명

| 브랜치명 | 설명 |
|----------|------|
| `main` | 기본 TurtleBot3 및 autorace 환경 |
| `feature/rear-camera` | 후방 카메라 장착 및 노드 통합, TF 설정 및 rear/image_raw 발행 기능 포함 |

---

## 📌 사용된 주요 ROS2 패키지
- `turtlebot3_gazebo`
- `gazebo_ros`
- `image_transport`
- `tf2_ros`
- `sensor_msgs`
## 🔧 Version History

### 🚀 v0.2.0 - Rear Camera Integration
- Rear camera 추가 및 TF 프레임 설정
- rear/image_raw, rear/camera_info 토픽 발행 확인
- rear_camera_link 기준 pose 및 카메라 각도 튜닝
- autorace 맵에서 rear camera 작동 테스트 완료
- 노란선 필터링을 위한 초기 Line Detector 노드 테스트 진행

> 브랜치: `feature/rear-camera`
