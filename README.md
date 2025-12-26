# Doosan Robot Project

두산로봇 M0609 기반 작업 자동화 프로젝트

## 📁 프로젝트 구조
```
├── dsr_project/              # 메인 제어 패키지
│   ├── dsr_project/          # Python 노드들
│   │   ├── main_node.py      # 메인 제어 노드
│   │   ├── *_action_client.py
│   │   ├── *_action_server.py
│   │   └── ...
│   ├── launch/               # Launch 파일
│   └── package.xml
│
└── my_robot_interfaces/      # Custom 인터페이스
    ├── action/
    │   └── BrushingAction.action
    └── CMakeLists.txt
```

## 🚀 주요 기능

- **브러싱**: 자동 브러싱 작업
- **오일링**: 오일 도포 작업
- **지우기**: 표면 청소 작업
- **픽앤플레이스**: 물체 집기 및 배치
- **Firebase 연동**: 실시간 데이터 모니터링
- **GUI 제어**: PyQt 기반 사용자 인터페이스

## 💻 개발 환경

- **ROS2**: Humble
- **Python**: 3.10+
- **로봇**: Doosan M0609
- **그리퍼**: Robotiq RG2
- **카메라**: Intel RealSense

## 📦 설치 방법
```bash
# 워크스페이스 생성
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src

# 저장소 클론
git clone https://github.com/juyeong82/doosan-robot-project1.git

# 의존성 설치
cd ~/robot_ws
rosdep install --from-paths src --ignore-src -r -y

# 빌드
colcon build --packages-select my_robot_interfaces dsr_project

# 환경 설정
source install/setup.bash
```

## 🎯 사용 방법
```bash
# Launch 파일 실행
ros2 launch dsr_project board_pipeline.launch.py
```

## 📝 개발자

- **Korea University Robotics Lab**
- 2024-2025 졸업 프로젝트
