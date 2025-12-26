# Doosan Robot Automated Board Processing System

두산로봇 M0609 기반 목재 도마 자동 가공 시스템

## 📋 프로젝트 개요

본 프로젝트는 ROS2 Action 기반의 목재 도마 자동화 가공 시스템으로, 픽앤플레이스부터 연마, 브러싱, 오일 도포까지 전체 공정을 순차적으로 수행합니다.

### 주요 특징
- **완전 자동화**: 5단계 공정 완전 자동 실행
- **개별 제어**: 각 공정 독립 실행 가능 (디버깅/테스트용)
- **실시간 피드백**: 작업 진행률 및 상태 실시간 모니터링
- **안전 설계**: 그리퍼 제어 및 충돌 방지 시스템

## 🏗️ 시스템 아키텍처

### 전체 공정 흐름
```
GUI 명령 → Main Coordinator → Action Servers → 로봇 제어
   ↓                              ↓
토픽 발행              피드백/결과 반환
```

### 5단계 작업 시퀀스
1. **Doma Pick** - 도마 픽업 및 작업대 이송
2. **Erasing** - 표면 연마 작업
3. **Brushing** - 먼지 제거 브러싱
4. **Oiling** - 오일 도포
5. **Doma Place** - 완성품 배치 및 복귀

## 📁 프로젝트 구조
```
my_doosan_project/
├── dsr_project/                    # 메인 제어 패키지
│   ├── dsr_project/
│   │   ├── main_node_gem.py       # 메인 코디네이터 노드
│   │   ├── Doma_a_pick_server.py  # 도마 픽업 액션 서버
│   │   ├── Doma_a_place_server.py # 도마 배치 액션 서버
│   │   ├── erasing_stop_server.py # 연마 액션 서버
│   │   ├── brushing_stop_server.py # 브러싱 액션 서버
│   │   ├── oiling_stop_server.py  # 오일링 액션 서버
│   │   ├── test_gui.py            # GUI 제어 인터페이스
│   │   └── ...
│   ├── launch/
│   │   └── board_pipeline.launch.py # 전체 시스템 런치 파일
│   ├── package.xml
│   └── setup.py
│
└── my_robot_interfaces/           # Custom ROS2 인터페이스
    ├── action/
    │   └── BrushingAction.action  # 공통 액션 인터페이스
    ├── CMakeLists.txt
    └── package.xml
```

## 🎯 핵심 컴포넌트

### 1. Main Coordinator (`main_node_gem.py`)
**역할**: 전체 공정 오케스트레이션

**주요 기능**:
- 5개 액션 서버 클라이언트 관리
- `/main_task_cmd` 토픽으로 명령 수신
- 순차 실행 제어 및 에러 핸들링
- 실시간 피드백 모니터링

**지원 명령어**:
```python
START_ALL          # 전체 공정 실행 (Pick→Erasing→Brushing→Oiling→Place)
START_DOMA_PICK    # 도마 픽업만 실행
START_ERASING      # 연마만 실행
START_BRUSHING     # 브러싱만 실행
START_OILING       # 오일링만 실행
START_DOMA_PLACE   # 도마 배치만 실행
```

### 2. Action Servers (5개)

각 서버는 다음 구조를 공유합니다:

#### 공통 특징
- **Action Type**: `BrushingAction` (공통 인터페이스 재사용)
- **Thread 구조**: ROS Spin Thread + Task Thread (이중 스레드)
- **피드백**: 실시간 진행률(0-100%) 및 상태 메시지
- **안전**: 작업 중 중복 요청 거부

#### 개별 서버 상세

##### `Doma_a_pick_server.py`
- **Action Name**: `/dsr01/do_doma_pick_action`
- **기능**: 도마 픽업 → 작업대 이송
- **주요 동작**:
  1. 홈 위치에서 출발
  2. 도마 위치로 접근 (2단계 경로)
  3. 그리퍼로 도마 파지
  4. 작업대로 이송 (고속 이동)
  5. 정밀 배치 (6단계 하강)

##### `erasing_stop_server.py`
- **Action Name**: `/dsr01/do_eraser_action`
- **기능**: 도마 표면 연마
- **특징**: 반복 스트로크 동작

##### `brushing_stop_server.py`
- **Action Name**: `/dsr01/do_brushing_action`
- **기능**: 연마 후 먼지 제거
- **특징**: 브러시 도구 사용

##### `oiling_stop_server.py`
- **Action Name**: `/dsr01/do_oiling_action`
- **기능**: 도마 표면 오일 도포
- **특징**: 균일한 코팅 패턴

##### `Doma_a_place_server.py`
- **Action Name**: `/dsr01/do_doma_place_action`
- **기능**: 완성품 배치 및 복귀
- **주요 동작**:
  1. 작업대에서 도마 픽업
  2. 배치 위치로 이동
  3. 정밀 하강 및 배치
  4. 그리퍼 해제
  5. 홈 위치 복귀

### 3. GUI Controller (`test_gui.py`)
PyQt 기반 사용자 인터페이스로 전체 공정 제어

### 4. Launch File (`board_pipeline.launch.py`)
전체 시스템을 한 번에 실행하는 통합 런치 파일
```python
# 실행되는 노드:
# - Doma_a_pick_server
# - erasing_stop_server  
# - brushing_stop_server
# - oiling_stop_server
# - Doma_a_place_server
# - main_node_gem (coordinator)
```

## 💻 개발 환경

### 하드웨어
- **로봇**: Doosan M0609 협동로봇
- **그리퍼**: Robotiq RG2 (디지털 I/O 제어)
- **컨트롤러**: Doosan 로봇 컨트롤러

### 소프트웨어
- **OS**: Ubuntu 22.04
- **ROS**: ROS2 Humble
- **Python**: 3.10+
- **필수 패키지**:
  - `rclpy`
  - `dsr_robot2` (Doosan 드라이버)
  - `std_msgs`

### 로봇 설정
```python
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TCP = "Tool Weight"
ROBOT_TOOL = "GripperDA_v1"
VELOCITY = 30        # 일반 속도
ACC = 30            # 일반 가속도
VELOCITY_fast = 60  # 고속 이동
ACC_fast = 60       # 고속 가속도
```

## 🚀 설치 및 실행

### 1. 저장소 클론
```bash
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src
git clone https://github.com/juyeong82/juyeong82-doosan-robot-project.git
```

### 2. 의존성 설치
```bash
cd ~/robot_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 3. 빌드
```bash
cd ~/robot_ws
colcon build --packages-select my_robot_interfaces dsr_project
source install/setup.bash
```

### 4. 실행

#### 전체 시스템 실행
```bash
# 모든 액션 서버 + 메인 코디네이터 실행
ros2 launch dsr_project board_pipeline.launch.py
```

#### 개별 노드 실행 (디버깅용)
```bash
# 메인 코디네이터만
ros2 run dsr_project main_node_gem

# 픽업 서버만
ros2 run dsr_project Doma_a_pick_server

# 연마 서버만
ros2 run dsr_project erasing_stop_server
```

### 5. 명령 전송

#### CLI에서 명령 전송
```bash
# 전체 공정 실행
ros2 topic pub --once /main_task_cmd std_msgs/String "data: 'START_ALL'"

# 픽업만 실행
ros2 topic pub --once /main_task_cmd std_msgs/String "data: 'START_DOMA_PICK'"
```

#### GUI 사용
```bash
ros2 run dsr_project test_gui
```

## 📊 Action 인터페이스

### BrushingAction 구조
```
# Goal
bool start_task              # 작업 시작 명령

---
# Result  
bool complete_task           # 작업 완료 여부
float64 total_duration       # 총 소요 시간 (초)
float64[6] final_pose        # 최종 로봇 자세 [x,y,z,rx,ry,rz]

---
# Feedback
string feedback_string       # 현재 작업 상태 메시지
float64 progress_percentage  # 진행률 (0.0 ~ 100.0)
int32 current_stroke         # 현재 스트로크 번호
```

## 🔧 트러블슈팅

### 자주 발생하는 문제

#### 1. Action 서버 연결 실패
```bash
# 서버 실행 확인
ros2 action list

# 예상 출력:
# /dsr01/do_doma_pick_action
# /dsr01/do_eraser_action
# /dsr01/do_brushing_action
# /dsr01/do_oiling_action
# /dsr01/do_doma_place_action
```

#### 2. 로봇 초기화 실패
- Tool/TCP 이름이 로봇 설정과 일치하는지 확인
- Doosan 컨트롤러 연결 상태 확인

#### 3. Goal 거부됨
- 이미 작업 실행 중인지 확인
- `start_task: true` 로 설정했는지 확인

## 📈 성능 지표

### 작업 시간 (평균)
- Doma Pick: ~45초
- Erasing: ~60초
- Brushing: ~40초
- Oiling: ~50초
- Doma Place: ~50초
- **전체 공정**: ~4분 30초

### 정밀도
- 위치 정밀도: ±1mm
- 각도 정밀도: ±0.5°

## 🔒 안전 기능

1. **작업 중 중복 요청 방지**: Goal 거부 메커니즘
2. **그리퍼 안전 제어**: 동작 전후 1초 대기
3. **경로 최적화**: Radius 파라미터로 부드러운 이동
4. **예외 처리**: 모든 동작에 try-except 적용
5. **스레드 안전**: Event 기반 동기화

## 📝 개발 이력

- **2024.11**: 기본 픽앤플레이스 시스템 구축
- **2024.12**: 5단계 공정 통합 완료
- **2024.12**: Action 기반 아키텍처 전환
- **2024.12**: 피드백 시스템 및 GUI 추가

## 👥 개발팀

- **Doosan Robotics BootCamp 5기**
- 협동1 프로젝트 2025

## 📄 라이선스

이 프로젝트는 교육 목적으로 개발되었습니다.

## 🙏 감사의 글

- Doosan Robotics: 로봇 플랫폼 및 드라이버 지원
- ROS2 Community: 프레임워크 제공

---

**Last Updated**: 2024.12.26
