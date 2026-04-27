# line_tracer

Ubuntu 20.04 + ROS Noetic 환경에서 **TurtleBot3**이 검은 바닥 위에 그려진 흰 선 두 개를 인식해 라인트레이싱하는 ROS 패키지입니다.  
노트북 카메라를 사용해 라즉탑에서 직접 영상을 처리하고 `cmd_vel` 토픽으로 보냔 로봇을 제어합니다.

---

## 환경

| 항목 | 버전 |
|---|---|
| OS | Ubuntu 20.04 LTS |
| ROS | Noetic (1.16+) |
| Python | 3.8 |
| 로봇 | TurtleBot3 Burger |
| 카메라 | OpenCR 내장 카메라 (320×240 @ 30fps) |

---

## 의존 패키지

### ROS 패키지

```bash
sudo apt update
sudo apt install -y \
  ros-noetic-rospy \
  ros-noetic-std-msgs \
  ros-noetic-sensor-msgs \
  ros-noetic-geometry-msgs \
  ros-noetic-cv-bridge \
  ros-noetic-image-transport
```

### Python 패키지

```bash
sudo apt install -y python3-opencv python3-numpy
```

| Python 패키지 | 용도 |
|---|---|
| `opencv-python` (`cv2`) | 영상 이진화, 컨투어 검출, 디버그 시각화 |
| `numpy` | 에러 클립, 적분 호이스트, 행렬 연산 |
| `cv_bridge` (ROS) | ROS Image 메시지 ↔ OpenCV Mat 변환 |

---

## 패키지 구조

```
line_tracer/
├── CMakeLists.txt
├── package.xml
├── launch/
│   └── line_tracer.launch      # 실행 진입점 + 파라미터 전체
├── scripts/
│   └── line_tracer_node.py     # 메인 노드 (v10)
└── config/                     # 파라미터 YAML (선택)
```

---

## 설치 및 빌드

```bash
# 1. 패키지 콜론
 cd ~/turtle_ws/src
git clone https://github.com/seongjun-k/line_tracer.git

# 2. 실행 권한 부여
chmod +x ~/turtle_ws/src/line_tracer/scripts/line_tracer_node.py

# 3. 빌드
cd ~/turtle_ws
catkin_make
source devel/setup.bash
```

---

## 실행

```bash
# TurtleBot3 모델 환경변수 설정 (한 번만 해두면 됨)
export TURTLEBOT3_MODEL=burger

# 라인트레이서 로시론치
roslaunch line_tracer line_tracer.launch
```

> 로봇측에서는 `roslaunch turtlebot3_bringup turtlebot3_robot.launch` 실행 후 위 명령어를 실행하세요.

---

## 2단 ROI 알고리즘 (v10)

카메라가 정면을 향할 때 선이 화면 하단에 맺히는 특성을 고려해 두 ROI를 모두 하단에 배치합니다.

```
┌─────────────────────┐  ← 0%  (화면 상단)
│                     │
│  (배경 / 벽 / 하늘)  │  ← 선 없는 구간 — 무시
│                     │
│  [주황] 예측 ROI     │  ← predict_start_ratio (0.65)
│  PRED   65% ~ 75%   │      커브 선행 예측
│                     │  ← predict_end_ratio   (0.75)
│  [빨강] 주행 ROI     │  ← drive_start_ratio   (0.77)
│  DRIVE  77% ~ 95%   │      실제 조향 결정
│                     │  ← drive_end_ratio     (0.95)
└─────────────────────┘  ← 100% (화면 하단)

최종 조향 = DRIVE × 0.75 + PREDICT × 0.25
```

### 데이터 흐름

```
카메라 프레임
  ↓
가우시안 블러 + 이진화 (white_threshold)
  ↓
모피로지 OPEN (노이즈 제거)
  ↓
컨투어 검출 + 폭 필터 (max_blob_width_ratio)
  ↓
[PREDICT ROI] 커브 방향 예측  +  [DRIVE ROI] 현재 조향 결정
  ↓
블렌딩 (0.75 / 0.25)
  ↓
비선형 PID → angular_z
  ↓
/cmd_vel 토픽 퍼블리시
```

---

## 주요 파라미터

### 제어

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `linear_speed` | `0.05` | 기본 직진 속도 (m/s) |
| `kp` | `0.004` | PID 비례 이득 |
| `ki` | `0.0` | PID 적분 이득 |
| `kd` | `0.005` | PID 미분 이득 |
| `dt` | `0.05` | 제어 주기 (s) |
| `curve_rate_threshold` | `120.0` | 급커브 판단 임계값 |
| `lost_max_frames` | `8` | 선 소실 시 PID 유지 업데이트 최대 프레임 |

### 2단 ROI

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `predict_start_ratio` | `0.65` | 예측 ROI 시작 (화면 높이 비율) |
| `predict_end_ratio` | `0.75` | 예측 ROI 끝 |
| `drive_start_ratio` | `0.77` | 주행 ROI 시작 |
| `drive_end_ratio` | `0.95` | 주행 ROI 끝 |
| `drive_weight` | `0.75` | 주행 ROI 블렌딩 비율 |
| `predict_weight` | `0.25` | 예측 ROI 블렌딩 비율 |

### 영상 처리

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `white_threshold` | `170` | 흰 선 이진화 임계값 (0~255) |
| `min_line_area` | `80` | 선 컨투어 최소 면적 (px²) |
| `max_blob_width_ratio` | `0.35` | 선 관심영역 대비 최대 폭 비율 |
| `camera_topic` | `/camera/image` | 카메라 입력 토픽 |
| `flip_image` | `false` | 좌우 반전 여부 |

---

## 제어 모드

| 모드 | 조건 | 동작 |
|---|---|---|
| `DRV+PRD` | 두 ROI 모두 인식 | 블렌딩 조향 (안정+선행) |
| `DRIVE` | 주행 ROI만 인식 | 주행 ROI 기준 안정 주행 |
| `PRED` | 예측 ROI만 인식 | 속도 ×0.10 초저속 + 예측 방향 |
| `LOST` | 두 ROI 모두 바닥 | 직전 에러로 PID 유지 (최대 8프레임) |
| `NO LINE` | LOST 초과 | 전수 정지 |

---

## 속도 감속 로직

| 상태 | 속도 배율 |
|---|---|
| PRED Only | × 0.10 |
| 급커브 (`error_rate > 120`) | × 0.15 |
| 큰 에러 (`> 50px`) | × 0.20 |
| 중간 에러 (`30~50px`) | × 0.35 |
| 직진 | × 0.20 ~ 1.0 (동적) |
| LOST 유지 | × 0.12 |

---

## 디버그 화면 구성

```
+---------------------------+
| DRV+PRD err=+12 ang=-0.05 |  ← 상태 텍스트
|  [PRED 주황 박스]        |  ← 예측 ROI 영역
|  [DRIVE 빨강 박스]       |  ← 주행 ROI 영역
|        ●(목표) ○(중심)    |  ← 목표점 / 중심점
+----------------+------+
                | mini |  ← 주행 ROI 이진화 미니맵
                +------+
```

---

## ROI 조정 가이드

| 증상 | 조치 |
|---|---|
| 선이 두 박스 **위**에 보임 | `predict_start_ratio` 낮추기 (0.55 등) |
| 선이 두 박스 **아래**(화면 박) | 카메라를 더 아래로 기울이기 |
| PRED만 잘 잡히고 DRIVE 놓침 | `drive_start_ratio` 낮추기 (0.72 등) |
| 선이 너무 올라가 자주 놓침 | `predict_weight` 영지 득더 낮추기 |

---

## 트러블슈팅

| 증상 | 원인 | 해결 |
|---|---|---|
| `cannot locate node ... not executable` | 실행 권한 없음 | `chmod +x scripts/line_tracer_node.py` |
| NO LINE 지속 | ROI가 선 위에 있음 | `predict_start_ratio` 낮추기 |
| 선 인식 후 직진 못 함 | Kp 너무 작음 | `kp` 0.006 등으로 운가 |
| 커브에서 주행 불안 | PID 진동 | `kd` 0.008로 올리기 |
| 마우스 노이즈 검틹 | `white_threshold` 너무 낙음 | 180~190으로 올리기 |

---

## 변경 이력

| 버전 | 주요 변경 |
|---|---|
| v1 | 기본 단일 ROI + 고정 임계값 |
| v2 | 비선형 PID (에러 크기별 Kp 자동 증가) |
| v3 | 적응형 이진화 임계값 (흰 바닥 노이즈 제거) |
| v4 | 듀얼 ROI Lookahead |
| v5 | 검은 트랙 추적 시도 |
| v6 | 흰선 컨투어 폭 필터로 바닥 블롭 제거 |
| v7 | 에러 변화율 기반 급커브 선제 감속, 한쪽 선 강한 회전 |
| v8 | 버그 수정: error_rate 계산 순서, 직진 speed_scale 상한 낮춤 |
| v9 | 선 소실 시 Last Known Error로 PID 유지 (최대 N프레임) |
| v10 | **2단 ROI**: 아래 DRIVE(주행) + 위 PREDICT(선행 예측), 정면 카메라 대응 |

---

## License

MIT
