# 🤖 TurtleBot3 Line Tracer

> Ubuntu 20.04 + ROS Noetic 환경에서 TurtleBot3가 **검은 바탕 위 흰 선 두 개**를 인식해 라인 트레이싱하는 ROS 패키지입니다.

---

## 📦 패키지 구조

```
line_tracer/
├── scripts/
│   └── line_tracer_node.py   # 메인 노드
├── launch/
│   └── line_tracer.launch    # 실행 런치 파일
├── config/
│   └── params.yaml           # 파라미터 모음
├── CMakeLists.txt
└── package.xml
```

---

## 🧠 알고리즘 구조

```
카메라 이미지
    │
    ▼
[이진화]  GaussianBlur → THRESH_BINARY (white_threshold)
    │
    ▼
[컨투어 폭 필터]  가로 폭 > roi_w × max_blob_width_ratio → 바닥 블롭 제거
    │
    ▼
[듀얼 ROI]  FAR ROI(원거리) + NEAR ROI(근거리) 가중 합산
    │
    ▼
[에러 계산]  두 선의 중점 - 이미지 중심
    │
    ▼
[비선형 PID]  에러 크기별 Kp 자동 증가
    │
    ▼
[속도 제어]  에러 변화율 기반 급커브 선제 감속
    │
    ▼
/cmd_vel 퍼블리시
```

---

## 🚗 속도 감속 로직

| 상태 | 조건 | 속도 배율 |
|---|---|---|
| 급커브 진입 감지 | `error_rate > curve_rate_threshold` | × 0.15 |
| 급커브 | `\|err\| > 50` | × 0.20 |
| 중간 커브 | `\|err\| > 30` | × 0.35 |
| 직진 / 완만한 커브 | 그 외 | × 0.15 ~ 1.0 (angular에 비례 감속) |
| 한쪽 선만 감지 | left/right only | 0.03 m/s + ±0.8 회전 |
| 선 없음 | none | 정지 |

> **`error_rate`** = 현재 에러와 직전 에러의 차이 / dt  
> 커브 진입 직전 에러가 급격히 변하면 속도를 선제적으로 줄임

---

## ⚙️ 설치 및 빌드

### 의존성 확인

```bash
sudo apt install ros-noetic-cv-bridge python3-opencv
```

### 클론 & 빌드

```bash
cd ~/turtle_ws/src
git clone https://github.com/seongjun-k/line_tracer.git
cd ~/turtle_ws
catkin_make
source devel/setup.bash
```

---

## 🚀 실행

```bash
roslaunch line_tracer line_tracer.launch
```

### 디버그 이미지 확인

```bash
rqt_image_view /line_tracer/debug_image
```

---

## 🔧 주요 파라미터

`launch/line_tracer.launch` 또는 `config/params.yaml` 에서 수정 가능합니다.

### 제어 파라미터

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `linear_speed` | `0.06` | 기본 직진 속도 (m/s) |
| `kp` | `0.004` | PID 비례 이득 |
| `ki` | `0.0` | PID 적분 이득 |
| `kd` | `0.005` | PID 미분 이득 (진동 억제) |
| `dt` | `0.05` | 제어 주기 (s) |
| `curve_rate_threshold` | `200.0` | 급커브 감속 트리거 (에러 변화율) |

### 이미지 처리 파라미터

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `white_threshold` | `170` | 흰 선 이진화 임계값 (낮을수록 민감) |
| `min_line_area` | `80` | 최소 선 픽셀 면적 (노이즈 제거) |
| `max_blob_width_ratio` | `0.35` | 바닥 블롭 필터 비율 (낮을수록 엄격) |
| `flip_image` | `false` | 카메라 좌우 반전 여부 |
| `camera_topic` | `/camera/image` | 카메라 토픽 이름 |

### ROI 파라미터

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `near_start_ratio` | `0.80` | 근거리 ROI 시작 위치 (이미지 높이 비율) |
| `far_start_ratio` | `0.75` | 원거리 ROI 시작 위치 |
| `far_end_ratio` | `0.85` | 원거리 ROI 끝 위치 |
| `far_weight` | `0.0` | 원거리 ROI 가중치 (0이면 비활성) |
| `near_weight` | `1.0` | 근거리 ROI 가중치 |

---

## 🐛 디버그 화면 설명

```
┌────────────────────────────────────────────┐
│ NEAR err=+12 ang=-0.048 spd=0.055 rate=24  │  ← 상태 텍스트
│                                            │
│   ──── 주황 박스: FAR ROI ────             │
│   ──── 청록 박스: NEAR ROI ───             │
│        ● 초록점: 왼쪽 선                  │
│        ● 파란점: 오른쪽 선               │
│        ● 노란점: 두 선의 중점             │
│        ● 분홍점: 이미지 중심             │
│                                            │
│                         ┌──────────┐       │
│                         │ 이진화   │       │  ← 우하단 미니맵
│                         │ 미니뷰   │       │
│                         └──────────┘       │
└────────────────────────────────────────────┘
```

---

## 📝 변경 이력

| 버전 | 주요 변경 내용 |
|---|---|
| v1 | 기본 단일 ROI + 고정 임계값 |
| v2 | 비선형 PID (에러 크기별 Kp 자동 증가) |
| v3 | 적응형 임계값 (흰 바닥 노이즈 제거) |
| v4 | 듀얼 ROI Lookahead 추가 |
| v5 | 검은 트랙 추적 시도 |
| v6 | 흰선 + 컨투어 폭 필터로 바닥 블롭 제거 |
| v7 | 에러 변화율 기반 급커브 선제 감속, 한쪽 선 감지 시 강한 회전 |
| **v8** | **버그 수정: error_rate 계산 순서 오류, 직진 최소 속도 고정 문제 해결** |

---

## 🛠 트러블슈팅

| 증상 | 원인 | 해결 |
|---|---|---|
| 커브에서 이탈 | `white_threshold` 너무 높음 | 값을 낮춰서 선 인식 민감도 증가 |
| 바닥을 선으로 인식 | `max_blob_width_ratio` 너무 높음 | 값을 낮춰서 넓은 블롭 제거 |
| 직진에서 좌우 떨림 | `kd` 너무 낮음 | `kd` 값 증가 |
| 반응이 느림 | `kp` 너무 낮음 | `kp` 값 증가 |
| 한쪽 선만 인식 | ROI 위치 오류 | `near_start_ratio` 조정 |
| 커브에서도 속도가 안 줄어듦 | `curve_rate_threshold` 너무 높음 | 값을 낮춰서 감속 조기 발동 |

---

## 📋 환경

- **OS**: Ubuntu 20.04
- **ROS**: Noetic
- **로봇**: TurtleBot3 (Burger / Waffle)
- **카메라**: 라즈베리파이 카메라 v2 또는 USB 웹캠
- **Python**: 3.8+
- **OpenCV**: 4.x
