# 🎙️ SonicTrack-ESP32: Real-time TDOA Sound Source Localization

> **ESP32와 4채널 마이크 어레이를 이용한 실시간 음원 위치 추적 및 시각화 시스템**

이 프로젝트는 **TDOA(Time Difference of Arrival)** 원리를 활용하여 소리가 발생하는 방향(방위각 및 고도)을 실시간으로 추적합니다. ESP32에서 수집된 고속 오디오 데이터를 PC로 전송하고, **GCC-PHAT** 알고리즘을 통해 소음 환경에서도 정밀한 각도를 계산하여 카메라 영상 위에 시각화합니다.

---

## ✨ Key Features
* **4-Channel Sync Sampling**: 2개의 I2S 인터페이스를 병렬 동기화하여 4개 마이크 간의 미세한 위상 차를 데이터화.
* **GCC-PHAT Algorithm**: 위상 변환 가중치를 적용하여 실내 반사음(Multipath) 환경에서도 높은 상관관계 피크 추출.
* **Advanced Signal Processing**:
  * **Butterworth Bandpass Filter**: 300Hz ~ 3000Hz 음성 대역 외 노이즈 차단.
  * **Confidence-based Filtering**: 신호 신뢰도(Peak-to-Mean Ratio)를 분석하여 허위 탐지(False Positive) 방지.
* **Real-time GUI**: OpenCV 기반의 컨트롤 패널을 통해 Azimuth/Elevation 궤적 히스토리 모니터링.

---

## 🛠 Tech Stack
| Category | Technologies |
| :--- | :--- |
| **Embedded** | ESP32 (C++ / Arduino), I2S Driver, Dual-core Processing |
| **Analysis** | Python 3.x, NumPy, SciPy (Signal Processing) |
| **Visual** | OpenCV (UI & Camera Overlay) |
| **Protocol** | High-speed Serial (1.5 Mbps) |

---

## 🏗 System Architecture

### 1. Hardware Connection
마이크는 **십자(Cross) 형태**로 배치하며, ESP32와 다음과 같이 연결합니다.

| Mic Index | Channel | Pin (WS/SD/SCK) | Note |
| :--- | :--- | :--- | :--- |
| **Mic 1** | I2S0 (L) | 25 / 22 / 26 | Horizontal Pair A |
| **Mic 2** | I2S0 (R) | 25 / 22 / 26 | Horizontal Pair B |
| **Mic 3** | I2S1 (L) | 15 / 34 / 14 | Vertical Pair A |
| **Mic 4** | I2S1 (R) | 15 / 34 / 14 | Vertical Pair B |

### 2. Localization Principle
마이크 간 거리 $d$와 소리의 속도 $v$를 기반으로, 도달 시간 차이 $\Delta t$를 측정하여 각도 $\theta$를 산출합니다.

$$\Delta t = \frac{d \cdot \sin(\theta)}{v}$$

---

## 📈 Technical Optimization (v4.1)

* **동기화 오차 방지**: `portDISABLE_INTERRUPTS()`를 사용하여 두 I2S 유닛의 시작 시점을 하드웨어 수준에서 정렬했습니다.
* **지터(Jitter) 억제**: Alpha Smoothing(고정 계수 보간)과 중앙값 필터(Median Filter)를 결합하여 추적 십자선이 떨리는 현상을 해결했습니다.
* **고속 데이터 스트리밍**: 16kHz/32bit 샘플링 데이터를 실시간으로 전송하기 위해 UART 대역폭을 1.5Mbps로 최적화했습니다..

---

## 🚀 Getting Started

### 1. Repository Structure
```text
.
├── firmware/
│   └── tracker_esp32.ino    # ESP32 High-speed sampling code
└── software/
    └── tdoa_visualizer.py   # Python analysis & UI code



원본코드 찾기
