# 🛰️ TDOA Spatial Tracker v5.0
> **Advanced Real-time 4-Channel Sound Source Localization System**

본 프로젝트는 ESP32와 4채널 I2S MEMS 마이크 배열을 활용하여 음원의 방향(Azimuth, Elevation)을 실시간으로 추적하고 공간 특성을 분석하는 시스템입니다. 단순한 소리 감지를 넘어, 하드웨어 동기화 제약과 신호 처리 오차를 공학적으로 해결하여 높은 정밀도를 구현했습니다.

---

## 🚀 Key Features

* **GCC-PHAT & Subsample Interpolation**: 주파수 영역의 위상차를 이용한 GCC-PHAT 알고리즘과 포물선 보간법(Parabolic Interpolation)을 적용하여 각도 오차를 초기 대비 최대 70% 감소시켰습니다.
* **정밀한 신호 처리 알고리즘**:
    * **De-aliasing**: 300~3000Hz 대역 제한 및 공간 에일리어싱 제거를 통해 데이터 안정성 확보. 
    * **Linear Correction**: 실측 데이터 기반의 선형 회귀 분석 모델을 적용하여 마이크 부착 오차 보정. 
* **고성능 데이터 파이프라인**: 1.5Mbps 고속 시리얼 통신 및 전용 바이너리 패킷 프로토콜을 설계하여 128KB/s의 PCM 데이터를 저지연 전송합니다. 
* **통합 실시간 UI (OpenCV)**: 타겟 오버레이, 위치 히스토리 그래프, 반향 지수(Reverb Index), 음원 이동 상태(Motion)를 대시보드 형태로 시각화합니다. 

---

## 🛠️ Tech Stack

* **Hardware**: ESP32 (Main MCU), INMP441 I2S Microphone x4 
* **Firmware**: C++ (Arduino/ESP-IDF), I2S DMA 동기화 및 인터럽트 제어 
* **Software**: Python 3.8+, NumPy, OpenCV, SciPy, PySerial 

---

## 🔌 Hardware Configuration

### Mic Array Geometry
* **구조**: 4채널 정사가각형 배열 (Half-side = 3.5cm) 
* **동기화**: ESP32의 독립 클럭 문제를 해결하기 위해 인터럽트 비활성화 후 두 I2S 포트를 동시 시작하도록 설계하여 샘플 시간 오프셋을 최소화했습니다. 


---

## 📊 Accuracy Improvement (Step-by-Step)

발표 자료에 근거한 알고리즘 단계별 성능 개선 수치입니다. 

| 단계 | 적용 알고리즘/설정 | 2D 평균 오차 | 주요 개선 사항 |
| :--- | :--- | :--- | :--- |
| **① 초기** | 정수 TDOA, PHAT 미적용 | 31° | 가장 단순한 버전, 실사용 어려움 |
| **② GCC-PHAT** | PHAT 가중치 적용 | 26° | 피크 선명도 개선 (약 15~20%) |
| **③ Subsample** | 포물선 보간법 적용 | 17° | Fractional delay로 약 35% 추가 개선 |
| **④ Band-pass** | 2-6kHz 대역 필터링 | 12.5° | Aliasing 감소 및 유효 대역 집중 |
| **⑤ 보정 모델** | 선형 보정 (a+b) 적용 | **7.8°** | 마이크 부착 오차 보정, 최종 성능 |

---

## 🎥 Demo & UI Overview



https://github.com/user-attachments/assets/1c542a70-31f4-4c53-9b3d-8e6cb9195844



* **Direction (Az/El)**: 현재 추정된 방위각과 고도각 수치 및 미니 컴파스 표시.
* **Distance Estimate (Ref)**: 에너지(RMS) 기반 거리 추정. (절대 거리 정확도는 환경에 따라 변동될 수 있음) 
* **Spatial Analysis**:
    * **Reverb Index**: Direct/Mixed/Reverb 환경 분류.
    * **Motion Status**: 음원 각속도 기반 이동 상태(Static/Moving/Fast) 분류. 
    * **Certainty**: 현재 위치 데이터의 확실성(%) 시각화. 

---

## ⚠️ Limitations & Future Work

* **한계점**: 절대 거리 정확도 부족, 2S 동기화 및 각도 분해능 제한, 단일 음원 추적만 가능.
* **향후 계획**: 
    * 2.5D 틸팅 라이다 기반 자율주행 전차(UGV)의 사각지대 보완 센서로 통합. 
    * 다중 음원 분리 및 정밀 하드웨어 동기화 모듈 추가. 

---

### 💡 Engineering Note
> "하드웨어의 물리적 제약(클럭 오프셋, 위상차 보존)을 소프트웨어적 최적화(DMA 제어, 보간 알고리즘)로 극복한 프로젝트입니다. AI를 도구로 활용해 개발 속도를 높이되, 실제 물리 환경에서 발생하는 오차 원인을 분석하고 '보정 모델'을 직접 설계하여 해결하는 엔지니어링 과정을 거쳤습니다."
