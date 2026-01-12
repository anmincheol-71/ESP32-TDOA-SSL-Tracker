"""
TDOA Tracker v5.0 - Advanced Spatial Analysis

추가 기능:
1. 에너지 기반 거리 추정 (Near/Mid/Far)
2. 신호 안정성 분석 (직접음 vs 반사음)
3. 음원 이동 속도 추정
4. 공간 반향 지수 (Reverb Index)
5. 신뢰도 기반 위치 확실성 표시
"""

import serial
import struct
import numpy as np
import cv2
from collections import deque
import time
from scipy.signal import butter, filtfilt

# -----------------------------
# 설정
# -----------------------------
PORT = "COM3"
BAUD = 1500000
HEADER = b'\xFF\xAA'
PACKET_SIZE = 10

CAM_ID = 0
VID_W, VID_H = 640, 480
PANEL_W = 450
WIN_W = VID_W + PANEL_W
WIN_H = VID_H

SAMPLE_RATE = 16000
MIC_DIST = 0.035
SOUND_SPEED = 343.0

FRAME_SIZE = 512
HOP_SIZE = 128

ENERGY_THRESHOLD = 800
CONF_THRESHOLD = 3.0
FREQ_LOW = 300
FREQ_HIGH = 3000

H_SCALE = 1.5
V_SCALE = 1.8

ALPHA_H = 0.25
ALPHA_V = 0.20

HISTORY_LEN = 30
MAX_DELAY = 6
DEBUG_PRINT = False

FOV_H = 70.0
FOV_V = 50.0

# === 거리 추정 캘리브레이션 ===
# 이 값들은 환경에 맞게 조정 필요
ENERGY_NEAR = 3000      # 이 이상이면 "가까움" (~0.5m 이내)
ENERGY_MID = 1500       # 이 이상이면 "중간" (~1-2m)
ENERGY_FAR = 800        # 이 이상이면 "멀음" (~2-4m)
# 이하면 "매우 멀음" 또는 노이즈

# UI 색상 (BGR)
COLOR_BG = (15, 15, 20)
COLOR_PANEL = (25, 28, 32)
COLOR_CARD = (35, 38, 45)
COLOR_TEXT = (220, 220, 220)
COLOR_TEXT_DIM = (120, 120, 130)
COLOR_ACCENT = (0, 200, 120)
COLOR_ACCENT2 = (255, 180, 50)
COLOR_WARN = (0, 150, 255)
COLOR_DANGER = (60, 60, 220)
COLOR_INACTIVE = (60, 60, 70)

# 거리 색상
COLOR_NEAR = (0, 100, 255)      # 빨강 (가까움)
COLOR_MID = (0, 200, 255)       # 주황 (중간)
COLOR_FAR = (0, 255, 100)       # 녹색 (멀음)
COLOR_VERY_FAR = (200, 200, 0)  # 시안 (매우 멀음)

# -----------------------------
# 필터
# -----------------------------
def create_bandpass_filter(lowcut, highcut, fs, order=4):
    nyq = 0.5 * fs
    low = lowcut / nyq
    high = highcut / nyq
    b, a = butter(order, [low, high], btype='band')
    return b, a

FILTER_B, FILTER_A = create_bandpass_filter(FREQ_LOW, FREQ_HIGH, SAMPLE_RATE)

def apply_bandpass(signal):
    if len(signal) < 15:
        return signal
    try:
        return filtfilt(FILTER_B, FILTER_A, signal)
    except:
        return signal

# -----------------------------
# 알고리즘
# -----------------------------
def gcc_phat_simple(sig1, sig2, max_delay=3):
    n = len(sig1)
    sig1 = sig1 - np.mean(sig1)
    sig2 = sig2 - np.mean(sig2)

    e1 = np.sqrt(np.mean(sig1**2))
    e2 = np.sqrt(np.mean(sig2**2))
    if e1 < 1 or e2 < 1:
        return 0.0, 0.0, 0.0

    nfft = 2 * n
    S1 = np.fft.fft(sig1, nfft)
    S2 = np.fft.fft(sig2, nfft)

    R = S1 * np.conj(S2)
    cc = np.fft.ifft(R).real
    cc = np.fft.fftshift(cc)

    center = len(cc) // 2
    search_start = center - max_delay
    search_end = center + max_delay + 1
    search_region = cc[search_start:search_end]

    if len(search_region) == 0:
        return 0.0, 0.0, 0.0

    peak_idx = np.argmax(search_region)
    peak_val = search_region[peak_idx]

    mean_val = np.mean(np.abs(cc)) + 1e-12
    confidence = peak_val / mean_val if mean_val > 0 else 0
    
    # 피크 선명도 (직접음 vs 반사음 구분용)
    sorted_vals = np.sort(search_region)[::-1]
    if len(sorted_vals) >= 2 and sorted_vals[1] > 0:
        peak_sharpness = sorted_vals[0] / (sorted_vals[1] + 1e-12)
    else:
        peak_sharpness = 1.0

    if peak_idx > 0 and peak_idx < len(search_region) - 1:
        left = search_region[peak_idx - 1]
        center_val = search_region[peak_idx]
        right = search_region[peak_idx + 1]
        denom = left - 2 * center_val + right
        if abs(denom) > 1e-10:
            offset = 0.5 * (left - right) / denom
            offset = np.clip(offset, -0.5, 0.5)
        else:
            offset = 0.0
    else:
        offset = 0.0

    delay = (peak_idx - max_delay) + offset
    return delay, confidence, peak_sharpness

def tdoa_to_angle(tdoa):
    time_delay = tdoa / SAMPLE_RATE
    sin_theta = (time_delay * SOUND_SPEED) / MIC_DIST
    sin_theta = np.clip(sin_theta, -1.0, 1.0)
    return np.degrees(np.arcsin(sin_theta))

def estimate_direction(m1, m2, m3, m4):
    m1 = apply_bandpass(m1)
    m2 = apply_bandpass(m2)
    m3 = apply_bandpass(m3)
    m4 = apply_bandpass(m4)
    
    tdoa_h1, conf_h1, sharp_h1 = gcc_phat_simple(m1, m2, max_delay=MAX_DELAY)
    tdoa_h2, conf_h2, sharp_h2 = gcc_phat_simple(m3, m4, max_delay=MAX_DELAY)
    tdoa_v1, conf_v1, sharp_v1 = gcc_phat_simple(m1, m3, max_delay=MAX_DELAY)
    tdoa_v2, conf_v2, sharp_v2 = gcc_phat_simple(m2, m4, max_delay=MAX_DELAY)

    if conf_h1 + conf_h2 > 0:
        tdoa_h = (tdoa_h1 * conf_h1 + tdoa_h2 * conf_h2) / (conf_h1 + conf_h2)
    else:
        tdoa_h = 0.0

    if conf_v1 + conf_v2 > 0:
        tdoa_v = (tdoa_v1 * conf_v1 + tdoa_v2 * conf_v2) / (conf_v1 + conf_v2)
    else:
        tdoa_v = 0.0

    azimuth = tdoa_to_angle(tdoa_h) * H_SCALE
    elevation = -tdoa_to_angle(tdoa_v) * V_SCALE
    
    # 평균 피크 선명도 (반향 지수)
    avg_sharpness = (sharp_h1 + sharp_h2 + sharp_v1 + sharp_v2) / 4.0

    return azimuth, elevation, tdoa_h, tdoa_v, (conf_h1, conf_h2, conf_v1, conf_v2), avg_sharpness

# -----------------------------
# 공간 분석 클래스
# -----------------------------
class SpatialAnalyzer:
    def __init__(self):
        self.energy_history = deque(maxlen=30)
        self.az_history = deque(maxlen=HISTORY_LEN)
        self.el_history = deque(maxlen=HISTORY_LEN)
        self.sharpness_history = deque(maxlen=20)
        self.position_history = deque(maxlen=10)  # (az, el, time) 튜플
        
        self.distance_estimate = "---"
        self.distance_meters = 0.0
        self.reverb_index = 0.0
        self.movement_speed = 0.0
        self.position_certainty = 0.0
        
    def update(self, az, el, energy, confidences, sharpness):
        current_time = time.time()
        
        # 히스토리 업데이트
        self.energy_history.append(energy)
        self.az_history.append(az)
        self.el_history.append(el)
        self.sharpness_history.append(sharpness)
        self.position_history.append((az, el, current_time))
        
        # 거리 추정
        self._estimate_distance(energy)
        
        # 반향 지수 계산
        self._calculate_reverb_index()
        
        # 이동 속도 계산
        self._calculate_movement_speed()
        
        # 위치 확실성 계산
        self._calculate_position_certainty(confidences)
    
    def _estimate_distance(self, energy):
        """에너지 기반 거리 추정"""
        if energy >= ENERGY_NEAR:
            self.distance_estimate = "NEAR"
            # 에너지 → 거리 근사 (역제곱 법칙)
            self.distance_meters = 0.3 + (ENERGY_NEAR - energy) / ENERGY_NEAR * 0.3
            self.distance_meters = max(0.1, self.distance_meters)
        elif energy >= ENERGY_MID:
            self.distance_estimate = "MID"
            ratio = (energy - ENERGY_MID) / (ENERGY_NEAR - ENERGY_MID)
            self.distance_meters = 2.0 - ratio * 1.5
        elif energy >= ENERGY_FAR:
            self.distance_estimate = "FAR"
            ratio = (energy - ENERGY_FAR) / (ENERGY_MID - ENERGY_FAR)
            self.distance_meters = 4.0 - ratio * 2.0
        else:
            self.distance_estimate = "VERY FAR"
            self.distance_meters = 4.0 + (ENERGY_FAR - energy) / ENERGY_FAR * 2.0
            self.distance_meters = min(10.0, self.distance_meters)
    
    def _calculate_reverb_index(self):
        """반향 지수 계산 (높을수록 직접음, 낮을수록 반사음 많음)"""
        if len(self.sharpness_history) > 0:
            self.reverb_index = np.mean(list(self.sharpness_history))
        else:
            self.reverb_index = 1.0
    
    def _calculate_movement_speed(self):
        """음원 이동 속도 추정 (각도/초)"""
        if len(self.position_history) >= 2:
            positions = list(self.position_history)
            
            # 최근 위치들로 속도 계산
            total_angular_dist = 0.0
            total_time = 0.0
            
            for i in range(1, len(positions)):
                az1, el1, t1 = positions[i-1]
                az2, el2, t2 = positions[i]
                
                angular_dist = np.sqrt((az2 - az1)**2 + (el2 - el1)**2)
                time_diff = t2 - t1
                
                if time_diff > 0:
                    total_angular_dist += angular_dist
                    total_time += time_diff
            
            if total_time > 0:
                self.movement_speed = total_angular_dist / total_time
            else:
                self.movement_speed = 0.0
        else:
            self.movement_speed = 0.0
    
    def _calculate_position_certainty(self, confidences):
        """위치 확실성 계산"""
        avg_conf = sum(confidences) / 4.0
        
        # confidence + 안정성 조합
        if len(self.az_history) >= 3:
            az_std = np.std(list(self.az_history)[-5:])
            el_std = np.std(list(self.el_history)[-5:])
            stability = 1.0 / (1.0 + az_std + el_std)
        else:
            stability = 0.5
        
        # 반향이 적을수록 확실
        reverb_factor = min(1.0, self.reverb_index / 2.0)
        
        self.position_certainty = (avg_conf / 20.0) * stability * reverb_factor
        self.position_certainty = np.clip(self.position_certainty, 0.0, 1.0)
    
    def get_distance_color(self):
        if self.distance_estimate == "NEAR":
            return COLOR_NEAR
        elif self.distance_estimate == "MID":
            return COLOR_MID
        elif self.distance_estimate == "FAR":
            return COLOR_FAR
        else:
            return COLOR_VERY_FAR
    
    def get_movement_status(self):
        if self.movement_speed < 5:
            return "STATIC", COLOR_ACCENT
        elif self.movement_speed < 20:
            return "SLOW", COLOR_ACCENT2
        elif self.movement_speed < 50:
            return "MOVING", COLOR_WARN
        else:
            return "FAST", COLOR_DANGER

# -----------------------------
# UI 그리기
# -----------------------------
def draw_rounded_rect(img, pt1, pt2, color, radius=8, thickness=-1):
    x1, y1 = pt1
    x2, y2 = pt2
    
    if thickness == -1:
        cv2.rectangle(img, (x1 + radius, y1), (x2 - radius, y2), color, -1)
        cv2.rectangle(img, (x1, y1 + radius), (x2, y2 - radius), color, -1)
        cv2.circle(img, (x1 + radius, y1 + radius), radius, color, -1)
        cv2.circle(img, (x2 - radius, y1 + radius), radius, color, -1)
        cv2.circle(img, (x1 + radius, y2 - radius), radius, color, -1)
        cv2.circle(img, (x2 - radius, y2 - radius), radius, color, -1)

def draw_panel(panel, az, el, energy, analyzer):
    panel[:] = COLOR_PANEL
    
    # === 헤더 ===
    cv2.rectangle(panel, (0, 0), (PANEL_W, 40), COLOR_CARD, -1)
    cv2.putText(panel, "TDOA Spatial Tracker v5.0", (15, 27),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, COLOR_TEXT, 1)
    
    # 상태 LED
    status_color = COLOR_ACCENT if energy > ENERGY_THRESHOLD else COLOR_INACTIVE
    cv2.circle(panel, (PANEL_W - 25, 20), 8, status_color, -1)
    
    y_offset = 55
    
    # === 방향 카드 ===
    draw_rounded_rect(panel, (10, y_offset), (PANEL_W - 10, y_offset + 90), COLOR_CARD, 6)
    cv2.putText(panel, "DIRECTION", (20, y_offset + 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, COLOR_TEXT_DIM, 1)
    
    cv2.putText(panel, "Az", (20, y_offset + 50),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, COLOR_TEXT_DIM, 1)
    cv2.putText(panel, f"{az:+06.1f}", (55, y_offset + 52),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, COLOR_ACCENT, 2)
    
    cv2.putText(panel, "El", (20, y_offset + 75),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, COLOR_TEXT_DIM, 1)
    cv2.putText(panel, f"{el:+06.1f}", (55, y_offset + 77),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, COLOR_ACCENT, 2)
    
    # 미니 컴파스
    cx, cy = PANEL_W - 70, y_offset + 50
    r = 30
    cv2.circle(panel, (cx, cy), r, COLOR_INACTIVE, 1)
    angle_rad = np.radians(-az)
    tx = int(cx + (r - 5) * np.cos(angle_rad))
    ty = int(cy + (r - 5) * np.sin(angle_rad))
    cv2.line(panel, (cx, cy), (tx, ty), COLOR_ACCENT, 2)
    cv2.circle(panel, (tx, ty), 4, COLOR_ACCENT, -1)
    
    y_offset += 100
    
    # === 거리 카드 ===
    draw_rounded_rect(panel, (10, y_offset), (PANEL_W - 10, y_offset + 85), COLOR_CARD, 6)
    cv2.putText(panel, "DISTANCE ESTIMATE", (20, y_offset + 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, COLOR_TEXT_DIM, 1)
    
    dist_color = analyzer.get_distance_color()
    cv2.putText(panel, analyzer.distance_estimate, (20, y_offset + 50),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, dist_color, 2)
    cv2.putText(panel, f"~{analyzer.distance_meters:.1f}m", (150, y_offset + 50),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, COLOR_TEXT_DIM, 1)
    
    # 거리 바
    bar_x, bar_y = 20, y_offset + 62
    bar_w = PANEL_W - 50
    cv2.rectangle(panel, (bar_x, bar_y), (bar_x + bar_w, bar_y + 12), COLOR_INACTIVE, -1)
    
    # 거리에 따른 바 위치 (0~10m 범위)
    dist_ratio = np.clip(analyzer.distance_meters / 10.0, 0.0, 1.0)
    marker_x = int(bar_x + dist_ratio * bar_w)
    cv2.rectangle(panel, (bar_x, bar_y), (marker_x, bar_y + 12), dist_color, -1)
    cv2.circle(panel, (marker_x, bar_y + 6), 6, dist_color, -1)
    
    # 거리 눈금
    for i, label in enumerate(["0m", "5m", "10m"]):
        lx = bar_x + int(i * bar_w / 2)
        cv2.putText(panel, label, (lx - 8, bar_y + 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.3, COLOR_TEXT_DIM, 1)
    
    y_offset += 95
    
    # === 공간 분석 카드 ===
    draw_rounded_rect(panel, (10, y_offset), (PANEL_W - 10, y_offset + 100), COLOR_CARD, 6)
    cv2.putText(panel, "SPATIAL ANALYSIS", (20, y_offset + 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, COLOR_TEXT_DIM, 1)
    
    # 반향 지수
    cv2.putText(panel, "Reverb:", (20, y_offset + 45),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, COLOR_TEXT_DIM, 1)
    reverb_text = "Direct" if analyzer.reverb_index > 1.5 else "Mixed" if analyzer.reverb_index > 1.0 else "Reverb"
    reverb_color = COLOR_ACCENT if analyzer.reverb_index > 1.5 else COLOR_WARN if analyzer.reverb_index > 1.0 else COLOR_DANGER
    cv2.putText(panel, f"{reverb_text} ({analyzer.reverb_index:.2f})", (90, y_offset + 45),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, reverb_color, 1)
    
    # 이동 상태
    cv2.putText(panel, "Motion:", (20, y_offset + 68),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, COLOR_TEXT_DIM, 1)
    motion_status, motion_color = analyzer.get_movement_status()
    cv2.putText(panel, f"{motion_status} ({analyzer.movement_speed:.1f} deg/s)", (90, y_offset + 68),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, motion_color, 1)
    
    # 위치 확실성
    cv2.putText(panel, "Certainty:", (20, y_offset + 91),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, COLOR_TEXT_DIM, 1)
    cert_pct = int(analyzer.position_certainty * 100)
    cert_color = COLOR_ACCENT if cert_pct > 70 else COLOR_WARN if cert_pct > 40 else COLOR_DANGER
    cv2.putText(panel, f"{cert_pct}%", (100, y_offset + 91),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, cert_color, 1)
    
    # 확실성 바
    cert_bar_x = 140
    cert_bar_w = PANEL_W - 160
    cv2.rectangle(panel, (cert_bar_x, y_offset + 80), (cert_bar_x + cert_bar_w, y_offset + 92), COLOR_INACTIVE, -1)
    cv2.rectangle(panel, (cert_bar_x, y_offset + 80), 
                  (cert_bar_x + int(analyzer.position_certainty * cert_bar_w), y_offset + 92), cert_color, -1)
    
    y_offset += 110
    
    # === 에너지 바 ===
    draw_rounded_rect(panel, (10, y_offset), (PANEL_W - 10, y_offset + 50), COLOR_CARD, 6)
    cv2.putText(panel, "AUDIO LEVEL", (20, y_offset + 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, COLOR_TEXT_DIM, 1)
    
    bar_x, bar_y = 20, y_offset + 30
    bar_w = PANEL_W - 50
    bar_h = 12
    cv2.rectangle(panel, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h), COLOR_INACTIVE, -1)
    
    energy_norm = np.clip(energy / 5000.0, 0.0, 1.0)
    bar_color = analyzer.get_distance_color()
    cv2.rectangle(panel, (bar_x, bar_y), (bar_x + int(energy_norm * bar_w), bar_y + bar_h), bar_color, -1)
    
    cv2.putText(panel, f"{energy:.0f}", (PANEL_W - 60, y_offset + 42),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, COLOR_TEXT_DIM, 1)
    
    y_offset += 60
    
    # === 히스토리 그래프 ===
    draw_rounded_rect(panel, (10, y_offset), (PANEL_W - 10, y_offset + 100), COLOR_CARD, 6)
    cv2.putText(panel, "POSITION HISTORY", (20, y_offset + 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, COLOR_TEXT_DIM, 1)
    
    gx, gy = 20, y_offset + 28
    gw, gh = PANEL_W - 50, 60
    
    cv2.rectangle(panel, (gx, gy), (gx + gw, gy + gh), (20, 20, 25), -1)
    cv2.line(panel, (gx, gy + gh//2), (gx + gw, gy + gh//2), COLOR_INACTIVE, 1)
    
    # Az history (초록)
    if len(analyzer.az_history) > 1:
        az_list = list(analyzer.az_history)
        pts = []
        for i, v in enumerate(az_list):
            nx = int(gx + i * gw / max(1, len(az_list) - 1))
            ny = int(gy + gh/2 - (v / 90.0) * (gh/2))
            ny = np.clip(ny, gy + 2, gy + gh - 2)
            pts.append((nx, ny))
        for i in range(len(pts) - 1):
            cv2.line(panel, pts[i], pts[i+1], COLOR_ACCENT, 2)
    
    # El history (주황)
    if len(analyzer.el_history) > 1:
        el_list = list(analyzer.el_history)
        pts = []
        for i, v in enumerate(el_list):
            nx = int(gx + i * gw / max(1, len(el_list) - 1))
            ny = int(gy + gh/2 - (v / 90.0) * (gh/2))
            ny = np.clip(ny, gy + 2, gy + gh - 2)
            pts.append((nx, ny))
        for i in range(len(pts) - 1):
            cv2.line(panel, pts[i], pts[i+1], COLOR_ACCENT2, 2)
    
    # 범례
    cv2.circle(panel, (gx + gw - 45, y_offset + 16), 4, COLOR_ACCENT, -1)
    cv2.putText(panel, "Az", (gx + gw - 38, y_offset + 19), cv2.FONT_HERSHEY_SIMPLEX, 0.3, COLOR_TEXT_DIM, 1)
    cv2.circle(panel, (gx + gw - 15, y_offset + 16), 4, COLOR_ACCENT2, -1)
    cv2.putText(panel, "El", (gx + gw - 8, y_offset + 19), cv2.FONT_HERSHEY_SIMPLEX, 0.3, COLOR_TEXT_DIM, 1)
    
    # === 푸터 ===
    cv2.putText(panel, "ESC: Exit | S: Snapshot", (15, WIN_H - 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, COLOR_TEXT_DIM, 1)

def draw_video_overlay(frame, az, el, analyzer):
    h, w = frame.shape[:2]
    
    # 타겟 위치 계산
    px = int(w/2 + (az / (FOV_H/2)) * (w/2))
    py = int(h/2 - (el / (FOV_V/2)) * (h/2))
    px = np.clip(px, 30, w - 30)
    py = np.clip(py, 30, h - 30)
    
    # 거리에 따른 색상
    target_color = analyzer.get_distance_color()
    
    # 확실성에 따른 원 크기
    base_radius = 15
    certainty_radius = int(base_radius + (1 - analyzer.position_certainty) * 30)
    
    # 불확실성 링 (희미하게)
    if analyzer.position_certainty < 0.8:
        cv2.circle(frame, (px, py), certainty_radius, (*target_color[:3],), 1)
    
    # 메인 타겟
    cv2.line(frame, (px - 25, py), (px - 8, py), target_color, 2)
    cv2.line(frame, (px + 8, py), (px + 25, py), target_color, 2)
    cv2.line(frame, (px, py - 25), (px, py - 8), target_color, 2)
    cv2.line(frame, (px, py + 8), (px, py + 25), target_color, 2)
    cv2.circle(frame, (px, py), base_radius, target_color, 2)
    cv2.circle(frame, (px, py), 4, target_color, -1)
    
    # 거리 라벨
    dist_label = f"{analyzer.distance_estimate} ~{analyzer.distance_meters:.1f}m"
    cv2.putText(frame, dist_label, (px + 20, py - 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, target_color, 1)
    
    # 하단 정보바
    cv2.rectangle(frame, (0, h - 35), (w, h), (0, 0, 0), -1)
    info_text = f"Az:{az:+06.1f}  El:{el:+06.1f}  Dist:{analyzer.distance_meters:.1f}m  Cert:{int(analyzer.position_certainty*100)}%"
    cv2.putText(frame, info_text, (10, h - 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLOR_TEXT, 1)
    
    # 이동 상태
    motion_status, motion_color = analyzer.get_movement_status()
    cv2.putText(frame, motion_status, (w - 80, h - 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, motion_color, 1)

# -----------------------------
# 메인
# -----------------------------
def main():
    print("=" * 50)
    print("  TDOA Spatial Tracker v5.0")
    print("  Advanced distance & spatial analysis")
    print("=" * 50)

    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.02)
    except Exception as e:
        print("Serial error:", e)
        return

    time.sleep(0.2)
    ser.reset_input_buffer()

    cap = cv2.VideoCapture(CAM_ID)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, VID_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, VID_H)
    if not cap.isOpened():
        print("Camera error")
        ser.close()
        return

    serial_buf = bytearray()
    mics = [deque(maxlen=FRAME_SIZE*2) for _ in range(4)]

    az_smooth = 0.0
    el_smooth = 0.0
    energy = 0.0
    
    analyzer = SpatialAnalyzer()

    win_name = "TDOA Spatial Tracker v5.0"
    cv2.namedWindow(win_name, cv2.WINDOW_AUTOSIZE)

    try:
        while True:
            data = ser.read(ser.in_waiting or 1)
            if data:
                serial_buf.extend(data)

            while len(serial_buf) >= PACKET_SIZE:
                idx = serial_buf.find(HEADER)
                if idx == -1:
                    serial_buf = serial_buf[-1:]
                    break
                if idx > 0:
                    serial_buf = serial_buf[idx:]
                if len(serial_buf) < PACKET_SIZE:
                    break
                pkt = bytes(serial_buf[:PACKET_SIZE])
                serial_buf = serial_buf[PACKET_SIZE:]
                try:
                    m1, m2, m3, m4 = struct.unpack('<hhhh', pkt[2:])
                except struct.error:
                    continue
                mics[0].append(m1)
                mics[1].append(m2)
                mics[2].append(m3)
                mics[3].append(m4)

            if len(mics[0]) >= FRAME_SIZE:
                sig1 = np.array(list(mics[0])[-FRAME_SIZE:], dtype=np.float64)
                sig2 = np.array(list(mics[1])[-FRAME_SIZE:], dtype=np.float64)
                sig3 = np.array(list(mics[2])[-FRAME_SIZE:], dtype=np.float64)
                sig4 = np.array(list(mics[3])[-FRAME_SIZE:], dtype=np.float64)

                energy = np.sqrt(np.mean(((sig1 + sig2 + sig3 + sig4) / 4.0) ** 2))

                if energy > ENERGY_THRESHOLD:
                    az, el, tdoa_h, tdoa_v, confidences, sharpness = estimate_direction(sig1, sig2, sig3, sig4)
                    
                    avg_conf = sum(confidences) / 4.0
                    
                    if avg_conf >= CONF_THRESHOLD:
                        # 스무딩
                        max_step_h, max_step_v = 8.0, 6.0
                        az_diff = np.clip(az - az_smooth, -max_step_h, max_step_h)
                        el_diff = np.clip(el - el_smooth, -max_step_v, max_step_v)
                        az_smooth += ALPHA_H * az_diff
                        el_smooth += ALPHA_V * el_diff
                        
                        # 공간 분석 업데이트
                        analyzer.update(az_smooth, el_smooth, energy, confidences, sharpness)
                    
                    if DEBUG_PRINT:
                        print(f"Az:{az_smooth:+.1f} El:{el_smooth:+.1f} "
                              f"Dist:{analyzer.distance_meters:.1f}m Sharp:{sharpness:.2f}")

                for m in mics:
                    for _ in range(HOP_SIZE):
                        if m:
                            m.popleft()

            ret, frame = cap.read()
            if not ret:
                break

            frame = cv2.resize(frame, (VID_W, VID_H))
            draw_video_overlay(frame, az_smooth, el_smooth, analyzer)

            panel = np.zeros((WIN_H, PANEL_W, 3), dtype=np.uint8)
            draw_panel(panel, az_smooth, el_smooth, energy, analyzer)

            canvas = np.zeros((WIN_H, WIN_W, 3), dtype=np.uint8)
            canvas[:, :VID_W] = frame
            canvas[:, VID_W:] = panel

            cv2.imshow(win_name, canvas)
            key = cv2.waitKey(1) & 0xFF
            if key == 27:
                break
            elif key == ord('s'):
                fname = f"tdoa_v5_{int(time.time())}.png"
                cv2.imwrite(fname, canvas)
                print(f"Saved: {fname}")

    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        ser.close()
        cv2.destroyAllWindows()
        print("Exit")

if __name__ == "__main__":
    main()