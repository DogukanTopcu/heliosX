"""
v2 — Fly detection (motion + blackhat + adaptive + trajectory)
Raspberry Pi 5 + Camera Module 3, CPU-only.
Integrated with Pan-Tilt Smooth Tracking & Smart Laser Lock (3-Second Rule).
"""

import math
import time
import threading
import os
import logging
from collections import deque
from dataclasses import dataclass, field
from http import server
from socketserver import ThreadingMixIn

import cv2
import numpy as np
from picamera2 import Picamera2

# Raspberry Pi 5 için yerel lgpio sürücüsünü zorunlu kılıyoruz
os.environ["GPIOZERO_PIN_FACTORY"] = "lgpio"

try:
    from gpiozero import Servo, LED
    HAS_GPIO = True
except Exception:
    HAS_GPIO = False


# =========================================================================
# AYARLAR (sahnene gore ayarla)
# =========================================================================

# Kamera
CAPTURE_W, CAPTURE_H = 1280, 720
PROCESS_W, PROCESS_H = 640, 360       # detection bu cozunurlukte (hizli)
TARGET_FPS = 30

# Kisa pozlama -> hizli sinek motion blur'suz net nokta
EXPOSURE_TIME_US = 5000               # mikrosaniye (5ms shutter). 0 = auto
ANALOGUE_GAIN = 6.0                   # isiklandirma kompansasyonu (1-16)

# Adaptif pozlama — loş/değişken ışık ortamı için otomatik ayar
BRIGHTNESS_TARGET  = 80               # hedef ortalama frame parlaklığı (0-255)
BRIGHTNESS_WINDOW  = 90               # her 90 frame'de (~3 sn) bir güncelle
EXPOSURE_MIN_US    = 3000             # minimum shutter (hareket bulanıklığı sınırı)
EXPOSURE_MAX_US    = 25000            # maksimum shutter (25ms)
GAIN_MIN           = 2.0
GAIN_MAX           = 12.0

# Preprocessing
BLUR_KERNEL = 5
CLAHE_CLIP = 2.0
CLAHE_GRID = (8, 8)

# Motion (frame diff)
MOTION_THRESHOLD = 20                 # piksel parlaklik degisimi esigi
MOTION_DILATE = 2

# Black-hat (dark blob highlight)
BLACKHAT_KERNEL = 17                  # daha buyuk -> motion blur'lu sinegi de yakalar
BLACK_THRESHOLD = 18                  # dusurursen daha cok aday

# Adaptive (local-dark)
ADAPTIVE_BLOCK = 21
ADAPTIVE_C = 9

# Detection size / shape (process resolution)
MIN_AREA = 8
MAX_AREA = 200
MIN_WH = 2
MAX_WH = 22
MIN_ASPECT = 0.25
MAX_ASPECT = 4.0

# Per-detection scoring
MIN_MOTION_SCORE = 0.12               # bbox icindeki motion piksel orani
MIN_DARK_SCORE = 18.0                 # blackhat ortalamasi
MAX_LOCAL_MEAN = 200.0                # cok parlak yerde sinek olmaz (ust limit)

# Global motion suppression
MAX_GLOBAL_MOTION_RATIO = 0.04        # frame'in %4'unden cok hareket -> el/kol
MAX_TOTAL_DETECTIONS = 25             # cok sayida aday -> sahne kaotik, bastir

# Bolgesel hariclendirme (insan/parmak/el icin)
LARGE_MOTION_AREA = 4000              # px
LARGE_MOTION_DIM = 110                # bbox max(w,h)
EXCLUSION_PADDING = 20
EXCLUSION_DILATE_ITERS = 4
EXCLUSION_DILATE_KERNEL = 5

# Tracking
MATCH_DISTANCE = 90                   # px (process scale), hizli sinek icin genis
MAX_MISSED = 12                       # track tek frame'i kacirsa olmesin
MIN_HITS = 2                          # iki onay yeter
CONFIRM_FRAMES = 1                    # ekstra bekleme yok
TRAJECTORY_WINDOW = 10                # son N pozisyon trajectory icin
MIN_PATH_LENGTH = 4.0                 # yavas sinek bile gecsin, edge titremesi gecemez
MIN_DIR_CHANGES = 0                   # duz ucan sinek de gecsin (path_length yeter)

# Tetikleme & Donanım Pin Ayarları
TRIGGER_COOLDOWN = 2.0
TRIGGER_PIN = 17                      # Orijinal röle/tetik pini
PAN_PIN = 12                          # Servo Pan (Sağ-Sol) GPIO pini
TILT_PIN = 13                         # Servo Tilt (Yukarı-Aşağı) GPIO pini
LAZER_PIN = 14                        # Lazer Modülü GPIO pini

# Servo mekanik hareket sınırları — manual-control5.py ile fiziksel limitleri test edip ayarla
PAN_MIN_DEG  = 30.0
PAN_MAX_DEG  = 150.0
TILT_MIN_DEG = 45.0
TILT_MAX_DEG = 135.0

# Adaptif servo kazanımı — hata büyükse hızlı, küçükse hassas
KP_BASE          = 0.020   # küçük hata (<5 px) — hassas ince ayar modu
KP_BOOST         = 0.060   # büyük hata (>=BOOST_ERR px) — hızlı yakalama modu
KP_BOOST_ERROR_PX = 80.0   # bu piksel hatasından itibaren KP_BOOST'a geçilir

# Lazer-Kamera fiziksel offset (piksel, 640×360 process çözünürlüğünde)
# Pozitif X: lazer kamera merkezinin sağında; Pozitif Y: lazer kamera merkezinin altında
# Kalibrasyon sonrası calibration.json'dan otomatik yüklenir, burayı manuel değiştirme
LASER_OFFSET_PX_X = 0.0
LASER_OFFSET_PX_Y = 0.0
CALIB_PATH = "/home/heliosx/v2/calibration.json"
# Camera Module 3, 1280×720 modunda yaklaşık FoV değerleri
CAMERA_FOV_H_DEG = 66.0
CAMERA_FOV_V_DEG = 49.0

# Stream / log
ENABLE_STREAM = True
STREAM_PORT = 8080
STREAM_QUALITY = 75
WARMUP_FRAMES = 15                    # detection bu kadar frame sonra basla
LOG_TO_FILE = True
LOG_PATH = "/home/heliosx/v2/detections.log"

# =========================================================================
# LOGGING KURULUMU (sabitlerden sonra — LOG_TO_FILE ve LOG_PATH burada okunuyor)
# =========================================================================
import logging.handlers as _log_handlers

_log_fmt = logging.Formatter('[%(asctime)s] %(message)s', datefmt='%Y-%m-%d %H:%M:%S')
_logger = logging.getLogger("turret")
_logger.setLevel(logging.INFO)

_sh = logging.StreamHandler()
_sh.setFormatter(_log_fmt)
_logger.addHandler(_sh)

if LOG_TO_FILE:
    _fh = _log_handlers.RotatingFileHandler(
        LOG_PATH, maxBytes=5 * 1024 * 1024, backupCount=3, encoding="utf-8"
    )
    _fh.setFormatter(_log_fmt)
    _logger.addHandler(_fh)

def log(msg: str) -> None:
    _logger.info(msg)


import json as _json
import datetime as _dt

_current_exposure: float = float(EXPOSURE_TIME_US)
_current_gain: float = ANALOGUE_GAIN


def _adjust_exposure(cam, brightness: float) -> None:
    global _current_exposure, _current_gain
    if EXPOSURE_TIME_US == 0:
        return  # otomatik AE açıksa dokunma

    error = BRIGHTNESS_TARGET - brightness  # pozitif = çok karanlık

    if error > 10:
        _current_exposure = min(_current_exposure * 1.15, EXPOSURE_MAX_US)
    elif error < -10:
        _current_exposure = max(_current_exposure * 0.87, EXPOSURE_MIN_US)

    if _current_exposure >= EXPOSURE_MAX_US and error > 15:
        _current_gain = min(_current_gain * 1.1, GAIN_MAX)
    elif _current_exposure <= EXPOSURE_MIN_US and error < -15:
        _current_gain = max(_current_gain * 0.9, GAIN_MIN)

    cam.set_controls({
        "ExposureTime": int(_current_exposure),
        "AnalogueGain": _current_gain,
    })
    log(f"[EXPOSURE] parlaklık={brightness:.0f} exp={_current_exposure:.0f}µs gain={_current_gain:.1f}")


def _load_calibration() -> None:
    global LASER_OFFSET_PX_X, LASER_OFFSET_PX_Y
    if not os.path.exists(CALIB_PATH):
        log("Kalibrasyon dosyası yok, offset=(0,0) kullanılıyor.")
        return
    try:
        with open(CALIB_PATH) as f:
            data = _json.load(f)
        LASER_OFFSET_PX_X = float(data.get("laser_offset_px_x", 0.0))
        LASER_OFFSET_PX_Y = float(data.get("laser_offset_px_y", 0.0))
        log(f"Kalibrasyon yüklendi: offset=({LASER_OFFSET_PX_X:.1f}, {LASER_OFFSET_PX_Y:.1f})px"
            f" [{data.get('calibrated_at', '?')}]")
    except Exception as e:
        log(f"Kalibrasyon dosyası okunamadı: {e} — offset=(0,0) kullanılıyor.")


def _save_calibration(offset_px_x: float, offset_px_y: float,
                      distance_cm: int = 150) -> None:
    deg_per_px_h = CAMERA_FOV_H_DEG / PROCESS_W
    deg_per_px_v = CAMERA_FOV_V_DEG / PROCESS_H
    data = {
        "laser_offset_px_x":       round(offset_px_x, 2),
        "laser_offset_px_y":       round(offset_px_y, 2),
        "laser_offset_deg_x":      round(offset_px_x * deg_per_px_h, 3),
        "laser_offset_deg_y":      round(offset_px_y * deg_per_px_v, 3),
        "calibration_distance_cm": distance_cm,
        "calibrated_at":           _dt.datetime.now().isoformat(timespec="seconds"),
        "process_resolution":      f"{PROCESS_W}x{PROCESS_H}",
    }
    with open(CALIB_PATH, "w") as f:
        _json.dump(data, f, indent=2)
    log(f"[KALİBRASYON] Kaydedildi: offset=({offset_px_x:.1f}, {offset_px_y:.1f})px "
        f"= ({data['laser_offset_deg_x']:.2f}°, {data['laser_offset_deg_y']:.2f}°) "
        f"@ {distance_cm}cm")


# =========================================================================
# DONANIM VE MOTOR GLOBAL DEĞİŞKENLERİ
# =========================================================================
data_lock = threading.Lock()
is_running = True

# Motor Açı Kontrolü (Başlangıç: 90 derece / Tam Merkez)
target_pan_deg = 90.0
target_tilt_deg = 90.0
current_pan_deg = 90.0
current_tilt_deg = 90.0

# Akıllı Lazer Takip Durum Makinesi Değişkenleri
current_target_id = None    # Şu an kilitlenilen sineğin benzersiz ID'si
lock_start_time = None      # Kilitlenme anının zaman damgası
killed_flies = set()        # 3 saniye boyunca vurularak imha edilen sineklerin ID listesi

# Kalibrasyon modu
calibrating: bool = False
calib_click_px: tuple[float, float] | None = None

# Web handler'ların thread-safe okuyabileceği durum snapshot'ı
web_status: dict = {}

# Nesne tanımlayıcı placeholder'lar (Main içinde global olarak set edilecek)
pan_servo = None
tilt_servo = None
lazer = None

def deg_to_servo_val(deg):
    """ 0-180 dereceyi gpiozero'nun -1.0 ile 1.0 skalasına çevirir """
    return (deg / 90.0) - 1.0

def adaptive_kp(error_px: float) -> float:
    t = min(abs(error_px) / KP_BOOST_ERROR_PX, 1.0)
    return KP_BASE + t * (KP_BOOST - KP_BASE)

def ensure_odd(n: int, minimum: int = 3) -> int:
    n = max(n, minimum)
    return n if n % 2 == 1 else n + 1

# =========================================================================
# Detection Dataclasses & Classes
# =========================================================================

@dataclass
class Detection:
    cx: int
    cy: int
    x: int
    y: int
    w: int
    h: int
    area: float
    motion_score: float
    dark_score: float


@dataclass
class Track:
    track_id: int
    cx: float
    cy: float
    vx: float = 0.0
    vy: float = 0.0
    hits: int = 1
    misses: int = 0
    age: int = 1
    first_frame: int = 0
    triggered: bool = False
    history: deque = field(default_factory=lambda: deque(maxlen=TRAJECTORY_WINDOW))
    last_score: float = 0.0
    kf: object = field(default=None, repr=False, compare=False)

    def __post_init__(self):
        self.kf = cv2.KalmanFilter(4, 2)
        dt = 1.0
        self.kf.transitionMatrix = np.array([
            [1, 0, dt, 0],
            [0, 1,  0, dt],
            [0, 0,  1,  0],
            [0, 0,  0,  1],
        ], dtype=np.float32)
        self.kf.measurementMatrix = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
        ], dtype=np.float32)
        self.kf.processNoiseCov = np.eye(4, dtype=np.float32) * 1e-2
        self.kf.processNoiseCov[2, 2] = 5.0
        self.kf.processNoiseCov[3, 3] = 5.0
        self.kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * 4.0
        self.kf.errorCovPost = np.eye(4, dtype=np.float32)
        self.kf.statePost = np.array(
            [[self.cx], [self.cy], [0.0], [0.0]], dtype=np.float32
        )
        self._pred_cx: float = self.cx
        self._pred_cy: float = self.cy

    def kalman_predict(self) -> tuple[float, float]:
        pred = self.kf.predict()
        return (float(pred[0]), float(pred[1]))

    def predict(self) -> tuple[float, float]:
        return (self._pred_cx, self._pred_cy)

    def speed(self) -> float:
        return math.hypot(self.vx, self.vy)

    def path_length(self) -> float:
        if len(self.history) < 2:
            return 0.0
        total = 0.0
        for i in range(1, len(self.history)):
            x0, y0 = self.history[i - 1]
            x1, y1 = self.history[i]
            total += math.hypot(x1 - x0, y1 - y0)
        return total

    def direction_changes(self, min_angle_deg: float = 45.0) -> int:
        if len(self.history) < 3:
            return 0
        min_cos = math.cos(math.radians(min_angle_deg))
        changes = 0
        for i in range(2, len(self.history)):
            x0, y0 = self.history[i - 2]
            x1, y1 = self.history[i - 1]
            x2, y2 = self.history[i]
            v1x, v1y = x1 - x0, y1 - y0
            v2x, v2y = x2 - x1, y2 - y1
            n1 = math.hypot(v1x, v1y)
            n2 = math.hypot(v2x, v2y)
            if n1 < 0.5 or n2 < 0.5:
                continue
            cos = (v1x * v2x + v1y * v2y) / (n1 * n2)
            if cos < min_cos:
                changes += 1
        return changes


class Tracker:
    def __init__(self) -> None:
        self.next_id = 1
        self.tracks: dict[int, Track] = {}

    def update(self, dets: list[Detection], frame_idx: int) -> None:
        for tr in self.tracks.values():
            px, py = tr.kalman_predict()
            tr._pred_cx = px
            tr._pred_cy = py

        unmatched_tracks = set(self.tracks.keys())
        unmatched_dets = set(range(len(dets)))

        pairs: list[tuple[float, int, int]] = []
        for di, d in enumerate(dets):
            for tid, tr in self.tracks.items():
                px, py = tr.predict()
                dist = math.hypot(px - d.cx, py - d.cy)
                if dist <= MATCH_DISTANCE:
                    bonus = d.dark_score * 0.1 + d.motion_score * 10.0
                    pairs.append((dist - bonus, tid, di))
        pairs.sort(key=lambda p: p[0])

        for _, tid, di in pairs:
            if tid not in unmatched_tracks or di not in unmatched_dets:
                continue
            unmatched_tracks.remove(tid)
            unmatched_dets.remove(di)
            d = dets[di]
            tr = self.tracks[tid]
            meas = np.array([[float(d.cx)], [float(d.cy)]], dtype=np.float32)
            corrected = tr.kf.correct(meas)
            tr.cx = float(corrected[0])
            tr.cy = float(corrected[1])
            tr.vx = float(corrected[2])
            tr.vy = float(corrected[3])
            tr.hits += 1
            tr.age += 1
            tr.misses = 0
            tr.history.append((tr.cx, tr.cy))
            tr.last_score = d.motion_score + d.dark_score / 100.0

        for di in unmatched_dets:
            d = dets[di]
            tr = Track(
                track_id=self.next_id,
                cx=float(d.cx),
                cy=float(d.cy),
                first_frame=frame_idx,
            )
            tr.history.append((tr.cx, tr.cy))
            self.tracks[self.next_id] = tr
            self.next_id += 1

        expired = []
        for tid in unmatched_tracks:
            tr = self.tracks[tid]
            tr.misses += 1
            tr.age += 1
            tr.cx = tr._pred_cx
            tr.cy = tr._pred_cy
            tr.vx = float(tr.kf.statePost[2])
            tr.vy = float(tr.kf.statePost[3])
            if tr.misses > MAX_MISSED:
                expired.append(tid)
        for tid in expired:
            self.tracks.pop(tid, None)

    def confirmed(self, frame_idx: int) -> list[Track]:
        out = []
        for tr in self.tracks.values():
            if tr.misses > 0:
                continue
            if tr.hits < MIN_HITS:
                continue
            if (frame_idx - tr.first_frame) < CONFIRM_FRAMES:
                continue
            if tr.path_length() < MIN_PATH_LENGTH:
                continue
            if tr.direction_changes() < MIN_DIR_CHANGES:
                continue
            out.append(tr)
        return out


def candidate_score(tr: "Track") -> float:
    dist = math.hypot(tr.cx - PROCESS_W / 2, tr.cy - PROCESS_H / 2)
    max_dist = math.hypot(PROCESS_W / 2, PROCESS_H / 2)
    proximity = 1.0 - (dist / max_dist)
    return tr.hits * 3.0 + tr.path_length() * 0.5 + proximity * 20.0 + tr.last_score * 5.0


# =========================================================================
# Mask pipeline
# =========================================================================

class MaskPipeline:
    def __init__(self) -> None:
        self.prev_blur: np.ndarray | None = None
        self.clahe = cv2.createCLAHE(clipLimit=CLAHE_CLIP, tileGridSize=CLAHE_GRID)
        self.bh_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (ensure_odd(BLACKHAT_KERNEL, 5), ensure_odd(BLACKHAT_KERNEL, 5)),
        )
        self.k3 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        self.k_exclude = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (ensure_odd(EXCLUSION_DILATE_KERNEL, 3),
             ensure_odd(EXCLUSION_DILATE_KERNEL, 3)),
        )

    def process(self, gray: np.ndarray):
        blur = cv2.GaussianBlur(gray, (ensure_odd(BLUR_KERNEL), ensure_odd(BLUR_KERNEL)), 0)
        enhanced = self.clahe.apply(blur)

        if self.prev_blur is None:
            motion_raw = np.zeros_like(gray)
        else:
            delta = cv2.absdiff(blur, self.prev_blur)
            _, motion_raw = cv2.threshold(delta, MOTION_THRESHOLD, 255, cv2.THRESH_BINARY)
        self.prev_blur = blur.copy()

        motion_clean = cv2.morphologyEx(motion_raw, cv2.MORPH_OPEN, self.k3, iterations=1)

        blackhat = cv2.morphologyEx(enhanced, cv2.MORPH_BLACKHAT, self.bh_kernel)
        _, dark = cv2.threshold(blackhat, BLACK_THRESHOLD, 255, cv2.THRESH_BINARY)

        adaptive = cv2.adaptiveThreshold(
            enhanced, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV,
            ensure_odd(ADAPTIVE_BLOCK, 5), ADAPTIVE_C,
        )

        motion_dil = cv2.dilate(motion_clean, self.k3, iterations=MOTION_DILATE)
        combined = cv2.bitwise_and(motion_dil, dark)
        combined = cv2.bitwise_and(combined, adaptive)
        combined = cv2.morphologyEx(combined, cv2.MORPH_OPEN, self.k3, iterations=1)
        combined = cv2.dilate(combined, self.k3, iterations=1)

        motion_exclude = cv2.dilate(
            motion_clean, self.k_exclude, iterations=EXCLUSION_DILATE_ITERS
        )
        return enhanced, motion_dil, motion_exclude, blackhat, combined


# =========================================================================
# MJPEG Stream Sunucusu
# =========================================================================

class FrameBuffer:
    def __init__(self) -> None:
        self.frame: bytes | None = None
        self.cond = threading.Condition()

    def update(self, b: bytes) -> None:
        with self.cond:
            self.frame = b
            self.cond.notify_all()


_NAV = """
<nav>
  <span class="brand">🎯 Turret</span>
  <a href="/control" id="nav-control">Kontrol</a>
  <a href="/debug"   id="nav-debug">Debug Mask</a>
  <a href="/calibrate" id="nav-cal">Kalibrasyon</a>
</nav>"""

_NAV_CSS = """
*{box-sizing:border-box;margin:0;padding:0}
body{background:#0d0d0d;color:#e0e0e0;font-family:system-ui,sans-serif;height:100vh;display:flex;flex-direction:column}
nav{background:#161616;border-bottom:1px solid #2a2a2a;padding:10px 16px;display:flex;align-items:center;gap:8px;flex-shrink:0}
.brand{font-size:15px;font-weight:600;color:#fff;margin-right:auto}
nav a{color:#888;text-decoration:none;padding:6px 12px;border-radius:5px;font-size:13px}
nav a:hover{background:#2a2a2a;color:#fff}
nav a.active{background:#252525;color:#5af}
"""

PAGE = """<!DOCTYPE html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Turret — Stream</title>
<style>""" + _NAV_CSS + """
.stream-wrap{flex:1;background:#000;display:flex;align-items:center;justify-content:center;overflow:hidden}
.stream-wrap img{max-width:100%;max-height:100%;object-fit:contain}
</style></head><body>""" + _NAV + """
<div class="stream-wrap"><img src="/stream.mjpg"/></div>
<script>document.getElementById('nav-control').classList.add('active')</script>
</body></html>"""

DEBUG_PAGE = """<!DOCTYPE html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Turret — Debug</title>
<style>""" + _NAV_CSS + """
.content{flex:1;display:flex;flex-direction:column;align-items:center;justify-content:center;background:#000;gap:10px}
.content img{max-width:100%;max-height:calc(100% - 40px);object-fit:contain}
.hint{font-size:12px;color:#555}
</style></head><body>""" + _NAV + """
<div class="content">
  <img src="/debug.mjpg"/>
  <p class="hint">Beyaz alan: deteksiyon pipeline birleşik maskesi (hareket + karanlık + adaptif)</p>
</div>
<script>document.getElementById('nav-debug').classList.add('active')</script>
</body></html>"""

CONTROL_PAGE = """<!DOCTYPE html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Turret — Kontrol</title>
<style>
""" + _NAV_CSS + """
a{color:#5af;text-decoration:none}
.layout{flex:1;display:flex;overflow:hidden}
.stream-panel{flex:1;background:#000;display:flex;align-items:center;justify-content:center;min-width:0}
.stream-panel img{max-width:100%;max-height:100%;object-fit:contain}
.sidebar{width:260px;background:#111;border-left:1px solid #222;overflow-y:auto;display:flex;flex-direction:column;gap:10px;padding:12px;flex-shrink:0}
.card{background:#191919;border:1px solid #252525;border-radius:8px;padding:12px}
.card h3{font-size:10px;font-weight:600;color:#555;text-transform:uppercase;letter-spacing:.1em;margin-bottom:10px}
.stat{display:flex;justify-content:space-between;align-items:center;padding:3px 0;font-size:13px}
.stat-label{color:#666}
.stat-value{color:#ccc;font-variant-numeric:tabular-nums}
.laser-on{color:#f44;font-weight:700}
.laser-off{color:#444}
.btn{display:block;width:100%;background:#222;color:#bbb;border:1px solid #333;border-radius:6px;padding:9px 12px;margin-bottom:6px;cursor:pointer;font-size:13px;text-align:left;transition:background .15s}
.btn:hover{background:#2c2c2c;color:#fff}
.btn:last-child{margin-bottom:0}
.btn-danger{border-color:#5a1515;color:#f77}
.btn-danger:hover{background:#2a1010}
.btn-ok{border-color:#1a4a1a;color:#7d7}
.btn-ok:hover{background:#0f200f}
.calib-info{font-size:11px;color:#555;margin-top:8px}
</style></head><body>
""" + _NAV + """
<div class="layout">
  <div class="stream-panel"><img src="/stream.mjpg"/></div>
  <div class="sidebar">

    <div class="card">
      <h3>Sistem Durumu</h3>
      <div class="stat"><span class="stat-label">FPS</span><span class="stat-value" id="s-fps">—</span></div>
      <div class="stat"><span class="stat-label">Tracks</span><span class="stat-value" id="s-trk">—</span></div>
      <div class="stat"><span class="stat-label">Lazer</span><span class="stat-value laser-off" id="s-laz">—</span></div>
      <div class="stat"><span class="stat-label">Hedef ID</span><span class="stat-value" id="s-tid">—</span></div>
      <div class="stat"><span class="stat-label">İmha</span><span class="stat-value" id="s-kld">—</span></div>
      <div class="stat"><span class="stat-label">Pan</span><span class="stat-value" id="s-pan">—</span></div>
      <div class="stat"><span class="stat-label">Tilt</span><span class="stat-value" id="s-tlt">—</span></div>
    </div>

    <div class="card">
      <h3>Operasyon</h3>
      <button class="btn btn-danger" onclick="post('/laser/off')">⬛ Lazer KAPAT</button>
      <button class="btn" onclick="post('/laser/on')">🔴 Lazer AÇ (test)</button>
      <button class="btn" onclick="post('/reset')">↺ Hedef Listesini Sıfırla</button>
    </div>

    <div class="card">
      <h3>Kalibrasyon</h3>
      <a href="/calibrate"><button class="btn btn-ok" style="width:100%">⚙ Kalibrasyon Arayüzü</button></a>
      <div class="calib-info" id="s-cal">Yükleniyor...</div>
    </div>

  </div>
</div>

<script>
document.getElementById('nav-control').classList.add('active')
function post(u){fetch(u,{method:'POST'}).then(r=>r.json()).then(d=>console.log(d)).catch(()=>{})}
function refresh(){
  fetch('/status').then(r=>r.json()).then(d=>{
    document.getElementById('s-fps').textContent = d.fps+' fps'
    document.getElementById('s-trk').textContent = d.tracks
    const lEl=document.getElementById('s-laz')
    if(d.laser){lEl.textContent='AÇIK';lEl.className='stat-value laser-on'}
    else{lEl.textContent='kapalı';lEl.className='stat-value laser-off'}
    document.getElementById('s-tid').textContent = d.target_id??'—'
    document.getElementById('s-kld').textContent = d.killed
    document.getElementById('s-pan').textContent = d.pan_deg+'°'
    document.getElementById('s-tlt').textContent = d.tilt_deg+'°'
  }).catch(()=>{})
  fetch('/calibrate/status').then(r=>r.json()).then(d=>{
    const el=document.getElementById('s-cal')
    if(d.has_calibration_file){
      const ox=(d.laser_offset_px_x||0).toFixed(1),oy=(d.laser_offset_px_y||0).toFixed(1)
      el.textContent='Offset: ('+ox+', '+oy+') px'
    } else {el.textContent='Kalibrasyon yok — offset=(0,0)'}
  }).catch(()=>{})
}
setInterval(refresh,1000);refresh()
</script></body></html>"""

CALIBRATE_PAGE = """<!DOCTYPE html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Turret — Kalibrasyon</title>
<style>
""" + _NAV_CSS + """
.content{flex:1;overflow-y:auto;padding:16px;display:flex;flex-direction:column;gap:14px}
.card{background:#191919;border:1px solid #252525;border-radius:8px;padding:14px}
.card h3{font-size:10px;font-weight:600;color:#555;text-transform:uppercase;letter-spacing:.1em;margin-bottom:10px}
.steps{color:#7ab;font-size:13px;line-height:2}
.btn{background:#222;color:#bbb;border:1px solid #333;border-radius:6px;padding:9px 16px;cursor:pointer;font-size:13px;transition:background .15s;margin-right:6px}
.btn:hover{background:#2c2c2c;color:#fff}
.btn-ok{border-color:#1a4a1a;color:#7d7}.btn-ok:hover{background:#0f200f}
.btn-danger{border-color:#5a1515;color:#f77}.btn-danger:hover{background:#2a1010}
#stream-wrap{position:relative;display:inline-block}
#stream{max-width:100%;cursor:crosshair;border:2px solid #2a2a2a;border-radius:6px;display:block}
#marker{position:absolute;width:24px;height:24px;pointer-events:none;border:2px solid #f44;border-radius:50%;display:none;transform:translate(-50%,-50%)}
#info{font-size:13px;color:#888;min-height:18px}
#cal-status{font-size:12px;color:#555;white-space:pre;margin-top:8px}
input[type=number]{background:#1a1a1a;color:#ddd;border:1px solid #333;padding:6px 10px;border-radius:5px;width:90px;font-size:13px}
</style></head><body>
""" + _NAV + """
<div class="content">

<div class="card">
  <h3>Nasıl Yapılır?</h3>
  <div class="steps">
    1. Kalibrasyonu Başlat → servolar merkeze gider, lazer açılır<br>
    2. Hedefi <b>~150 cm</b> mesafeye koy (kağıt yüzeye lazer noktasını düşür)<br>
    3. Aşağıdaki stream'de <b>lazer noktasının tam merkezine</b> tıkla<br>
    4. Mesafeyi gir → Onayla ve Kaydet
  </div>
</div>

<div class="card">
  <h3>Kontroller</h3>
  <button class="btn btn-ok" onclick="startCalib()">▶ Kalibrasyonu Başlat</button>
  <button class="btn btn-danger" onclick="cancelCalib()">✕ İptal</button>
  <div style="margin-top:12px;display:flex;align-items:center;gap:8px">
    <label style="font-size:13px;color:#888">Mesafe (cm):</label>
    <input id="dist" type="number" value="150" min="30" max="500">
    <button class="btn btn-ok" onclick="doConfirm()">✓ Onayla ve Kaydet</button>
  </div>
  <div id="info" style="margin-top:10px">Henüz başlatılmadı.</div>
</div>

<div class="card">
  <h3>Canlı Stream — Lazer Noktasına Tıkla</h3>
  <div id="stream-wrap">
    <img id="stream" src="/stream.mjpg" onclick="onStreamClick(event)"/>
    <div id="marker"></div>
  </div>
</div>

<div class="card">
  <h3>Mevcut Kalibrasyon</h3>
  <div id="cal-status">Yükleniyor...</div>
</div>

</div>
<script>
document.getElementById('nav-cal').classList.add('active')
let clickX=null,clickY=null

function startCalib(){
  fetch('/calibrate/start',{method:'POST'}).then(r=>r.json()).then(()=>{
    document.getElementById('info').textContent='Lazer açık. Stream\'de lazer noktasına tıklayın.'
    refreshStatus()
  })
}
function onStreamClick(e){
  const img=document.getElementById('stream')
  const r=img.getBoundingClientRect()
  clickX=Math.round((e.clientX-r.left)*(img.naturalWidth/r.width))
  clickY=Math.round((e.clientY-r.top)*(img.naturalHeight/r.height))
  const m=document.getElementById('marker')
  m.style.left=(e.clientX-r.left)+'px';m.style.top=(e.clientY-r.top)+'px';m.style.display='block'
  document.getElementById('info').textContent='Seçilen: ('+clickX+', '+clickY+') px — Onayla ya da farklı noktaya tıkla.'
  fetch('/calibrate/click',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({x:clickX,y:clickY})})
}
function doConfirm(){
  if(clickX===null){alert('Önce lazer noktasına tıklayın.');return}
  const dist=parseInt(document.getElementById('dist').value)||150
  fetch('/calibrate/confirm',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({distance_cm:dist})})
  .then(r=>r.json()).then(d=>{
    document.getElementById('info').textContent=d.status==='error'?'Hata: '+d.msg
      :'✓ Kaydedildi! offset=('+d.offset_px_x+', '+d.offset_px_y+')px @ '+d.distance_cm+'cm'
    refreshStatus()
  })
}
function cancelCalib(){
  fetch('/calibrate/cancel',{method:'POST'}).then(()=>{
    document.getElementById('info').textContent='İptal edildi.'
    document.getElementById('marker').style.display='none'
    clickX=null;clickY=null;refreshStatus()
  })
}
function refreshStatus(){
  fetch('/calibrate/status').then(r=>r.json()).then(d=>{
    const ox=(d.laser_offset_px_x||0).toFixed(2),oy=(d.laser_offset_px_y||0).toFixed(2)
    document.getElementById('cal-status').textContent=d.has_calibration_file
      ?'offset_px  : ('+ox+', '+oy+')\ncalibrating: '+d.calibrating
      :'Kalibrasyon dosyası yok — offset=(0,0)'
  })
}
refreshStatus();setInterval(refreshStatus,3000)

<div id="steps">
  1. "Kalibrasyonu Başlat"a tıkla — servolar merkeze gider, lazer açılır<br>
  2. Hedefi <b>~150 cm</b> mesafeye koy (kağıt yüzeye lazer noktasını düşür)<br>
  3. Aşağıdaki stream'de <b>lazer noktasının tam merkezine</b> tıkla<br>
  4. Mesafeyi gir ve "Onayla ve Kaydet"e bas
</div>

<div>
  <button class="ok" onclick="startCalib()">Kalibrasyonu Başlat</button>
  <button class="danger" onclick="cancelCalib()">İptal</button>
</div>

<div id="canvas-wrap">
  <img id="stream" src="/stream.mjpg" onclick="onStreamClick(event)"/>
  <div id="marker"></div>
</div>

<div id="info">Henüz tıklanmadı.</div>

<div style="margin-top:12px">
  Kalibrasyon mesafesi (cm):
  <input id="dist" type="number" value="150" min="30" max="500">
  <button class="ok" onclick="doConfirm()">Onayla ve Kaydet</button>
</div>

<div id="status"></div>

<script>
let clickX = null, clickY = null;

function startCalib() {
  fetch('/calibrate/start', {method:'POST'}).then(r=>r.json()).then(d=>{
    document.getElementById('info').textContent =
      'Kalibrasyon modu aktif — lazer açık. Stream üzerinde lazer noktasına tıklayın.';
    refreshStatus();
  });
}

function onStreamClick(e) {
  const img = document.getElementById('stream');
  const rect = img.getBoundingClientRect();
  const scaleX = img.naturalWidth / rect.width;
  const scaleY = img.naturalHeight / rect.height;
  clickX = Math.round((e.clientX - rect.left) * scaleX);
  clickY = Math.round((e.clientY - rect.top)  * scaleY);

  const marker = document.getElementById('marker');
  marker.style.left = (e.clientX - rect.left) + 'px';
  marker.style.top  = (e.clientY - rect.top)  + 'px';
  marker.style.display = 'block';

  document.getElementById('info').textContent =
    'Seçilen: (' + clickX + ', ' + clickY + ') px. Onayla ya da farklı bir noktaya tıkla.';

  fetch('/calibrate/click', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({x: clickX, y: clickY})
  });
}

function doConfirm() {
  if (clickX === null) { alert('Önce lazer noktasına tıklayın.'); return; }
  const dist = parseInt(document.getElementById('dist').value) || 150;
  fetch('/calibrate/confirm', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({distance_cm: dist})
  }).then(r=>r.json()).then(d=>{
    if (d.status === 'error') {
      document.getElementById('info').textContent = 'Hata: ' + d.msg;
    } else {
      document.getElementById('info').textContent =
        'Kaydedildi! offset=(' + d.offset_px_x + ', ' + d.offset_px_y + ')px @ ' + d.distance_cm + 'cm';
    }
    refreshStatus();
  });
}

function cancelCalib() {
  fetch('/calibrate/cancel', {method:'POST'}).then(r=>r.json()).then(()=>{
    document.getElementById('info').textContent = 'İptal edildi.';
    document.getElementById('marker').style.display = 'none';
    clickX = null; clickY = null;
    refreshStatus();
  });
}

function refreshStatus() {
  fetch('/calibrate/status').then(r=>r.json()).then(d=>{
    document.getElementById('status').textContent = JSON.stringify(d, null, 2);
  });
}

refreshStatus();
setInterval(refreshStatus, 3000);
</script></body></html>"""


def make_handler(buf_main: FrameBuffer, buf_debug: FrameBuffer):
    class H(server.BaseHTTPRequestHandler):
        def log_message(self, *a): return

        def _send_page(self, body: str):
            b = body.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(b)))
            self.end_headers()
            self.wfile.write(b)

        def _stream(self, buf: FrameBuffer):
            self.send_response(200)
            self.send_header("Age", "0")
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=FRAME")
            self.end_headers()
            try:
                while True:
                    with buf.cond:
                        buf.cond.wait(timeout=2.0)
                        frame = buf.frame
                    if frame is None:
                        continue
                    self.wfile.write(b"--FRAME\r\n")
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", str(len(frame)))
                    self.end_headers()
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                pass

        def _json_ok(self, payload: dict):
            body = _json.dumps(payload).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                self._send_page(PAGE)
            elif self.path == "/debug":
                self._send_page(DEBUG_PAGE)
            elif self.path == "/stream.mjpg":
                self._stream(buf_main)
            elif self.path == "/debug.mjpg":
                self._stream(buf_debug)
            elif self.path == "/control":
                self._send_page(CONTROL_PAGE)
            elif self.path == "/calibrate":
                self._send_page(CALIBRATE_PAGE)
            elif self.path == "/status":
                with data_lock:
                    snapshot = dict(web_status)
                self._json_ok(snapshot)
            elif self.path == "/calibrate/status":
                self._json_ok({
                    "laser_offset_px_x":  LASER_OFFSET_PX_X,
                    "laser_offset_px_y":  LASER_OFFSET_PX_Y,
                    "calibrating":        calibrating,
                    "has_calibration_file": os.path.exists(CALIB_PATH),
                })
            else:
                self.send_error(404)

        def do_POST(self):
            global calibrating, calib_click_px, current_target_id, killed_flies
            global LASER_OFFSET_PX_X, LASER_OFFSET_PX_Y

            if self.path == "/laser/off":
                with data_lock:
                    current_target_id = None
                    target_pan_deg    = 90.0
                    target_tilt_deg   = 90.0
                if HAS_GPIO and lazer:
                    lazer.off()
                log("[WEB] Lazer zorla kapatıldı.")
                self._json_ok({"status": "laser_off"})

            elif self.path == "/laser/on":
                if HAS_GPIO and lazer:
                    lazer.on()
                log("[WEB] Lazer manuel açıldı (test — state machine override edebilir).")
                self._json_ok({"status": "laser_on", "warning": "state machine may override"})

            elif self.path == "/reset":
                with data_lock:
                    killed_flies.clear()
                    current_target_id = None
                log("[WEB] Hedef listesi sıfırlandı.")
                self._json_ok({"status": "reset"})

            elif self.path == "/calibrate/start":
                with data_lock:
                    calibrating       = True
                    calib_click_px    = None
                    current_target_id = None
                    target_pan_deg    = 90.0
                    target_tilt_deg   = 90.0
                if HAS_GPIO and lazer:
                    lazer.on()
                log("[KALİBRASYON] Kalibrasyon modu başlatıldı.")
                self._json_ok({"status": "calibrating"})

            elif self.path == "/calibrate/click":
                length = int(self.headers.get("Content-Length", 0))
                body   = _json.loads(self.rfile.read(length))
                with data_lock:
                    calib_click_px = (float(body["x"]), float(body["y"]))
                self._json_ok({"status": "click_received", "x": body["x"], "y": body["y"]})

            elif self.path == "/calibrate/confirm":
                length = int(self.headers.get("Content-Length", 0))
                body   = _json.loads(self.rfile.read(length)) if length else {}
                distance_cm = int(body.get("distance_cm", 150))
                with data_lock:
                    click       = calib_click_px
                    calibrating = False
                if HAS_GPIO and lazer:
                    lazer.off()
                if click is None:
                    self._json_ok({"status": "error", "msg": "Önce lazer noktasına tıkla"})
                    return
                proc_x   = click[0] * (PROCESS_W / CAPTURE_W)
                proc_y   = click[1] * (PROCESS_H / CAPTURE_H)
                offset_x = proc_x - PROCESS_W / 2
                offset_y = proc_y - PROCESS_H / 2
                _save_calibration(offset_x, offset_y, distance_cm)
                LASER_OFFSET_PX_X = offset_x
                LASER_OFFSET_PX_Y = offset_y
                self._json_ok({
                    "status":      "calibrated",
                    "offset_px_x": round(offset_x, 1),
                    "offset_px_y": round(offset_y, 1),
                    "distance_cm": distance_cm,
                })

            elif self.path == "/calibrate/cancel":
                with data_lock:
                    calibrating    = False
                    calib_click_px = None
                if HAS_GPIO and lazer:
                    lazer.off()
                log("[KALİBRASYON] İptal edildi.")
                self._json_ok({"status": "cancelled"})

            else:
                self.send_error(404)
    return H


class ThreadedHTTPServer(ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True


def start_stream(buf_main: FrameBuffer, buf_debug: FrameBuffer, port: int):
    httpd = ThreadedHTTPServer(("0.0.0.0", port), make_handler(buf_main, buf_debug))
    t = threading.Thread(target=httpd.serve_forever, daemon=True)
    t.start()
    log(f"Stream: http://heliosx.local:{port}/ (debug: /debug)")
    return httpd


# =========================================================================
# THREAD: MOTORLARIN PÜRÜZSÜZ LERP SÜZÜLME DÖNGÜSÜ
# =========================================================================
def motor_smooth_thread():
    """ Ana kodun döngü hızından bağımsız çalışarak motorları süzerek hedefe götürür """
    global target_pan_deg, target_tilt_deg, current_pan_deg, current_tilt_deg, is_running
    
    # Geçiş pürüzsüzlük faktörü (Hızlı reaksiyon için 0.22 seçildi)
    smooth_factor = 0.22 
    pan_active = False
    tilt_active = False
    
    while is_running:
        with data_lock:
            t_pan = target_pan_deg
            t_tilt = target_tilt_deg
            
        # PAN Süzülme Kontrolü
        pan_diff = t_pan - current_pan_deg
        if abs(pan_diff) > 0.05:
            current_pan_deg += pan_diff * smooth_factor
            current_pan_deg = max(PAN_MIN_DEG, min(PAN_MAX_DEG, current_pan_deg))
            pan_servo.value = deg_to_servo_val(current_pan_deg)
            pan_active = True
        elif pan_active:
            pan_servo.detach()  # Hedefe milimetrik oturduğunda enerjiyi kes, titremesin
            pan_active = False

        # TILT Süzülme Kontrolü
        tilt_diff = t_tilt - current_tilt_deg
        if abs(tilt_diff) > 0.05:
            current_tilt_deg += tilt_diff * smooth_factor
            current_tilt_deg = max(TILT_MIN_DEG, min(TILT_MAX_DEG, current_tilt_deg))
            tilt_servo.value = deg_to_servo_val(current_tilt_deg)
            tilt_active = True
        elif tilt_active:
            tilt_servo.detach()
            tilt_active = False
            
        time.sleep(0.005) # 5ms döngü hızı ile mükemmel donanımsal akıcılık (50 Hz PWM için ideal) [cite: 32, 33]


# =========================================================================
# Main
# =========================================================================

def main() -> None:
    global current_target_id, lock_start_time, target_pan_deg, target_tilt_deg, is_running
    global pan_servo, tilt_servo, lazer

    _load_calibration()

    # Donanım Başlatma Kontrolleri
    trigger = LED(TRIGGER_PIN) if HAS_GPIO else None
    
    if HAS_GPIO:
        # MG90S için datasheet'te doğrulanmış 1-2 ms pulse aralıkları [cite: 29]
        min_pw = 0.001
        max_pw = 0.002
        
        pan_servo = Servo(PAN_PIN, min_pulse_width=min_pw, max_pulse_width=max_pw)
        tilt_servo = Servo(TILT_PIN, min_pulse_width=min_pw, max_pulse_width=max_pw)
        lazer = LED(LAZER_PIN)
        
        # İlk kalibrasyon: Merkeze al ve enerjiyi geçici olarak kes
        pan_servo.value = deg_to_servo_val(90)
        tilt_servo.value = deg_to_servo_val(90)
        time.sleep(0.3)
        pan_servo.detach()
        tilt_servo.detach()
        log("Servolar ve Lazer başarıyla ilklendirildi.")
    else:
        log("UYARI: gpiozero kütüphanesi yüklenemedi, donanım kontrolü simüle edilecek.")

    # Bağımsız Motor Kontrol Thread'ini Başlatıyoruz
    t_motor = threading.Thread(target=motor_smooth_thread)
    t_motor.start()

    picam2 = Picamera2()
    cam_controls = {"FrameRate": TARGET_FPS}
    if EXPOSURE_TIME_US > 0:
        cam_controls["AeEnable"] = False
        cam_controls["ExposureTime"] = EXPOSURE_TIME_US
        cam_controls["AnalogueGain"] = ANALOGUE_GAIN
        log(f"Manuel pozlama: {EXPOSURE_TIME_US}us, gain={ANALOGUE_GAIN}")
    cfg = picam2.create_video_configuration(
        main={"size": (CAPTURE_W, CAPTURE_H), "format": "RGB888"},
        controls=cam_controls,
        queue=False,
    )
    picam2.configure(cfg)
    picam2.start()
    time.sleep(1.0)
    log(f"Kamera: {CAPTURE_W}x{CAPTURE_H} @ {TARGET_FPS}fps, "
        f"process {PROCESS_W}x{PROCESS_H}")

    buf_main = FrameBuffer()
    buf_debug = FrameBuffer()
    httpd = start_stream(buf_main, buf_debug, STREAM_PORT) if ENABLE_STREAM else None

    pipeline = MaskPipeline()
    tracker = Tracker()
    encode = [int(cv2.IMWRITE_JPEG_QUALITY), STREAM_QUALITY]

    sx = CAPTURE_W / PROCESS_W
    sy = CAPTURE_H / PROCESS_H
    max_motion_px = int(MAX_GLOBAL_MOTION_RATIO * PROCESS_W * PROCESS_H)

    frame_idx = 0
    last_trigger = 0.0
    fps = 0.0
    _fps_window = 60
    _frame_times: deque = deque(maxlen=_fps_window + 1)
    n_suppressed = 0
    biggest_history: deque = deque(maxlen=TARGET_FPS)
    display_max = (0, 0, 0)
    display_max_t = time.time()

    try:
        while True:
            rgb = picam2.capture_array("main")
            frame = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            small = cv2.resize(frame, (PROCESS_W, PROCESS_H),
                               interpolation=cv2.INTER_AREA)
            gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)

            enhanced, motion, motion_exclude, blackhat, combined = pipeline.process(gray)

            if frame_idx < WARMUP_FRAMES:
                frame_idx += 1
                continue

            # KALIBRASYON MODU — detection ve tracking duraklatılır
            if calibrating:
                tracker.tracks.clear()
                overlay = frame.copy()
                cv2.drawMarker(overlay, (CAPTURE_W // 2, CAPTURE_H // 2),
                               (0, 255, 255), cv2.MARKER_CROSS, 40, 2)
                cv2.putText(overlay, "KALIBRASYON — Lazer noktasina tiklayin",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                            (0, 255, 255), 2, cv2.LINE_AA)
                with data_lock:
                    click = calib_click_px
                if click is not None:
                    cv2.drawMarker(overlay, (int(click[0]), int(click[1])),
                                   (0, 0, 255), cv2.MARKER_TILTED_CROSS, 30, 2)
                if ENABLE_STREAM:
                    ok, jpg = cv2.imencode(".jpg", overlay, encode)
                    if ok:
                        buf_main.update(jpg.tobytes())
                frame_idx += 1
                continue

            _frame_times.append(time.time())
            total_motion = int(cv2.countNonZero(motion))
            suppressed = total_motion > max_motion_px

            exclusion_zones: list[tuple[int, int, int, int]] = []
            biggest_comp = (0, 0, 0)  
            if not suppressed:
                n_lbl, _, comp_stats, _ = cv2.connectedComponentsWithStats(
                    motion_exclude, connectivity=8
                )
                for i in range(1, n_lbl):
                    area_i = int(comp_stats[i, cv2.CC_STAT_AREA])
                    cw = int(comp_stats[i, cv2.CC_STAT_WIDTH])
                    ch = int(comp_stats[i, cv2.CC_STAT_HEIGHT])
                    if area_i > biggest_comp[0]:
                        biggest_comp = (area_i, cw, ch)
                    if area_i >= LARGE_MOTION_AREA or max(cw, ch) >= LARGE_MOTION_DIM:
                        cx0 = int(comp_stats[i, cv2.CC_STAT_LEFT])
                        cy0 = int(comp_stats[i, cv2.CC_STAT_TOP])
                        x1 = max(0, cx0 - EXCLUSION_PADDING)
                        y1 = max(0, cy0 - EXCLUSION_PADDING)
                        x2 = min(PROCESS_W, cx0 + cw + EXCLUSION_PADDING)
                        y2 = min(PROCESS_H, cy0 + ch + EXCLUSION_PADDING)
                        exclusion_zones.append((x1, y1, x2, y2))

            def in_exclusion(px: int, py: int) -> bool:
                for x1, y1, x2, y2 in exclusion_zones:
                    if x1 <= px <= x2 and y1 <= py <= y2:
                        return True
                return False

            dets: list[Detection] = []
            if not suppressed:
                contours, _ = cv2.findContours(
                    combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
                )
                for c in contours:
                    area = cv2.contourArea(c)
                    if not (MIN_AREA <= area <= MAX_AREA):
                        continue
                    x, y, w, h = cv2.boundingRect(c)
                    if not (MIN_WH <= w <= MAX_WH and MIN_WH <= h <= MAX_WH):
                        continue
                    if in_exclusion(x + w // 2, y + h // 2):
                        continue
                    aspect = w / max(h, 1)
                    if not (MIN_ASPECT <= aspect <= MAX_ASPECT):
                        continue

                    bbox_motion = motion[y:y + h, x:x + w]
                    motion_score = float(np.mean(bbox_motion)) / 255.0
                    if motion_score < MIN_MOTION_SCORE:
                        continue

                    bbox_bh = blackhat[y:y + h, x:x + w]
                    dark_score = float(np.mean(bbox_bh))
                    if dark_score < MIN_DARK_SCORE:
                        continue

                    bbox_enh = enhanced[y:y + h, x:x + w]
                    local_mean = float(np.mean(bbox_enh))
                    if local_mean > MAX_LOCAL_MEAN:
                        continue

                    dets.append(Detection(
                        cx=x + w // 2, cy=y + h // 2,
                        x=x, y=y, w=w, h=h,
                        area=area,
                        motion_score=motion_score,
                        dark_score=dark_score,
                    ))

                if len(dets) > MAX_TOTAL_DETECTIONS:
                    suppressed = True
                    dets = []

            if suppressed:
                tracker.tracks.clear()
                n_suppressed += 1
            else:
                if exclusion_zones:
                    for tid in list(tracker.tracks.keys()):
                        tr = tracker.tracks[tid]
                        if in_exclusion(int(tr.cx), int(tr.cy)):
                            tracker.tracks.pop(tid)
                tracker.update(dets, frame_idx)

            confirmed = tracker.confirmed(frame_idx)

            # =========================================================================
            # AKILLI LAZER TAKİP VE DURUM MAKİNESİ (3 SANİYE KURALI - COORDİNAT KİLİTLİ)
            # =========================================================================
            active_tracks = [tr for tr in tracker.tracks.values() if tr.misses == 0]
            now = time.time()
            target_still_visible = False

            # ADIM 1: Eğer halihazırda kilitli bir hedefimiz varsa onu takip et
            if current_target_id is not None:
                for tr in active_tracks:
                    if tr.track_id == current_target_id:
                        target_still_visible = True
                        gecen_sure = now - lock_start_time

                        if gecen_sure >= 3.0:
                            # 3 saniye kesintisiz vurduk, sinek imha edildi!
                            if HAS_GPIO:
                                lazer.off()
                            killed_flies.add(tr.track_id)
                            log(f"[İMHA EDİLDİ] ID: {tr.track_id} 3 saniye boyunca vuruldu. Kilit açıldı.")
                            current_target_id = None
                            with data_lock:
                                target_pan_deg  = 90.0
                                target_tilt_deg = 90.0
                            if HAS_GPIO:
                                pan_servo.detach()
                                tilt_servo.detach()
                        else:
                            # 3 saniye henüz dolmadı, sineği pürüzsüzce merkeze doğru takip et
                            error_x = tr.cx - (PROCESS_W / 2 + LASER_OFFSET_PX_X)
                            error_y = tr.cy - (PROCESS_H / 2 + LASER_OFFSET_PX_Y)

                            # Adaptif kazanım: hata büyükse KP_BOOST, küçükse KP_BASE
                            kp_x = adaptive_kp(error_x)
                            kp_y = adaptive_kp(error_y)

                            with data_lock:
                                target_pan_deg  = max(PAN_MIN_DEG,  min(PAN_MAX_DEG,
                                                      target_pan_deg  - (error_x * kp_x)))
                                target_tilt_deg = max(TILT_MIN_DEG, min(TILT_MAX_DEG,
                                                      target_tilt_deg - (error_y * kp_y)))
                        break

            # ADIM 2: Eğer kilitli sinek gerçekten kadrajdan çıktıysa kilidi temizle
            if current_target_id is not None and not target_still_visible:
                if HAS_GPIO:
                    lazer.off()
                    pan_servo.detach()
                    tilt_servo.detach()
                log(f"[HEDEF KAÇTI] ID {current_target_id} gözden kayboldu. Lazer Kapatıldı.")
                current_target_id = None
                with data_lock:
                    target_pan_deg  = 90.0
                    target_tilt_deg = 90.0

            # ADIM 3: Eğer sistem boştaysa ve yeni bir aday sinek geldiyse kilitlen
            if current_target_id is None:
                candidates = [
                    tr for tr in active_tracks
                    if tr.track_id not in killed_flies and tr.hits >= MIN_HITS
                ]
                if candidates:
                    best = max(candidates, key=candidate_score)
                    best_score = candidate_score(best)
                    current_target_id = best.track_id
                    lock_start_time = now
                    target_still_visible = True  # Aynı frame içerisinde hedef kaçtı kontrolüne düşmemesi için
                    if HAS_GPIO:
                        lazer.on()
                    log(f"[KİLİTLENDİ] Hedef ID: {best.track_id} "
                        f"skor={best_score:.1f} hits={best.hits} "
                        f"path={best.path_length():.1f}px. Lazer AÇIK.")

            # Web durum snapshot'ı — HTTP handler thread'leri buradan okur
            with data_lock:
                web_status.update({
                    "fps":       round(fps, 1),
                    "tracks":    len(tracker.tracks),
                    "confirmed": len(confirmed),
                    "laser":     bool(lazer.is_lit) if HAS_GPIO and lazer else False,
                    "target_id": current_target_id,
                    "killed":    len(killed_flies),
                    "pan_deg":   round(current_pan_deg, 1),
                    "tilt_deg":  round(current_tilt_deg, 1),
                    "calibrating": calibrating,
                })

            # Orijinal röle tetikleyici mekanizması
            for tr in confirmed:
                if not tr.triggered and (now - last_trigger) > TRIGGER_COOLDOWN:
                    if trigger is not None:
                        trigger.on()
                        time.sleep(0.05)
                        trigger.off()
                    last_trigger = now
                    tr.triggered = True
                    log(f"[DETECTION] id={tr.track_id} cx={tr.cx:.0f} cy={tr.cy:.0f} "
                        f"hits={tr.hits} path={tr.path_length():.1f}px speed={tr.speed():.1f}px/f")

            # =========================================================================
            # Orijinal Çizim ve Görüntü Katmanı (Overlay)
            # =========================================================================
            for (x1, y1, x2, y2) in exclusion_zones:
                fx1, fy1 = int(x1 * sx), int(y1 * sy)
                fx2, fy2 = int(x2 * sx), int(y2 * sy)
                overlay = frame.copy()
                cv2.rectangle(overlay, (fx1, fy1), (fx2, fy2), (0, 0, 180), -1)
                cv2.addWeighted(overlay, 0.20, frame, 0.80, 0, frame)
                cv2.rectangle(frame, (fx1, fy1), (fx2, fy2), (0, 0, 200), 1)
                cv2.putText(frame, "exclude", (fx1 + 4, fy1 + 16),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                            (0, 0, 200), 1, cv2.LINE_AA)

            for tr in tracker.tracks.values():
                px = int(tr.cx * sx)
                py = int(tr.cy * sy)
                if tr.track_id == current_target_id:
                    color = (255, 0, 0) # Kilitli hedefe mavi çember çiz
                    cv2.putText(frame, f"LASER LOCK {time.time()-lock_start_time:.1f}s", (px - 30, py - 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2, cv2.LINE_AA)
                else:
                    color = (0, 200, 200) if tr.hits < MIN_HITS else (0, 255, 0)
                cv2.circle(frame, (px, py), 12, color, 1)
                if tr.hits >= MIN_HITS and tr.track_id not in killed_flies and tr.track_id != current_target_id:
                    cv2.putText(frame, f"{candidate_score(tr):.0f}",
                                (px + 14, py + 4), cv2.FONT_HERSHEY_SIMPLEX,
                                0.35, color, 1, cv2.LINE_AA)

            for tr in confirmed:
                px = int(tr.cx * sx)
                py = int(tr.cy * sy)
                cv2.circle(frame, (px, py), 18, (0, 0, 255), 2)
                cv2.putText(frame, f"id{tr.track_id}",
                            (px + 18, py), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 0, 255), 1, cv2.LINE_AA)

            status = "SUPPRESSED" if suppressed else f"mot:{total_motion}"
            ba, bw, bh = biggest_comp
            cv2.putText(
                frame,
                f"FPS:{fps:4.1f} dets:{len(dets):2d} "
                f"trk:{len(tracker.tracks):2d} conf:{len(confirmed):d} "
                f"excl:{len(exclusion_zones):d} {status}",
                (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                (0, 255, 0) if not suppressed else (0, 165, 255), 1, cv2.LINE_AA,
            )

            biggest_history.append(biggest_comp)
            if time.time() - display_max_t > 1.0:
                display_max = max(
                    biggest_history,
                    key=lambda c: c[0],
                    default=(0, 0, 0),
                )
                display_max_t = time.time()
            dba, dbw, dbh = display_max
            cv2.putText(
                frame,
                f"max blob (1s): area={dba} w={dbw} h={dbh} "
                f" thresholds: area>={LARGE_MOTION_AREA} dim>={LARGE_MOTION_DIM}",
                (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.48,
                (200, 200, 200), 1, cv2.LINE_AA,
            )

            if ENABLE_STREAM:
                ok, jpg = cv2.imencode(".jpg", frame, encode)
                if ok:
                    buf_main.update(jpg.tobytes())

                dbg = cv2.cvtColor(combined, cv2.COLOR_GRAY2BGR)
                dbg[motion > 0] = (200, 80, 0)
                dbg[combined > 0] = (0, 255, 0)
                for d in dets:
                    cv2.rectangle(dbg, (d.x, d.y), (d.x + d.w, d.y + d.h),
                                  (0, 0, 255), 1)
                ok, jpg = cv2.imencode(".jpg", dbg, encode)
                if ok:
                    buf_debug.update(jpg.tobytes())

            frame_idx += 1
            if frame_idx % 30 == 0 and len(_frame_times) >= 2:
                elapsed = _frame_times[-1] - _frame_times[0]
                fps = (len(_frame_times) - 1) / elapsed if elapsed > 0 else 0.0
            if frame_idx % BRIGHTNESS_WINDOW == 0 and frame_idx > WARMUP_FRAMES:
                _adjust_exposure(picam2, float(np.mean(gray)))
            if frame_idx % 300 == 0:
                log(f"FPS~{fps:.1f} trk:{len(tracker.tracks)} "
                    f"dets:{len(dets)} suppr_frames:{n_suppressed}")

    except KeyboardInterrupt:
        log("Durduruldu (Ctrl+C).")
    finally:
        # =========================================================================
        # GÜVENLİ KAPANIŞ (TÜM GÜÇLERİ KESME)
        # =========================================================================
        is_running = False
        t_motor.join()
        
        if HAS_GPIO:
            lazer.off()
            pan_servo.detach()
            tilt_servo.detach()
        log("Donanım sinyalleri kesildi, sistem güvenli moda alındı.")

        if httpd is not None:
            httpd.shutdown()
        picam2.stop()
        log("Kamera ve yayın sunucusu kapatıldı.")


if __name__ == "__main__":
    main()
