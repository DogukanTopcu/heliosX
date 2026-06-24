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
import argparse
from collections import deque
from dataclasses import dataclass, field
from http import server
from socketserver import ThreadingMixIn

import cv2
import numpy as np
try:
    from picamera2 import Picamera2
    HAS_PICAMERA2 = True
except Exception:
    Picamera2 = None
    HAS_PICAMERA2 = False

# Raspberry Pi 5 için yerel lgpio sürücüsünü zorunlu kılıyoruz
os.environ["GPIOZERO_PIN_FACTORY"] = "lgpio"

try:
    from gpiozero import AngularServo, LED
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

# Servo mekanik hareket sınırları ve konvansiyon — manual-control5.py / servo-angle-finder.py
# ile fiziksel olarak kalibre edildi. Pulse aralığı 0.5–2.5 ms (klon MG90S).
# Pan:  fiziksel açı, 0° = merkez (+sağ/−sol)
# Tilt: 0° = fiziksel alt limit; servo sinyaline çevrilirken TILT_ZERO_OFFSET eklenir
PAN_MIN_DEG  =  -5.0
PAN_MAX_DEG  =   5.0
TILT_MIN_DEG =   0.0
TILT_MAX_DEG =  10.0
TILT_ZERO_OFFSET = 25.0    # gösterge 0° = fiziksel alt limit (25° servo konumu)
PAN_HOME_DEG  = 0.0        # boşta/merkez pan
TILT_HOME_DEG = 0.0        # boşta/alt-limit tilt (başlangıç konumu)
SERVO_MIN_PW = 0.0005      # 0.5 ms
SERVO_MAX_PW = 0.0025      # 2.5 ms
SERVO_MOVE_DURATION = 0.25 # her hedef komutu için cubic ease süresi
SERVO_MOVE_STEPS = 50      # 200 Hz açı güncellemesi (0.25 s / 50 adım)

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

# Otomatik kalibrasyon — lazer noktasını görüntüde bulup piksel→servo haritası fit eder
AUTOCAL_GRID_PAN   = 6      # pan ekseninde tarama noktası sayısı
AUTOCAL_GRID_TILT  = 5      # tilt ekseninde tarama noktası sayısı
AUTOCAL_SETTLE_FR  = 18     # her noktada servo+kamera oturana dek beklenen frame (~0.6s@30fps)
AUTOCAL_REF_FR     = 8      # referans (lazer kapalı) frame sayısı
AUTOCAL_MEASURE_FR = 6      # settle sonrası kaç frame lazer noktası toplanacak
AUTOCAL_MAX_JITTER_PX = 6.0 # bir grid noktasındaki lazer gözlemlerinin izinli saçılımı
LASER_RED_MIN      = 45     # kırmızı baskınlık eşiği (R - max(G,B)), referanssız önizleme
LASER_DIFF_MIN     = 40     # lazer açık-kapalı parlaklık farkı eşiği (doygun nokta da yakalanır)
LASER_MIN_AREA     = 2      # px², bu alandan küçük lekeler yok sayılır
AUTOCAL_MIN_SAMPLES = 12    # polinom fit'i kabul etmek için minimum nokta
AUTOCAL_MAX_RMS_PAN_DEG = 6.0
AUTOCAL_MAX_RMS_TILT_DEG = 6.0
AUTOCAL_MIN_PX_SPAN_X = 120.0
AUTOCAL_MIN_PX_SPAN_Y = 90.0

# Stream / log
ENABLE_STREAM = True
STREAM_PORT = 8080
STREAM_QUALITY = 75
WARMUP_FRAMES = 15                    # detection bu kadar frame sonra basla
LOG_TO_FILE = True
LOG_PATH = "/home/heliosx/v2/detections.log"
DRY_RUN_BG = 235

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


def _adjust_exposure(cam, brightness: float) -> bool:
    """Pozlamayı parlaklığa göre ayarlar. Donanım kontrolü değiştiyse True döner
    (çağıran taraf frame-diff'i sıfırlamak için bunu kullanır — B2 çatışması)."""
    global _current_exposure, _current_gain
    if EXPOSURE_TIME_US == 0:
        return False  # otomatik AE açıksa dokunma

    error = BRIGHTNESS_TARGET - brightness  # pozitif = çok karanlık
    prev_exp, prev_gain = _current_exposure, _current_gain

    if error > 10:
        _current_exposure = min(_current_exposure * 1.15, EXPOSURE_MAX_US)
    elif error < -10:
        _current_exposure = max(_current_exposure * 0.87, EXPOSURE_MIN_US)

    if _current_exposure >= EXPOSURE_MAX_US and error > 15:
        _current_gain = min(_current_gain * 1.1, GAIN_MAX)
    elif _current_exposure <= EXPOSURE_MIN_US and error < -15:
        _current_gain = max(_current_gain * 0.9, GAIN_MIN)

    if int(_current_exposure) == int(prev_exp) and abs(_current_gain - prev_gain) < 1e-3:
        return False  # kayda değer değişiklik yok, kameraya dokunma

    cam.set_controls({
        "ExposureTime": int(_current_exposure),
        "AnalogueGain": _current_gain,
    })
    log(f"[EXPOSURE] parlaklık={brightness:.0f} exp={_current_exposure:.0f}µs gain={_current_gain:.1f}")
    return True


def _load_calibration() -> None:
    global LASER_OFFSET_PX_X, LASER_OFFSET_PX_Y
    global HAS_PIXEL_MAP, MAP_PAN_COEF, MAP_TILT_COEF
    if not os.path.exists(CALIB_PATH):
        log("Kalibrasyon dosyası yok, offset=(0,0) kullanılıyor.")
        return
    try:
        with open(CALIB_PATH) as f:
            data = _json.load(f)
        LASER_OFFSET_PX_X = float(data.get("laser_offset_px_x", 0.0))
        LASER_OFFSET_PX_Y = float(data.get("laser_offset_px_y", 0.0))
        if "pan_coef" in data and "tilt_coef" in data:
            MAP_PAN_COEF  = np.array(data["pan_coef"],  dtype=np.float64)
            MAP_TILT_COEF = np.array(data["tilt_coef"], dtype=np.float64)
            HAS_PIXEL_MAP = MAP_PAN_COEF.shape == (6,) and MAP_TILT_COEF.shape == (6,)
            rms_pan = float(data.get("rms_pan_deg", 999.0))
            rms_tilt = float(data.get("rms_tilt_deg", 999.0))
            n_samples = int(data.get("n_samples", 0))
            px_span_x = float(data.get("px_span_x", 0.0))
            px_span_y = float(data.get("px_span_y", 0.0))
            if HAS_PIXEL_MAP and (
                n_samples < AUTOCAL_MIN_SAMPLES
                or px_span_x < AUTOCAL_MIN_PX_SPAN_X
                or px_span_y < AUTOCAL_MIN_PX_SPAN_Y
                or rms_pan > AUTOCAL_MAX_RMS_PAN_DEG
                or rms_tilt > AUTOCAL_MAX_RMS_TILT_DEG
            ):
                log(f"[KALİBRASYON] Piksel haritası REDDEDİLDİ "
                    f"(n={n_samples}, span={px_span_x:.0f}x{px_span_y:.0f}px, "
                    f"rms={rms_pan}/{rms_tilt}°). "
                    "Offset moduna düşülüyor.")
                HAS_PIXEL_MAP = False
                MAP_PAN_COEF = None
                MAP_TILT_COEF = None
        if HAS_PIXEL_MAP:
            log(f"Piksel→servo haritası yüklendi "
                f"(n={data.get('n_samples','?')}, "
                f"span={data.get('px_span_x','?')}x{data.get('px_span_y','?')}px, "
                f"rms={data.get('rms_pan_deg','?')}/{data.get('rms_tilt_deg','?')}°) "
                f"[{data.get('calibrated_at', '?')}]")
        else:
            log(f"Kalibrasyon yüklendi: offset=({LASER_OFFSET_PX_X:.1f}, {LASER_OFFSET_PX_Y:.1f})px"
                f" [{data.get('calibrated_at', '?')}] — piksel haritası yok, eski offset modu.")
    except Exception as e:
        log(f"Kalibrasyon dosyası okunamadı: {e} — offset=(0,0) kullanılıyor.")


def _detect_laser_dot(small_bgr, ref_bgr=None):
    """Process çözünürlüklü BGR frame'de lazer noktasını bulur.
    ref_bgr (lazer kapalı referans) verilirse: lazer açık-kapalı PARLAKLIK farkı kullanılır
    — doygun (beyaz çekirdekli) noktayı da yakalar, sahnedeki statik nesneleri eler.
    Yoksa: salt kırmızı baskınlığı (önizleme için). Dönüş: (px, py, area) veya None."""
    if ref_bgr is not None:
        diff = cv2.absdiff(small_bgr, ref_bgr)
        bright = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY).astype(np.int16)
        b, g, r = cv2.split(diff.astype(np.int16))
        redness = np.clip(r - np.maximum(g, b), 0, 255)
        score = np.clip(bright + redness, 0, 255).astype(np.uint8)
        thr = LASER_DIFF_MIN
    else:
        b, g, r = cv2.split(small_bgr.astype(np.int16))
        score = np.clip(r - np.maximum(g, b), 0, 255).astype(np.uint8)
        thr = LASER_RED_MIN
    _, mask = cv2.threshold(score, thr, 255, cv2.THRESH_BINARY)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)))
    n, _, stats, cent = cv2.connectedComponentsWithStats(mask, connectivity=8)
    best, best_area = None, 0
    for i in range(1, n):
        area = int(stats[i, cv2.CC_STAT_AREA])
        if area < LASER_MIN_AREA:
            continue
        if area > best_area:
            best_area = area
            best = (float(cent[i][0]), float(cent[i][1]), area)
    return best


def _poly_terms(px: float, py: float) -> np.ndarray:
    """İkinci derece polinom tasarım vektörü: [1, px, py, px², py², px·py]"""
    return np.array([1.0, px, py, px * px, py * py, px * py], dtype=np.float64)


def _fit_pixel_to_servo(samples: list[tuple[float, float, float, float]]):
    """samples: (pan_deg, tilt_deg, px, py). Piksel→açı için ikinci derece polinom fit.
    Dönüş: katsayılar + RMS artık hata içeren dict, veya yetersiz veri varsa None."""
    if len(samples) < 6:
        return None
    pan  = np.array([s[0] for s in samples], dtype=np.float64)
    tilt = np.array([s[1] for s in samples], dtype=np.float64)
    pxs = np.array([s[2] for s in samples], dtype=np.float64)
    pys = np.array([s[3] for s in samples], dtype=np.float64)
    A = np.array([_poly_terms(s[2], s[3]) for s in samples], dtype=np.float64)
    pan_coef,  *_ = np.linalg.lstsq(A, pan,  rcond=None)
    tilt_coef, *_ = np.linalg.lstsq(A, tilt, rcond=None)
    rms_pan  = float(np.sqrt(np.mean((A @ pan_coef  - pan)  ** 2)))
    rms_tilt = float(np.sqrt(np.mean((A @ tilt_coef - tilt) ** 2)))
    return {
        "pan_coef":  pan_coef.tolist(),
        "tilt_coef": tilt_coef.tolist(),
        "rms_pan_deg":  round(rms_pan, 3),
        "rms_tilt_deg": round(rms_tilt, 3),
        "n_samples": len(samples),
        "px_span_x": round(float(np.max(pxs) - np.min(pxs)), 1),
        "px_span_y": round(float(np.max(pys) - np.min(pys)), 1),
    }


def pixel_to_servo(px: float, py: float) -> tuple[float, float]:
    """Hedef pikselini, lazeri o piksele götürecek (pan, tilt) açısına çevirir."""
    f = _poly_terms(px, py)
    return float(f @ MAP_PAN_COEF), float(f @ MAP_TILT_COEF)


def _save_pixel_map(result: dict) -> None:
    """Otomatik kalibrasyon sonucunu calibration.json'a yazar ve haritayı aktive eder."""
    global HAS_PIXEL_MAP, MAP_PAN_COEF, MAP_TILT_COEF
    data = {
        "mode":              "pixel_servo_poly2",
        "pan_coef":          result["pan_coef"],
        "tilt_coef":         result["tilt_coef"],
        "rms_pan_deg":       result["rms_pan_deg"],
        "rms_tilt_deg":      result["rms_tilt_deg"],
        "n_samples":         result["n_samples"],
        "px_span_x":         result.get("px_span_x"),
        "px_span_y":         result.get("px_span_y"),
        "calibrated_at":     _dt.datetime.now().isoformat(timespec="seconds"),
        "process_resolution": f"{PROCESS_W}x{PROCESS_H}",
    }
    with open(CALIB_PATH, "w") as f:
        _json.dump(data, f, indent=2)
    MAP_PAN_COEF  = np.array(result["pan_coef"],  dtype=np.float64)
    MAP_TILT_COEF = np.array(result["tilt_coef"], dtype=np.float64)
    HAS_PIXEL_MAP = True
    log(f"[OTO-KALİBRASYON] Harita kaydedildi: n={result['n_samples']} "
        f"rms={result['rms_pan_deg']}/{result['rms_tilt_deg']}°")


def _is_pixel_map_usable(result: dict | None) -> tuple[bool, str]:
    if result is None:
        return False, "fit oluşturulamadı"
    n_samples = int(result.get("n_samples", 0))
    rms_pan = float(result.get("rms_pan_deg", 999.0))
    rms_tilt = float(result.get("rms_tilt_deg", 999.0))
    px_span_x = float(result.get("px_span_x", 0.0))
    px_span_y = float(result.get("px_span_y", 0.0))
    if n_samples < AUTOCAL_MIN_SAMPLES:
        return False, f"çok az örnek ({n_samples} < {AUTOCAL_MIN_SAMPLES})"
    if px_span_x < AUTOCAL_MIN_PX_SPAN_X or px_span_y < AUTOCAL_MIN_PX_SPAN_Y:
        return False, (f"örnek yayılımı yetersiz "
                       f"(span={px_span_x:.0f}x{px_span_y:.0f}px < "
                       f"{AUTOCAL_MIN_PX_SPAN_X:.0f}x{AUTOCAL_MIN_PX_SPAN_Y:.0f}px)")
    if rms_pan > AUTOCAL_MAX_RMS_PAN_DEG or rms_tilt > AUTOCAL_MAX_RMS_TILT_DEG:
        return False, (f"yüksek RMS hata "
                       f"({rms_pan:.1f}/{rms_tilt:.1f}° > "
                       f"{AUTOCAL_MAX_RMS_PAN_DEG:.1f}/{AUTOCAL_MAX_RMS_TILT_DEG:.1f}°)")
    return True, "ok"


def _summarize_dot_samples(dots: list[tuple[float, float, float]]) -> tuple[tuple[float, float, float] | None, str | None]:
    if len(dots) < max(3, AUTOCAL_MEASURE_FR // 2):
        return None, f"yetersiz gözlem ({len(dots)})"
    xs = np.array([d[0] for d in dots], dtype=np.float64)
    ys = np.array([d[1] for d in dots], dtype=np.float64)
    areas = np.array([d[2] for d in dots], dtype=np.float64)
    mean_x = float(np.mean(xs))
    mean_y = float(np.mean(ys))
    mean_area = float(np.mean(areas))
    jitter = float(np.sqrt(np.mean((xs - mean_x) ** 2 + (ys - mean_y) ** 2)))
    if jitter > AUTOCAL_MAX_JITTER_PX:
        return None, f"yüksek jitter ({jitter:.1f}px)"
    return (mean_x, mean_y, mean_area), None


class AutoCalibrator:
    """Servoları ızgarada gezdirip her noktada lazer noktasını tespit eder,
    piksel→servo haritasını fit eder. Main loop her frame'de update() çağırır;
    dönen dict main loop'a hangi açıya gidileceğini ve lazer durumunu söyler."""

    def __init__(self) -> None:
        pans  = [PAN_MIN_DEG  + (PAN_MAX_DEG  - PAN_MIN_DEG)  * i / (AUTOCAL_GRID_PAN  - 1)
                 for i in range(AUTOCAL_GRID_PAN)]
        tilts = [TILT_MIN_DEG + (TILT_MAX_DEG - TILT_MIN_DEG) * j / (AUTOCAL_GRID_TILT - 1)
                 for j in range(AUTOCAL_GRID_TILT)]
        # Yılan (boustrophedon) sıralama — satırlar arası büyük sıçramayı azaltır
        self.grid: list[tuple[float, float]] = []
        for j, t in enumerate(tilts):
            row = pans if j % 2 == 0 else list(reversed(pans))
            self.grid += [(p, t) for p in row]
        self.i = 0
        self.samples: list[tuple[float, float, float, float]] = []
        self.ref = None
        self.phase = "ref"
        self.counter = AUTOCAL_REF_FR
        self.status = "Referans alınıyor (lazer kapalı)…"
        self.result = None
        self.dot_samples: list[tuple[float, float, float]] = []
        self.last_dot: tuple[float, float, float] | None = None

    def update(self, small_bgr) -> dict:
        n = len(self.grid)
        if self.phase == "ref":
            self.counter -= 1
            if self.counter <= 0:
                self.ref = small_bgr.copy()
                self.phase = "settle"
                self.counter = AUTOCAL_SETTLE_FR
                self.dot_samples = []
                p, t = self.grid[self.i]
                self.status = f"Tarama 1/{n}"
                return {"pan": p, "tilt": t, "laser": True, "done": False, "status": self.status}
            return {"laser": False, "done": False, "status": self.status}

        if self.phase == "settle":
            self.counter -= 1
            p, t = self.grid[self.i]
            if self.counter > 0:
                return {"pan": p, "tilt": t, "laser": True, "done": False, "status": self.status}
            self.phase = "measure"
            self.counter = AUTOCAL_MEASURE_FR
            self.dot_samples = []
            self.status = f"Tarama {self.i + 1}/{n} ölçülüyor"
            return {"pan": p, "tilt": t, "laser": True, "done": False, "status": self.status}

        if self.phase == "measure":
            p, t = self.grid[self.i]
            dot = _detect_laser_dot(small_bgr, self.ref)
            if dot is not None:
                self.dot_samples.append(dot)
                self.last_dot = dot
            self.counter -= 1
            if self.counter > 0:
                found = len(self.dot_samples)
                self.status = f"Tarama {self.i + 1}/{n} ölçülüyor ({found}/{AUTOCAL_MEASURE_FR})"
                return {"pan": p, "tilt": t, "laser": True, "done": False, "status": self.status}

            summary, err = _summarize_dot_samples(self.dot_samples)
            if summary is not None:
                self.samples.append((p, t, summary[0], summary[1]))
                log(f"[OTO-KAL] {self.i + 1}/{n}: pan={p:.0f}° tilt={t:.0f}° "
                    f"-> lazer px=({summary[0]:.1f},{summary[1]:.1f}) alan={summary[2]:.1f} "
                    f"obs={len(self.dot_samples)}")
            else:
                reason = err or "bilinmeyen hata"
                seen = len(self.dot_samples)
                log(f"[OTO-KAL] {self.i + 1}/{n}: pan={p:.0f}° tilt={t:.0f}° "
                    f"-> RED ({reason}, obs={seen})")
            self.i += 1
            if self.i >= n:
                self.phase = "done"
                self.result = _fit_pixel_to_servo(self.samples)
                return {"laser": False, "done": True,
                        "status": f"Bitti — {len(self.samples)}/{n} nokta bulundu",
                        "result": self.result}
            np_, nt = self.grid[self.i]
            self.phase = "settle"
            self.counter = AUTOCAL_SETTLE_FR
            self.dot_samples = []
            self.status = f"Tarama {self.i + 1}/{n} ({len(self.samples)} nokta bulundu)"
            return {"pan": np_, "tilt": nt, "laser": True, "done": False, "status": self.status}

        return {"laser": False, "done": True, "status": self.status, "result": self.result}


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

# Motor Açı Kontrolü (Başlangıç: pan merkez 0°, tilt alt limit 0°)
target_pan_deg = PAN_HOME_DEG
target_tilt_deg = TILT_HOME_DEG
current_pan_deg = PAN_HOME_DEG
current_tilt_deg = TILT_HOME_DEG

# Akıllı Lazer Takip Durum Makinesi Değişkenleri
current_target_id = None    # Şu an kilitlenilen sineğin benzersiz ID'si
lock_start_time = None      # Kilitlenme anının zaman damgası
killed_flies = set()        # 3 saniye boyunca vurularak imha edilen sineklerin ID listesi

# Kalibrasyon modu
calibrating: bool = False
calib_click_px: tuple[float, float] | None = None
auto_cal_active: bool = False          # web handler bunu set eder, main loop yakalar
auto_cal_status: str = ""              # otomatik kalibrasyon ilerleme mesajı (UI için)

# Piksel→servo açısı haritası (otomatik kalibrasyondan; ikinci derece polinom katsayıları)
HAS_PIXEL_MAP: bool = False
MAP_PAN_COEF = None                    # np.ndarray (6,) — [1, px, py, px², py², px·py]
MAP_TILT_COEF = None

# Web handler'ların thread-safe okuyabileceği durum snapshot'ı
web_status: dict = {}

# Nesne tanımlayıcı placeholder'lar (Main içinde global olarak set edilecek)
pan_servo = None
tilt_servo = None
lazer = None
sim_laser_on = False


def pulse_output(device, duration_s: float = 0.05) -> None:
    if device is None:
        return

    def _pulse():
        try:
            device.on()
            time.sleep(duration_s)
        finally:
            device.off()

    threading.Thread(target=_pulse, daemon=True).start()


class SyntheticCamera:
    """Picamera2 yerine kuru çalışma için basit hareketli sahne üretir."""

    def __init__(self) -> None:
        self.w = CAPTURE_W
        self.h = CAPTURE_H
        self.frame_idx = 0

    def create_video_configuration(self, main=None, controls=None, queue=False):
        if main and "size" in main:
            self.w, self.h = main["size"]
        return {"main": {"size": (self.w, self.h)}, "controls": controls or {}}

    def configure(self, cfg) -> None:
        main = cfg.get("main", {})
        if "size" in main:
            self.w, self.h = main["size"]

    def start(self) -> None:
        return None

    def stop(self) -> None:
        return None

    def set_controls(self, controls) -> None:
        return None

    def capture_array(self, stream_name="main"):
        self.frame_idx += 1
        img = np.full((self.h, self.w, 3), DRY_RUN_BG, dtype=np.uint8)
        cal_mode = auto_cal_active or calibrating

        if not cal_mode:
            # hareketli küçük koyu hedef
            x = int((self.frame_idx * 9) % max(self.w - 40, 1)) + 20
            y = int(self.h * 0.35 + 80 * math.sin(self.frame_idx * 0.08))
            cv2.circle(img, (x, y), 5, (20, 20, 20), -1)

            # ikinci hedef arada sahneye girsin
            if (self.frame_idx // 90) % 2 == 1:
                x2 = int(self.w * 0.75 + 60 * math.cos(self.frame_idx * 0.05))
                y2 = int(self.h * 0.60 + 45 * math.sin(self.frame_idx * 0.11))
                cv2.circle(img, (x2, y2), 4, (30, 30, 30), -1)

            # nadiren büyük motion bloğu üret, suppression akışını egzersiz etsin
            if (self.frame_idx // 150) % 3 == 2:
                bx = int(self.w * 0.1)
                by = int(self.h * 0.15)
                cv2.rectangle(img, (bx, by), (bx + 180, by + 140), (120, 120, 120), -1)

        # dry-run kalibrasyon için sentetik lazer noktası
        if cal_mode and sim_laser_on:
            with data_lock:
                pan = target_pan_deg
                tilt = target_tilt_deg
            px = int(((pan - PAN_MIN_DEG) / max(PAN_MAX_DEG - PAN_MIN_DEG, 1e-6)) * (self.w - 120) + 60)
            py = int(((tilt - TILT_MIN_DEG) / max(TILT_MAX_DEG - TILT_MIN_DEG, 1e-6)) * (self.h - 120) + 60)
            px = max(8, min(self.w - 9, px))
            py = max(8, min(self.h - 9, py))
            cv2.circle(img, (px, py), 4, (40, 40, 255), -1)

        # OpenCV sonrası kod RGB bekliyor
        return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

def pan_to_servo_angle(deg: float) -> float:
    """Pan gösterge açısı → AngularServo açısı (-90°..90°)."""
    return deg

def tilt_to_servo_angle(deg: float) -> float:
    """Tilt gösterge açısı → AngularServo açısı; mevcut 25° fiziksel offset korunur."""
    return deg + TILT_ZERO_OFFSET - 90.0


def ease(t: float) -> float:
    """Cubic ease-in-out: hareketin başında ve sonunda hızı sıfıra yaklaştırır."""
    t = max(0.0, min(1.0, t))
    return 4.0 * t * t * t if t < 0.5 else 1.0 - ((-2.0 * t + 2.0) ** 3) / 2.0

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
        return (float(pred[0, 0]), float(pred[1, 0]))

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
            tr.cx = float(corrected[0, 0])
            tr.cy = float(corrected[1, 0])
            tr.vx = float(corrected[2, 0])
            tr.vy = float(corrected[3, 0])
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
            tr.vx = float(tr.kf.statePost[2, 0])
            tr.vy = float(tr.kf.statePost[3, 0])
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

<div class="card" style="border-color:#1a4a1a">
  <h3>⚡ Otomatik Kalibrasyon (Önerilen)</h3>
  <div class="steps">
    Sistem servoları otomatik gezdirir, <b>kırmızı lazer noktasını kendisi bulur</b> ve
    piksel→servo haritasını çıkarır. Tek yapman gereken:<br>
    1. Lazerin vurduğu bölgeye düz bir yüzey (duvar/karton) koy — tarama menzilini görsün<br>
    2. Aşağıdaki butona bas, stream'de yeşil noktalar birikirken bekle (~20 sn)
  </div>
  <button class="btn btn-ok" style="margin-top:10px" onclick="startAutoCalib()">⚡ Otomatik Kalibrasyonu Başlat</button>
  <div id="auto-status" style="margin-top:10px;font-size:13px;color:#7ab">Hazır.</div>
</div>

<div class="card">
  <h3>Manuel Kalibrasyon (alternatif)</h3>
  <div class="steps">
    1. Kalibrasyonu Başlat → otomatik kalibrasyon varsa durur, sistem home konumuna gider, lazer açılır<br>
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
    document.getElementById('info').textContent="Lazer açık. Sistem home konumunda; stream'de lazer noktasına tıklayın."
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
function startAutoCalib(){
  document.getElementById('auto-status').textContent='Başlatılıyor… servolar hareket edecek, bekleyin.'
  fetch('/calibrate/auto',{method:'POST'}).then(r=>r.json()).then(d=>{
    if(d.status==='already_running')
      document.getElementById('auto-status').textContent='Zaten çalışıyor…'
    refreshStatus()
  })
}
function refreshStatus(){
  fetch('/calibrate/status').then(r=>r.json()).then(d=>{
    const ox=(d.laser_offset_px_x||0).toFixed(2),oy=(d.laser_offset_px_y||0).toFixed(2)
    document.getElementById('cal-status').innerHTML=
      (d.has_pixel_map?'✓ Piksel→servo haritası AKTİF':'Piksel haritası yok')
      +'<br>'+(d.has_calibration_file?'dosya: var':'dosya: yok')
      +'<br>offset_px: ('+ox+', '+oy+')'
    const a=document.getElementById('auto-status')
    if(d.auto_cal_active) a.textContent='⏳ '+(d.auto_cal_status||'çalışıyor…')
    else if(d.auto_cal_status) a.textContent=d.auto_cal_status
  }).catch(()=>{})
}
refreshStatus();setInterval(refreshStatus,1000)
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
                    "has_pixel_map":      HAS_PIXEL_MAP,
                    "auto_cal_active":    auto_cal_active,
                    "auto_cal_status":    auto_cal_status,
                })
            else:
                self.send_error(404)

        def do_POST(self):
            global calibrating, calib_click_px, current_target_id, killed_flies
            global LASER_OFFSET_PX_X, LASER_OFFSET_PX_Y
            global auto_cal_active, auto_cal_status
            global target_pan_deg, target_tilt_deg
            global sim_laser_on

            if self.path == "/laser/off":
                with data_lock:
                    current_target_id = None
                    target_pan_deg    = PAN_HOME_DEG
                    target_tilt_deg   = TILT_HOME_DEG
                    sim_laser_on      = False
                if HAS_GPIO and lazer:
                    lazer.off()
                log("[WEB] Lazer zorla kapatıldı.")
                self._json_ok({"status": "laser_off"})

            elif self.path == "/laser/on":
                if calibrating or auto_cal_active:
                    self._json_ok({"status": "error", "msg": "Kalibrasyon sırasında manuel lazer açılamaz"})
                    return
                sim_laser_on = True
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
                if auto_cal_active:
                    self._json_ok({"status": "error", "msg": "Önce otomatik kalibrasyonu iptal et"})
                    return
                with data_lock:
                    auto_cal_active   = False
                    auto_cal_status   = ""
                    calibrating       = True
                    calib_click_px    = None
                    current_target_id = None
                    target_pan_deg    = PAN_HOME_DEG
                    target_tilt_deg   = TILT_HOME_DEG
                    sim_laser_on      = True
                if HAS_GPIO and lazer:
                    lazer.on()
                log("[KALİBRASYON] Kalibrasyon modu başlatıldı.")
                self._json_ok({"status": "calibrating"})

            elif self.path == "/calibrate/click":
                if not calibrating or auto_cal_active:
                    self._json_ok({"status": "error", "msg": "Manuel kalibrasyon aktif değil"})
                    return
                length = int(self.headers.get("Content-Length", 0))
                body   = _json.loads(self.rfile.read(length))
                with data_lock:
                    calib_click_px = (float(body["x"]), float(body["y"]))
                self._json_ok({"status": "click_received", "x": body["x"], "y": body["y"]})

            elif self.path == "/calibrate/confirm":
                if not calibrating or auto_cal_active:
                    self._json_ok({"status": "error", "msg": "Onay için manuel kalibrasyon aktif olmalı"})
                    return
                length = int(self.headers.get("Content-Length", 0))
                body   = _json.loads(self.rfile.read(length)) if length else {}
                distance_cm = int(body.get("distance_cm", 150))
                with data_lock:
                    click       = calib_click_px
                    calibrating = False
                    sim_laser_on = False
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
                    calibrating     = False
                    calib_click_px  = None
                    auto_cal_active = False
                    target_pan_deg  = PAN_HOME_DEG
                    target_tilt_deg = TILT_HOME_DEG
                    sim_laser_on    = False
                if HAS_GPIO and lazer:
                    lazer.off()
                log("[KALİBRASYON] İptal edildi.")
                self._json_ok({"status": "cancelled"})

            elif self.path == "/calibrate/auto":
                if calibrating:
                    self._json_ok({"status": "error", "msg": "Önce manuel kalibrasyonu iptal et"})
                    return
                if auto_cal_active:
                    self._json_ok({"status": "already_running"})
                    return
                with data_lock:
                    calibrating       = False   # manuel mod ile çakışmasın
                    calib_click_px    = None
                    current_target_id = None
                    target_pan_deg    = PAN_HOME_DEG
                    target_tilt_deg   = TILT_HOME_DEG
                    auto_cal_active   = True
                    auto_cal_status   = "Başlatılıyor…"
                    sim_laser_on      = False
                log("[OTO-KALİBRASYON] Web'den başlatıldı.")
                self._json_ok({"status": "auto_calibrating"})

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


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Mosquito laser turret runtime")
    p.add_argument("--dry-run", action="store_true",
                   help="Picamera2/gpio olmadan sentetik kamera ile çalış")
    p.add_argument("--no-stream", action="store_true",
                   help="HTTP MJPEG server başlatma")
    p.add_argument("--stream-port", type=int, default=STREAM_PORT,
                   help="MJPEG HTTP portu")
    return p


# =========================================================================
# THREAD: MOTORLARIN CUBIC EASE-IN-OUT HAREKET DÖNGÜSÜ
# =========================================================================
def smooth_move(pan_to: float, tilt_to: float,
                duration: float = SERVO_MOVE_DURATION,
                steps: int = SERVO_MOVE_STEPS) -> None:
    """İki ekseni aynı zaman çizelgesinde cubic ease-in-out ile hedefe götürür."""
    global current_pan_deg, current_tilt_deg

    pan_to = max(PAN_MIN_DEG, min(PAN_MAX_DEG, pan_to))
    tilt_to = max(TILT_MIN_DEG, min(TILT_MAX_DEG, tilt_to))
    pan_from = current_pan_deg
    tilt_from = current_tilt_deg
    steps = max(1, int(steps))
    step_delay = max(0.0, float(duration)) / steps

    for i in range(steps + 1):
        if not is_running:
            break
        k = ease(i / steps)
        current_pan_deg = pan_from + (pan_to - pan_from) * k
        current_tilt_deg = tilt_from + (tilt_to - tilt_from) * k
        pan_servo.angle = pan_to_servo_angle(current_pan_deg)
        tilt_servo.angle = tilt_to_servo_angle(current_tilt_deg)
        if i < steps and step_delay:
            time.sleep(step_delay)


def motor_smooth_thread():
    """Ana döngüden bağımsız olarak en güncel hedefe eased hareket uygular."""
    global target_pan_deg, target_tilt_deg, current_pan_deg, current_tilt_deg, is_running

    pan_active = False
    tilt_active = False

    while is_running:
        with data_lock:
            t_pan = max(PAN_MIN_DEG, min(PAN_MAX_DEG, target_pan_deg))
            t_tilt = max(TILT_MIN_DEG, min(TILT_MAX_DEG, target_tilt_deg))

        if not HAS_GPIO or pan_servo is None or tilt_servo is None:
            current_pan_deg = t_pan
            current_tilt_deg = t_tilt
            time.sleep(0.005)
            continue

        pan_moving = abs(t_pan - current_pan_deg) > 0.05
        tilt_moving = abs(t_tilt - current_tilt_deg) > 0.05
        if pan_moving or tilt_moving:
            smooth_move(t_pan, t_tilt)
            # smooth_move iki servoya da açı yazar; ikisini de sonraki turda detach et.
            pan_active = True
            tilt_active = True
            continue

        if pan_active:
            pan_servo.detach()
            pan_active = False
        if tilt_active:
            tilt_servo.detach()
            tilt_active = False
        time.sleep(0.005)


# =========================================================================
# Main
# =========================================================================

def main(argv=None) -> None:
    global current_target_id, lock_start_time, target_pan_deg, target_tilt_deg, is_running
    global pan_servo, tilt_servo, lazer
    global auto_cal_active, auto_cal_status
    global sim_laser_on
    global STREAM_PORT

    args = build_parser().parse_args(argv)
    dry_run = bool(args.dry_run)
    STREAM_PORT = int(args.stream_port)
    enable_stream = ENABLE_STREAM and not args.no_stream

    _load_calibration()

    # Donanım Başlatma Kontrolleri
    trigger = LED(TRIGGER_PIN) if HAS_GPIO and not dry_run else None
    
    if HAS_GPIO and not dry_run:
        # AngularServo açı API'si; Pi 5 uyumlu lgpio backend ve kalibre edilmiş pulse aralığı.
        pan_servo = AngularServo(
            PAN_PIN, min_angle=-90, max_angle=90, initial_angle=None,
            min_pulse_width=SERVO_MIN_PW, max_pulse_width=SERVO_MAX_PW,
        )
        tilt_servo = AngularServo(
            TILT_PIN, min_angle=-90, max_angle=90, initial_angle=None,
            min_pulse_width=SERVO_MIN_PW, max_pulse_width=SERVO_MAX_PW,
        )
        lazer = LED(LAZER_PIN)

        # İlk konum: pan merkez (0°), tilt alt limit (0°); sonra enerjiyi geçici olarak kes
        pan_servo.angle = pan_to_servo_angle(PAN_HOME_DEG)
        tilt_servo.angle = tilt_to_servo_angle(TILT_HOME_DEG)
        time.sleep(0.3)
        pan_servo.detach()
        tilt_servo.detach()
        log("Servolar ve Lazer başarıyla ilklendirildi.")
    else:
        reason = "dry-run aktif" if dry_run else "gpiozero kütüphanesi yüklenemedi"
        log(f"UYARI: {reason}, donanım kontrolü simüle edilecek.")

    # Bağımsız Motor Kontrol Thread'ini Başlatıyoruz
    t_motor = threading.Thread(target=motor_smooth_thread)
    t_motor.start()

    if dry_run:
        picam2 = SyntheticCamera()
        log("Dry-run: sentetik kamera etkin.")
    else:
        if not HAS_PICAMERA2:
            raise RuntimeError("Picamera2 kullanılamıyor. Dry-run için --dry-run kullan.")
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
    httpd = None
    if enable_stream:
        try:
            httpd = start_stream(buf_main, buf_debug, STREAM_PORT)
        except OSError as e:
            if dry_run:
                log(f"UYARI: stream başlatılamadı ({e}). Dry-run stream'siz devam ediyor.")
            else:
                raise

    pipeline = MaskPipeline()
    tracker = Tracker()
    encode = [int(cv2.IMWRITE_JPEG_QUALITY), STREAM_QUALITY]

    sx = CAPTURE_W / PROCESS_W
    sy = CAPTURE_H / PROCESS_H
    max_motion_px = int(MAX_GLOBAL_MOTION_RATIO * PROCESS_W * PROCESS_H)

    frame_idx = 0
    exposure_settle = 0          # >0 ise pozlama yeni değişti, frame-diff'i bu kadar frame susturmak için
    auto_cal = None              # AutoCalibrator instance (aktifken)
    last_trigger = 0.0
    lock_visible_time = 0.0      # kilitli hedef frame'de gerçekten görülürken biriken süre
    last_loop_time = time.time()
    last_lock_log = 0.0
    fps = 0.0
    _fps_window = 60
    _frame_times: deque = deque(maxlen=_fps_window + 1)
    n_suppressed = 0
    biggest_history: deque = deque(maxlen=TARGET_FPS)
    display_max = (0, 0, 0)
    display_max_t = time.time()

    try:
        while True:
            loop_now = time.time()
            dt = max(0.0, loop_now - last_loop_time)
            last_loop_time = loop_now

            rgb = picam2.capture_array("main")
            frame = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            small = cv2.resize(frame, (PROCESS_W, PROCESS_H),
                               interpolation=cv2.INTER_AREA)
            gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)

            # B2: Pozlama yeni değiştiyse görüntü parlaklığı zıplar — bu frame'lerde
            # frame-diff'i sıfırla ki tüm-frame parlaklık sıçraması sahte motion üretmesin.
            if exposure_settle > 0:
                pipeline.prev_blur = None
                exposure_settle -= 1

            enhanced, motion, motion_exclude, blackhat, combined = pipeline.process(gray)

            if frame_idx < WARMUP_FRAMES:
                frame_idx += 1
                continue

            # İptal/bitiş sonrası bayat instance kalmasın — sonraki başlatma temiz olsun
            if not auto_cal_active and auto_cal is not None:
                auto_cal = None

            # OTOMATİK KALİBRASYON MODU — servoları gezdirip lazer noktasıyla harita fit eder
            if auto_cal_active:
                if auto_cal is None:
                    auto_cal = AutoCalibrator()
                    log("[OTO-KALİBRASYON] Başladı.")
                cmd = auto_cal.update(small)

                if "pan" in cmd:
                    with data_lock:
                        target_pan_deg  = cmd["pan"]
                        target_tilt_deg = cmd["tilt"]
                        sim_laser_on    = bool(cmd.get("laser"))
                if HAS_GPIO and lazer:
                    lazer.on() if cmd.get("laser") else lazer.off()

                if enable_stream:
                    overlay = frame.copy()
                    dot = auto_cal.last_dot or _detect_laser_dot(small, auto_cal.ref)
                    if dot is not None:
                        cv2.drawMarker(overlay, (int(dot[0] * sx), int(dot[1] * sy)),
                                       (0, 0, 255), cv2.MARKER_CROSS, 30, 2)
                    for (px, py, _a) in auto_cal.dot_samples:
                        cv2.circle(overlay, (int(px * sx), int(py * sy)), 2, (0, 180, 255), -1)
                    for (_p, _t, px, py) in auto_cal.samples:
                        cv2.circle(overlay, (int(px * sx), int(py * sy)), 3, (0, 255, 0), -1)
                    cv2.putText(overlay, "OTO-KALIBRASYON: " + cmd["status"], (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)
                    ok, jpg = cv2.imencode(".jpg", overlay, encode)
                    if ok:
                        buf_main.update(jpg.tobytes())

                if cmd.get("done"):
                    res = cmd.get("result")
                    usable, reason = _is_pixel_map_usable(res)
                    if usable:
                        _save_pixel_map(res)
                        auto_cal_status = (f"Tamamlandı — {res['n_samples']} nokta, "
                                           f"rms {res['rms_pan_deg']}/{res['rms_tilt_deg']}°")
                    else:
                        auto_cal_status = f"BAŞARISIZ — {reason}"
                        log(f"[OTO-KALİBRASYON] Başarısız: {reason}.")
                    auto_cal_active = False
                    auto_cal = None
                    sim_laser_on = False
                    if HAS_GPIO and lazer:
                        lazer.off()
                    with data_lock:
                        target_pan_deg  = PAN_HOME_DEG
                        target_tilt_deg = TILT_HOME_DEG
                else:
                    auto_cal_status = cmd["status"]

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
                if enable_stream:
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
                # Track'leri silme — Kalman ile coast et, MAX_MISSED dolunca kendiliğinden düşer.
                # Böylece tek frame'lik gürültü (el geçişi, ışık titremesi) kilidi yok etmez.
                tracker.update([], frame_idx)
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
            aim_error_x = None
            aim_error_y = None
            target_misses = None

            # ADIM 1: Eğer halihazırda kilitli bir hedefimiz varsa onu takip et.
            # Track coast ediyor olsa bile (misses>0, henüz MAX_MISSED'a ulaşmadı)
            # Kalman tahminiyle takibe devam et — kısa suppression kilidi öldürmez.
            if current_target_id is not None:
                tr = tracker.tracks.get(current_target_id)
                if tr is not None:
                    target_still_visible = True
                    target_misses = tr.misses
                    if tr.misses == 0:
                        lock_visible_time += dt

                    if lock_visible_time >= 3.0:
                        # 3 saniye kesintisiz vurduk, sinek imha edildi!
                        if HAS_GPIO and lazer:
                            lazer.off()
                        sim_laser_on = False
                        killed_flies.add(tr.track_id)
                        log(f"[İMHA EDİLDİ] ID: {tr.track_id} 3 saniye boyunca vuruldu. Kilit açıldı.")
                        current_target_id = None
                        lock_visible_time = 0.0
                        with data_lock:
                            target_pan_deg  = PAN_HOME_DEG
                            target_tilt_deg = TILT_HOME_DEG
                        if HAS_GPIO and pan_servo and tilt_servo:
                            pan_servo.detach()
                            tilt_servo.detach()
                    elif tr.misses == 0 and HAS_PIXEL_MAP:
                        # Doğru yöntem (sabit kamera): piksel→servo haritasıyla doğrudan nişan al.
                        # Lazeri sineğin pikseline götürecek mutlak açıyı hesapla.
                        aim_error_x = float(tr.cx - PROCESS_W / 2)
                        aim_error_y = float(tr.cy - PROCESS_H / 2)
                        want_pan, want_tilt = pixel_to_servo(tr.cx, tr.cy)
                        with data_lock:
                            target_pan_deg  = max(PAN_MIN_DEG,  min(PAN_MAX_DEG,  want_pan))
                            target_tilt_deg = max(TILT_MIN_DEG, min(TILT_MAX_DEG, want_tilt))
                    elif tr.misses == 0:
                        # Harita yoksa eski oransal mod (sabit kamerada geometrik olarak hatalı,
                        # sadece geriye dönük uyumluluk için — otomatik kalibrasyon önerilir).
                        error_x = tr.cx - (PROCESS_W / 2 + LASER_OFFSET_PX_X)
                        error_y = tr.cy - (PROCESS_H / 2 + LASER_OFFSET_PX_Y)
                        aim_error_x = float(error_x)
                        aim_error_y = float(error_y)
                        kp_x = adaptive_kp(error_x)
                        kp_y = adaptive_kp(error_y)
                        with data_lock:
                            target_pan_deg  = max(PAN_MIN_DEG,  min(PAN_MAX_DEG,
                                                  target_pan_deg  - (error_x * kp_x)))
                            target_tilt_deg = max(TILT_MIN_DEG, min(TILT_MAX_DEG,
                                                  target_tilt_deg - (error_y * kp_y)))

                    if tr.misses == 0 and now - last_lock_log >= 1.0:
                        last_lock_log = now
                        if aim_error_x is not None and aim_error_y is not None:
                            log(f"[LOCK] id={tr.track_id} vis={lock_visible_time:.1f}s "
                                f"err=({aim_error_x:.1f},{aim_error_y:.1f})px "
                                f"mode={'map' if HAS_PIXEL_MAP else 'offset'}")

            # ADIM 2: Eğer kilitli sinek gerçekten kadrajdan çıktıysa kilidi temizle
            if current_target_id is not None and not target_still_visible:
                if HAS_GPIO and lazer and pan_servo and tilt_servo:
                    lazer.off()
                    pan_servo.detach()
                    tilt_servo.detach()
                sim_laser_on = False
                log(f"[HEDEF KAÇTI] ID {current_target_id} gözden kayboldu. Lazer Kapatıldı.")
                current_target_id = None
                lock_visible_time = 0.0
                with data_lock:
                    target_pan_deg  = PAN_HOME_DEG
                    target_tilt_deg = TILT_HOME_DEG

            # ADIM 3: Eğer sistem boştaysa ve yeni bir aday sinek geldiyse kilitlen
            if current_target_id is None:
                candidates = [
                    tr for tr in confirmed
                    if tr.track_id not in killed_flies
                ]
                if candidates:
                    best = max(candidates, key=candidate_score)
                    best_score = candidate_score(best)
                    current_target_id = best.track_id
                    lock_start_time = now
                    lock_visible_time = 0.0
                    target_still_visible = True  # Aynı frame içerisinde hedef kaçtı kontrolüne düşmemesi için
                    if HAS_GPIO and lazer:
                        lazer.on()
                    sim_laser_on = True
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
                    "sim_laser": sim_laser_on,
                    "target_id": current_target_id,
                    "killed":    len(killed_flies),
                    "pan_deg":   round(current_pan_deg, 1),
                    "tilt_deg":  round(current_tilt_deg, 1),
                    "calibrating": calibrating,
                    "auto_cal_active": auto_cal_active,
                    "mode": "auto_cal" if auto_cal_active else ("manual_cal" if calibrating else "run"),
                    "dry_run": dry_run,
                    "suppressed": suppressed,
                    "target_misses": target_misses,
                    "lock_visible_s": round(lock_visible_time, 2),
                    "aim_error_x": round(aim_error_x, 1) if aim_error_x is not None else None,
                    "aim_error_y": round(aim_error_y, 1) if aim_error_y is not None else None,
                    "aim_mode": "map" if HAS_PIXEL_MAP else "offset",
                })

            # Orijinal röle tetikleyici mekanizması
            for tr in confirmed:
                if not tr.triggered and (now - last_trigger) > TRIGGER_COOLDOWN:
                    if trigger is not None:
                        pulse_output(trigger, 0.05)
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
                    cv2.putText(frame, f"LASER LOCK {lock_visible_time:.1f}s", (px - 30, py - 20),
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

            if enable_stream:
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
                if _adjust_exposure(picam2, float(np.mean(gray))):
                    exposure_settle = 3   # kamera yeni pozlamaya oturana dek frame-diff sustur
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
        sim_laser_on = False
        
        if HAS_GPIO and lazer and pan_servo and tilt_servo:
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
