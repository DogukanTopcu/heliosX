"""
v2 — Fly detection (motion + blackhat + adaptive + trajectory)
Raspberry Pi 5 + Camera Module 3, CPU-only.

Algoritma:
  1) Frame differencing -> motion_mask (lighting drift'e dayanikli)
  2) CLAHE + black-hat morfoloji -> dark blob mask (sinek koyu)
  3) Adaptive threshold -> local-dark mask
  4) AND(motion, blackhat, adaptive) -> detection mask
     => El/kol kenari: motion var, ama blackhat ve adaptive yok -> elenir
     => Sinek: 3'unde de aktif -> gecer
  5) Velocity tracker (pozisyon tahminli eslesme)
  6) Trajectory check: track'in toplam yol uzunlugu ve yon degisim sayisi
     => Statik edge titreme: yer degistirmez -> elenir
     => Sinek: zigzag erratik hareket -> gecer

Browser stream:
  http://heliosx.local:8080/         -> ana goruntu
  http://heliosx.local:8080/debug    -> debug mask
"""

import math
import time
import threading
from collections import deque
from dataclasses import dataclass, field
from http import server
from socketserver import ThreadingMixIn

import cv2
import numpy as np
from picamera2 import Picamera2

try:
    from gpiozero import LED
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
# (yetersiz isikta gorunmez olabilir, lighting kritik)
EXPOSURE_TIME_US = 5000               # mikrosaniye (5ms shutter). 0 = auto
ANALOGUE_GAIN = 6.0                   # isiklandirma kompansasyonu (1-16)

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
# Once motion'i agresif dilate ederek yakin fragmentleri birlestir,
# sonra component analizi yap. Sinek izole oldugu icin kucuk kalir,
# parmak/el fragmentleri tek buyuk component'e birleser.
# Gozlemlenen degerler (4 iter 5x5 dilate sonrasi):
#   tek sinek (old+new pos diff merge): area~1700 w/h~50
#   el (insan): area~16000 w/h~180
# 10x bosluk -> orta noktayi sec
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

# Tetikleme
TRIGGER_COOLDOWN = 2.0
TRIGGER_PIN = 17

# Stream / log
ENABLE_STREAM = True
STREAM_PORT = 8080
STREAM_QUALITY = 75
WARMUP_FRAMES = 15                    # detection bu kadar frame sonra basla
LOG_TO_FILE = True
LOG_PATH = "/home/heliosx/v2/detections.log"


# =========================================================================
# Yardimcilar
# =========================================================================

def log(msg: str) -> None:
    line = f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] {msg}"
    print(line, flush=True)
    if LOG_TO_FILE:
        with open(LOG_PATH, "a") as f:
            f.write(line + "\n")


def ensure_odd(n: int, minimum: int = 3) -> int:
    n = max(n, minimum)
    return n if n % 2 == 1 else n + 1


# =========================================================================
# Detection
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

    def predict(self) -> tuple[float, float]:
        return (self.cx + self.vx, self.cy + self.vy)

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
            new_vx = d.cx - tr.cx
            new_vy = d.cy - tr.cy
            tr.vx = 0.5 * tr.vx + 0.5 * new_vx
            tr.vy = 0.5 * tr.vy + 0.5 * new_vy
            tr.cx, tr.cy = float(d.cx), float(d.cy)
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
            tr.vx *= 0.7
            tr.vy *= 0.7
            tr.cx += tr.vx
            tr.cy += tr.vy
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

        # Gurultu temizle (yuksek gain noise'unu at)
        motion_clean = cv2.morphologyEx(motion_raw, cv2.MORPH_OPEN, self.k3, iterations=1)

        blackhat = cv2.morphologyEx(enhanced, cv2.MORPH_BLACKHAT, self.bh_kernel)
        _, dark = cv2.threshold(blackhat, BLACK_THRESHOLD, 255, cv2.THRESH_BINARY)

        adaptive = cv2.adaptiveThreshold(
            enhanced, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV,
            ensure_odd(ADAPTIVE_BLOCK, 5), ADAPTIVE_C,
        )

        # AND icin temiz motion + hafif dilation
        motion_dil = cv2.dilate(motion_clean, self.k3, iterations=MOTION_DILATE)
        combined = cv2.bitwise_and(motion_dil, dark)
        combined = cv2.bitwise_and(combined, adaptive)
        combined = cv2.morphologyEx(combined, cv2.MORPH_OPEN, self.k3, iterations=1)
        combined = cv2.dilate(combined, self.k3, iterations=1)

        # Exclusion analizi icin AGRESIF dilate - parmak/el fragmentlerini birlestir
        motion_exclude = cv2.dilate(
            motion_clean, self.k_exclude, iterations=EXCLUSION_DILATE_ITERS
        )
        return enhanced, motion_dil, motion_exclude, blackhat, combined


# =========================================================================
# MJPEG Stream
# =========================================================================

class FrameBuffer:
    def __init__(self) -> None:
        self.frame: bytes | None = None
        self.cond = threading.Condition()

    def update(self, b: bytes) -> None:
        with self.cond:
            self.frame = b
            self.cond.notify_all()


PAGE = """<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>v2 fly</title>
<style>body{background:#111;color:#eee;font-family:system-ui;margin:0;padding:16px}
img{max-width:100%;border:1px solid #333;border-radius:6px;margin-bottom:12px}
.col{display:flex;flex-direction:column;gap:8px}.meta{font-size:12px;color:#888}
a{color:#6cf}</style></head><body>
<h2>v2 fly stream</h2>
<div class="col">
  <img src="/stream.mjpg"/>
  <div class="meta">Yesil bbox: aday track. Kirmizi: trajectory onayli sinek (tetiklenen).
  <a href="/debug">/debug</a> -> mask gorunumu</div>
</div></body></html>"""

DEBUG_PAGE = """<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>v2 debug</title>
<style>body{background:#111;color:#eee;font-family:system-ui;margin:0;padding:16px}
img{max-width:100%;border:1px solid #333;border-radius:6px}
a{color:#6cf}</style></head><body>
<h2>v2 debug mask</h2><img src="/debug.mjpg"/>
<div><a href="/">geri</a></div></body></html>"""


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

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                self._send_page(PAGE)
            elif self.path == "/debug":
                self._send_page(DEBUG_PAGE)
            elif self.path == "/stream.mjpg":
                self._stream(buf_main)
            elif self.path == "/debug.mjpg":
                self._stream(buf_debug)
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
# Main
# =========================================================================

def main() -> None:
    trigger = LED(TRIGGER_PIN) if HAS_GPIO else None
    if trigger is None:
        log("UYARI: gpiozero yok, tetikleme devre disi.")

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
    t0 = time.time()
    fps = 0.0
    n_suppressed = 0
    # Son ~1 saniyenin maks degerleri (okunabilir diagnostic)
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

            total_motion = int(cv2.countNonZero(motion))
            suppressed = total_motion > max_motion_px

            # Buyuk motion bilesenlerini exclusion zone olarak isaretle
            # AGRESIF dilate'lenmis motion uzerinde -> parmak fragmentleri birlesir
            exclusion_zones: list[tuple[int, int, int, int]] = []
            biggest_comp = (0, 0, 0)  # (area, w, h) - diagnostic
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
                    # Exclusion zone'da mi?
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
                # Exclusion zone'a giren mevcut track'lari at
                if exclusion_zones:
                    for tid in list(tracker.tracks.keys()):
                        tr = tracker.tracks[tid]
                        if in_exclusion(int(tr.cx), int(tr.cy)):
                            tracker.tracks.pop(tid)
                tracker.update(dets, frame_idx)

            confirmed = tracker.confirmed(frame_idx)

            now = time.time()
            for tr in confirmed:
                if not tr.triggered and (now - last_trigger) > TRIGGER_COOLDOWN:
                    log(f"FLY id={tr.track_id} pos=({int(tr.cx)},{int(tr.cy)}) "
                        f"hits={tr.hits} path={tr.path_length():.1f} "
                        f"dirs={tr.direction_changes()} spd={tr.speed():.1f}")
                    if trigger is not None:
                        trigger.on()
                        time.sleep(0.05)
                        trigger.off()
                    last_trigger = now
                    tr.triggered = True

            # --- Overlay ---
            # Exclusion zonelari (yari saydam kirmizi)
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
                color = (0, 200, 200) if tr.hits < MIN_HITS else (0, 255, 0)
                cv2.circle(frame, (px, py), 12, color, 1)
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
            # Rolling max - okunabilir diagnostic
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

                # debug: combined mask + motion overlayed
                dbg = cv2.cvtColor(combined, cv2.COLOR_GRAY2BGR)
                # motion'i mavi olarak ekle (gormek icin)
                dbg[motion > 0] = (200, 80, 0)
                # combined olan yerleri yesile boya
                dbg[combined > 0] = (0, 255, 0)
                # contour bbox'larini ciz
                for d in dets:
                    cv2.rectangle(dbg, (d.x, d.y), (d.x + d.w, d.y + d.h),
                                  (0, 0, 255), 1)
                ok, jpg = cv2.imencode(".jpg", dbg, encode)
                if ok:
                    buf_debug.update(jpg.tobytes())

            frame_idx += 1
            if frame_idx % 30 == 0:
                fps = frame_idx / (time.time() - t0)
            if frame_idx % 300 == 0:
                log(f"FPS~{fps:.1f} trk:{len(tracker.tracks)} "
                    f"dets:{len(dets)} suppr_frames:{n_suppressed}")

    except KeyboardInterrupt:
        log("Durduruldu (Ctrl+C).")
    finally:
        if httpd is not None:
            httpd.shutdown()
        picam2.stop()
        if trigger is not None:
            trigger.off()


if __name__ == "__main__":
    main()
