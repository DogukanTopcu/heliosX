"""
Ok tuşlu manuel servo testi — pan/tilt'i ok tuşlarıyla sürer ve canlı pulse
genişliğini (µs) gösterir. Servonun nerede zorlandığını gözle görmek için
mekanik aralık geniş tutuldu (CLAUDE.md doğrulanmış: pan ±45°, tilt 0–75°).

ÖNEMLİ: Çalıştırmadan önce turret.service'i durdur (aynı GPIO çakışması olmasın):
    sudo systemctl stop turret.service

Kontroller:
    ← / →   Pan (sol / sağ)
    ↑ / ↓   Tilt (yukarı / aşağı)
    + / -   Adım büyüklüğü (derece) artır/azalt
    p / t   pan / tilt'i mekanik limite kadar otomatik tara (yavaş)
    c       Merkeze / alt limite dön
    q       Çıkış (servoları detach eder)
"""

import os
os.environ["GPIOZERO_PIN_FACTORY"] = "lgpio"

from gpiozero import Servo
import sys
import tty
import termios
import threading
import time

# --- Servo pulse aralığı (new2.py / manual-control5.py ile aynı) ---
MIN_PW = 0.0005  # 0.5 ms
MAX_PW = 0.0025  # 2.5 ms

pan_servo  = Servo(12, min_pulse_width=MIN_PW, max_pulse_width=MAX_PW)
tilt_servo = Servo(13, min_pulse_width=MIN_PW, max_pulse_width=MAX_PW)

# --- Test için GENİŞ aralık: gerçek mekanik sınırı kendin bul ---
# CLAUDE.md "doğrulanmış donanım gerçekleri": pan -45..45, tilt 0..75
PAN_MIN_DEG, PAN_MAX_DEG = -45.0, 45.0
TILT_MIN_DEG, TILT_MAX_DEG = 0.0, 75.0
TILT_ZERO_OFFSET = 25.0          # gösterge 0° = fiziksel alt limit

step_deg = 1.0                   # ok tuşu başına derece (+/- ile değişir)

target_pan_deg = 0.0
target_tilt_deg = 0.0
current_pan_deg = 0.0
current_tilt_deg = 0.0

is_running = True
data_lock = threading.Lock()


def pan_to_servo_val(deg: float) -> float:
    return deg / 90.0


def tilt_to_servo_val(deg: float) -> float:
    return ((deg + TILT_ZERO_OFFSET) / 90.0) - 1.0


def servo_val_to_pw_us(v: float) -> float:
    """gpiozero Servo value (-1..1) -> pulse genişliği (mikrosaniye)."""
    v = max(-1.0, min(1.0, v))
    pw = MIN_PW + (v + 1.0) / 2.0 * (MAX_PW - MIN_PW)
    return pw * 1e6


def motor_smooth_thread() -> None:
    global current_pan_deg, current_tilt_deg
    smooth_factor = 0.5
    threshold = 0.05
    while is_running:
        with data_lock:
            t_pan, t_tilt = target_pan_deg, target_tilt_deg
        pan_diff = t_pan - current_pan_deg
        if abs(pan_diff) > threshold:
            current_pan_deg += pan_diff * smooth_factor
            pan_servo.value = pan_to_servo_val(current_pan_deg)
        tilt_diff = t_tilt - current_tilt_deg
        if abs(tilt_diff) > threshold:
            current_tilt_deg += tilt_diff * smooth_factor
            tilt_servo.value = tilt_to_servo_val(current_tilt_deg)
        time.sleep(0.005)


def read_key() -> str:
    """Tek tuş okur; ok tuşlarını 'UP/DOWN/LEFT/RIGHT' olarak döndürür."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
        if ch == "\x1b":                    # ESC — ok tuşu olabilir
            seq = sys.stdin.read(2)
            return {"[A": "UP", "[B": "DOWN",
                    "[C": "RIGHT", "[D": "LEFT"}.get(seq, "ESC")
        return ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def print_status() -> None:
    pan_pw = servo_val_to_pw_us(pan_to_servo_val(target_pan_deg))
    tilt_pw = servo_val_to_pw_us(tilt_to_servo_val(target_tilt_deg))
    print(f"\r  PAN {target_pan_deg:>+6.1f}° ({pan_pw:4.0f}µs)  |  "
          f"TILT {target_tilt_deg:>6.1f}° ({tilt_pw:4.0f}µs)  |  "
          f"adim {step_deg:.1f}°     ",
          end="", flush=True)


def sweep(axis: str) -> None:
    """Bir ekseni min→max→min yavaşça tarar; zorlanma noktasını dinleyerek bul."""
    global target_pan_deg, target_tilt_deg
    if axis == "pan":
        lo, hi = PAN_MIN_DEG, PAN_MAX_DEG
        seq = [lo, hi, 0.0]
    else:
        lo, hi = TILT_MIN_DEG, TILT_MAX_DEG
        seq = [lo, hi, 0.0]
    for tgt in seq:
        with data_lock:
            if axis == "pan":
                target_pan_deg = tgt
            else:
                target_tilt_deg = tgt
        print_status()
        time.sleep(1.2)


def main() -> None:
    global target_pan_deg, target_tilt_deg, step_deg, is_running

    pan_servo.value = pan_to_servo_val(target_pan_deg)
    tilt_servo.value = tilt_to_servo_val(target_tilt_deg)
    time.sleep(0.5)

    t = threading.Thread(target=motor_smooth_thread, daemon=True)
    t.start()

    print("=== Ok Tuslu Manuel Servo Testi ===")
    print("< / > Pan  |  ^ / v Tilt  |  +/- adim  |  p pan-tara  t tilt-tara  |  c merkez  |  q cik")
    print(f"Aralik  Pan: {PAN_MIN_DEG:+.0f}..{PAN_MAX_DEG:+.0f}°   "
          f"Tilt: {TILT_MIN_DEG:.0f}..{TILT_MAX_DEG:.0f}°   "
          f"(pulse {MIN_PW*1000:.1f}-{MAX_PW*1000:.1f} ms)")
    print("Servo bir noktada vinliyor/titriyorsa orasi mekanik limittir — not al.")
    print("-" * 70)
    print_status()

    try:
        while True:
            key = read_key()
            if key in ("q", "Q"):
                print("\nCikiliyor...")
                break
            with data_lock:
                if key in ("c", "C"):
                    target_pan_deg, target_tilt_deg = 0.0, 0.0
                elif key == "LEFT":
                    target_pan_deg = max(PAN_MIN_DEG, target_pan_deg - step_deg)
                elif key == "RIGHT":
                    target_pan_deg = min(PAN_MAX_DEG, target_pan_deg + step_deg)
                elif key == "UP":
                    target_tilt_deg = min(TILT_MAX_DEG, target_tilt_deg + step_deg)
                elif key == "DOWN":
                    target_tilt_deg = max(TILT_MIN_DEG, target_tilt_deg - step_deg)
                elif key in ("+", "="):
                    step_deg = min(15.0, step_deg + 1.0)
                elif key in ("-", "_"):
                    step_deg = max(1.0, step_deg - 1.0)
                elif key in ("p", "P"):
                    threading.Thread(target=sweep, args=("pan",), daemon=True).start()
                elif key in ("t", "T"):
                    threading.Thread(target=sweep, args=("tilt",), daemon=True).start()
                else:
                    continue
            print_status()
    except KeyboardInterrupt:
        pass
    finally:
        is_running = False
        time.sleep(0.05)
        pan_servo.detach()
        tilt_servo.detach()
        print("\nServolar detach edildi, guvenli cikis.")


if __name__ == "__main__":
    main()
