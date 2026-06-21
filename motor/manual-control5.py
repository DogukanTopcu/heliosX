import os
os.environ["GPIOZERO_PIN_FACTORY"] = "lgpio"

from gpiozero import Servo
import sys
import tty
import termios
import threading
import time

min_pw = 0.0005  # 0.5 ms — geniş aralık
max_pw = 0.0025  # 2.5 ms

pan_servo  = Servo(12, min_pulse_width=min_pw, max_pulse_width=max_pw)
tilt_servo = Servo(13, min_pulse_width=min_pw, max_pulse_width=max_pw)

# -------------------------------------------------------------
# AYARLAR
# -------------------------------------------------------------
DERECE_ADIMI = 1  # Her tuş basışında kaç derece

# Tilt sıfır noktası: fiziksel alt limit = göstergede 0° olarak tanımlandı
TILT_ZERO_OFFSET = 25.0

# Pan: fiziksel açı skalası (0°=merkez, + bir yön, - diğer yön)
PAN_MIN_DEG  = -45.0
PAN_MAX_DEG  =  45.0

# Tilt: 0° = fiziksel alt limit, yukarı artar
TILT_MIN_DEG =   0.0
TILT_MAX_DEG =  75.0

# Başlangıç pozisyonları
target_pan_deg  =  0.0
target_tilt_deg =  0.0

current_pan_deg  =  0.0
current_tilt_deg =  0.0
# -------------------------------------------------------------

is_running = True
data_lock  = threading.Lock()


def pan_to_servo_val(deg: float) -> float:
    """–90°–+90° fiziksel açı → –1.0–1.0  (0°=merkez=1.5 ms)"""
    return deg / 90.0


def tilt_to_servo_val(deg: float) -> float:
    """Gösterge derecesini servo sinyaline çevirir (offset uygulanır)"""
    return ((deg + TILT_ZERO_OFFSET) / 90.0) - 1.0


def motor_smooth_thread() -> None:
    global current_pan_deg, current_tilt_deg, is_running

    # 0.5: her ilk hamlede ~0.5° gönderir → MG90S ölü bölgesini aşar, titreme yok
    smooth_factor = 0.5
    threshold     = 0.05   # °, bu altı fark → servo komutu gönderme

    while is_running:
        with data_lock:
            t_pan  = target_pan_deg
            t_tilt = target_tilt_deg

        pan_diff = t_pan - current_pan_deg
        if abs(pan_diff) > threshold:
            current_pan_deg += pan_diff * smooth_factor
            pan_servo.value  = pan_to_servo_val(current_pan_deg)

        tilt_diff = t_tilt - current_tilt_deg
        if abs(tilt_diff) > threshold:
            current_tilt_deg += tilt_diff * smooth_factor
            tilt_servo.value  = tilt_to_servo_val(current_tilt_deg)

        time.sleep(0.005)   # 200 Hz


def getch() -> str:
    fd  = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def print_status() -> None:
    print(f"\r  PAN: {target_pan_deg:>+6.1f}°  |  TILT: {target_tilt_deg:>6.1f}°    ",
          end="", flush=True)


# Başlangıç pozisyonuna getir
pan_servo.value  = pan_to_servo_val(target_pan_deg)
tilt_servo.value = tilt_to_servo_val(target_tilt_deg)
time.sleep(0.5)

t = threading.Thread(target=motor_smooth_thread, daemon=True)
t.start()

print(f"=== Pan-Tilt {DERECE_ADIMI}° Hassasiyetli Kontrol ===")
print("A / D → Pan (Sol / Sağ)   |   W / S → Tilt (Yukarı / Aşağı)")
print("C → Merkeze Dön   |   Q → Çıkış")
print(f"Pan: {PAN_MIN_DEG:+.0f}°–{PAN_MAX_DEG:+.0f}°   |   Tilt: {TILT_MIN_DEG:.0f}°–{TILT_MAX_DEG:.0f}°")
print("-------------------------------------------------------------------")
print_status()

try:
    while True:
        char = getch().lower()

        if char == 'q':
            print("\nÇıkılıyor...")
            break

        with data_lock:
            if char == 'c':
                target_pan_deg  =  0.0
                target_tilt_deg =  0.0
            elif char == 'a':
                target_pan_deg = min(PAN_MAX_DEG, target_pan_deg + DERECE_ADIMI)
            elif char == 'd':
                target_pan_deg = max(PAN_MIN_DEG, target_pan_deg - DERECE_ADIMI)
            elif char == 'w':
                target_tilt_deg = min(TILT_MAX_DEG, target_tilt_deg + DERECE_ADIMI)
            elif char == 's':
                target_tilt_deg = max(TILT_MIN_DEG, target_tilt_deg - DERECE_ADIMI)
            else:
                continue

        print_status()

except KeyboardInterrupt:
    pass

finally:
    is_running = False
    pan_servo.detach()
    tilt_servo.detach()
    print("\nSistem güvenli biçimde kapatıldı.")
