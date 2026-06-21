"""
Servo açı bulucu — hangi derece değeri hangi fiziksel konuma karşılık geliyor?

Kullanım:
  W/S  → Pan açısını artır/azalt
  A/D  → Tilt açısını artır/azalt
  +/-  → Adım büyüklüğünü değiştir (0.5 / 1 / 2 / 5 / 10)
  R    → Her iki servoya 0° (center) gönder
  Q    → Çıkış

Ekrandaki "servo_val" ve "pulse_ms" değerleri gpiozero'ya gönderilen sinyali gösterir.
Fiziksel açıyı gözlemle ve not al.
"""
import os
os.environ["GPIOZERO_PIN_FACTORY"] = "lgpio"

from gpiozero import Servo
import sys
import tty
import termios

STEPS = [0.5, 1.0, 2.0, 5.0, 10.0]
step_idx = 1  # başlangıç adımı: 1°

MIN_PW = 0.0005  # 0.5 ms — geniş aralık, klonlar genellikle destekler
MAX_PW = 0.0025  # 2.5 ms

pan_servo  = Servo(12, min_pulse_width=MIN_PW, max_pulse_width=MAX_PW)
tilt_servo = Servo(13, min_pulse_width=MIN_PW, max_pulse_width=MAX_PW)

# Tilt sıfır noktası: fiziksel alt limit = 0° olarak tanımlandı
# (eski 25° servo konumu artık göstergede 0° görünür)
TILT_ZERO_OFFSET = 25.0

pan_deg  = 0.0   # başlangıç: pan merkezi
tilt_deg = 0.0   # başlangıç: tilt alt limiti (fiziksel 25° → göstergede 0°)


def pan_val(deg: float) -> float:
    """–90°–+90° → –1.0–1.0"""
    return deg / 90.0


def tilt_val(deg: float) -> float:
    """Gösterge derecesini servo sinyaline çevirir (offset uygulanır)"""
    return ((deg + TILT_ZERO_OFFSET) / 90.0) - 1.0


def pulse_ms(servo_val: float) -> float:
    """gpiozero servo value → pulse genişliği (ms)"""
    return MIN_PW * 1000 + (MAX_PW - MIN_PW) * 1000 * (servo_val + 1) / 2


def send():
    pv = pan_val(pan_deg)
    tv = tilt_val(tilt_deg)
    pan_servo.value  = pv
    tilt_servo.value = tv
    step = STEPS[step_idx]
    print(
        f"\r  PAN  deg={pan_deg:>+7.1f}°  val={pv:>+5.3f}  pulse={pulse_ms(pv):.3f}ms"
        f"   ||   "
        f"TILT deg={tilt_deg:>6.1f}°  val={tv:>+5.3f}  pulse={pulse_ms(tv):.3f}ms"
        f"   [adım={step}°]   ",
        end="", flush=True
    )


def getch() -> str:
    fd  = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


# Başlangıç konumuna gönder
send()
print()
print("W/S=Pan  A/D=Tilt  +/-=Adım  R=Sıfırla  Q=Çıkış")
print("─" * 60)
send()

try:
    while True:
        ch = getch().lower()
        step = STEPS[step_idx]

        if ch == 'q':
            print("\nÇıkılıyor...")
            break
        elif ch == 'r':
            pan_deg  = 0.0
            tilt_deg = 90.0
        elif ch == 'w':
            pan_deg = max(-45.0, pan_deg - step)
        elif ch == 's':
            pan_deg = min( 45.0, pan_deg + step)
        elif ch == 'a':
            tilt_deg = max(  0.0, tilt_deg - step)
        elif ch == 'd':
            tilt_deg = min( 75.0, tilt_deg + step)
        elif ch == '+' or ch == '=':
            step_idx = min(len(STEPS) - 1, step_idx + 1)
        elif ch == '-':
            step_idx = max(0, step_idx - 1)
        else:
            continue

        send()

except KeyboardInterrupt:
    pass

finally:
    pan_servo.detach()
    tilt_servo.detach()
    print("\nServo'lar bırakıldı.")
