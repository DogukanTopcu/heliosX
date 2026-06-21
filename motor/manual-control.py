import os
# Raspberry Pi 5 için lgpio sürücüsünü zorunlu kılıyoruz
os.environ["GPIOZERO_PIN_FACTORY"] = "lgpio"

from gpiozero import Servo
import sys
import tty
import termios

# MG90S için pulse genişlik ayarları
min_pw = 0.001
max_pw = 0.002

# Motor tanımlamaları (GPIO 12 ve 13)
pan_servo = Servo(12, min_pulse_width=min_pw, max_pulse_width=max_pw)
tilt_servo = Servo(13, min_pulse_width=min_pw, max_pulse_width=max_pw)

# Başlangıç pozisyonları (Tam merkez: 0.0)
pan_pos = 0.0
tilt_pos = 0.0
step = 0.1 

# Motorları başlangıçta merkeze gönderiyoruz
pan_servo.value = pan_pos
tilt_servo.value = tilt_pos

print("=== Pan-Tilt W-A-S-D SSH Kontrol Programı ===")
print("A / D -> Pan (Sağ / Sol)")
print("W / S -> Tilt (Yukarı / Aşağı)")
print("Q     -> Çıkış")
print("--------------------------------------------")

# Terminalden Enter'a basmadan anlık tuş okumak için fonksiyon
def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

try:
    while True:
        char = getch().lower()
        
        if char == 'a': # Sola dön
            pan_pos = min(1.0, pan_pos + step)
            pan_servo.value = pan_pos
            print(f"\r[PAN] Sola -> Pozisyon: {pan_pos:.2f}", end="")
            
        elif char == 'd': # Sağa dön
            pan_pos = max(-1.0, pan_pos - step)
            pan_servo.value = pan_pos
            print(f"\r[PAN] Sağa -> Pozisyon: {pan_pos:.2f}", end="")
            
        elif char == 'w': # Yukarı bak
            tilt_pos = min(1.0, tilt_pos + step)
            tilt_servo.value = tilt_pos
            print(f"\r[TILT] Yukarı -> Pozisyon: {tilt_pos:.2f}", end="")
            
        elif char == 's': # Aşağı bak
            tilt_pos = max(-1.0, tilt_pos - step)
            tilt_servo.value = tilt_pos
            print(f"\r[TILT] Aşağı -> Pozisyon: {tilt_pos:.2f}", end="")
            
        elif char == 'q': # Çıkış
            print("\nProgramdan çıkılıyor...")
            break

except KeyboardInterrupt:
    pass

finally:
    # Motorları serbest bırakıyoruz
    pan_servo.detach()
    tilt_servo.detach()
    print("\nMotorlar kapatıldı.")
