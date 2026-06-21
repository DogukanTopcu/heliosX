import os
os.environ["GPIOZERO_PIN_FACTORY"] = "lgpio"

from gpiozero import Servo
import sys
import tty
import termios

# MG90S için pulse genişlik ayarları
min_pw = 0.001
max_pw = 0.002

# Motor tanımlamaları
pan_servo = Servo(12, min_pulse_width=min_pw, max_pulse_width=max_pw)
tilt_servo = Servo(13, min_pulse_width=min_pw, max_pulse_width=max_pw)

# Başlangıç pozisyonları
pan_pos = 0.0
tilt_pos = 0.0
step = 0.1  # Tepkiyi hızlandırmak için adımı 0.15 veya 0.2 de yapabilirsin

# İlk başta merkeze al ve titremeyi önlemek için hemen bırak
pan_servo.value = pan_pos
tilt_servo.value = tilt_pos
pan_servo.detach()
tilt_servo.detach()

print("=== Pan-Tilt W-A-S-D Gecikmesiz & Titremesiz Kontrol ===")
print("A / D -> Pan  |  W / S -> Tilt  |  Q -> Çıkış")
print("-----------------------------------------------------")

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
        moved = False
        
        if char == 'a':
            pan_pos = min(1.0, pan_pos + step)
            pan_servo.value = pan_pos  # Motora enerji ver ve döndür
            moved = True
            print(f"\r[PAN] Sola: {pan_pos:.2f}     ", end="", flush=True)
            
        elif char == 'd':
            pan_pos = max(-1.0, pan_pos - step)
            pan_servo.value = pan_pos
            moved = True
            print(f"\r[PAN] Sağa: {pan_pos:.2f}     ", end="", flush=True)
            
        elif char == 'w':
            tilt_pos = min(1.0, tilt_pos + step)
            tilt_servo.value = tilt_pos
            moved = True
            print(f"\r[TILT] Yukarı: {tilt_pos:.2f}   ", end="", flush=True)
            
        elif char == 's':
            tilt_pos = max(-1.0, tilt_pos - step)
            tilt_servo.value = tilt_pos
            moved = True
            print(f"\r[TILT] Aşağı: {tilt_pos:.2f}    ", end="", flush=True)
            
        elif char == 'q':
            print("\nÇıkılıyor...")
            break

        # Eğer hareket bittiyse, motorun titremesini engellemek için enerjiyi kes
        if moved:
            # Motorun fiziksel olarak hedefe varması için çok kısa bir süre tanıyoruz
            # (Eğer takılma yaparsa buradaki pas geçme mantığı titremeyi önler)
            pan_servo.detach()
            tilt_servo.detach()

except KeyboardInterrupt:
    pass

finally:
    pan_servo.detach()
    tilt_servo.detach()
    print("\nSistem kapatıldı.")
