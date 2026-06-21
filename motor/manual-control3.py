import os
os.environ["GPIOZERO_PIN_FACTORY"] = "lgpio"

from gpiozero import Servo
import sys
import tty
import termios
import threading
import time

# MG90S için pulse genişlik ayarları
min_pw = 0.001
max_pw = 0.002

# Motor tanımlamaları
pan_servo = Servo(12, min_pulse_width=min_pw, max_pulse_width=max_pw)
tilt_servo = Servo(13, min_pulse_width=min_pw, max_pulse_width=max_pw)

# Paylaşılan değişkenler ve Thread kilidi (Lock)
data_lock = threading.Lock()
pan_pos = 0.0
tilt_pos = 0.0
is_running = True
last_moved_time = time.time()

# Motorları başlangıçta merkeze al
pan_servo.value = pan_pos
tilt_servo.value = tilt_pos
time.sleep(0.3)
pan_servo.detach()
tilt_servo.detach()

def motor_control_thread():
    """ Motorların hareketini ve durduğunda enerjisinin kesilmesini
        arka planda bağımsız olarak yöneten fonksiyon """
    global pan_pos, tilt_pos, is_running, last_moved_time
    
    last_pan = None
    last_tilt = None
    
    while is_running:
        with data_lock:
            current_pan = pan_pos
            current_tilt = tilt_pos
            current_last_moved = last_moved_time
            
        # Eğer pozisyon değiştiyse motora yeni değeri gönder (Enerji verilir)
        if current_pan != last_pan:
            pan_servo.value = current_pan
            last_pan = current_pan
            
        if current_tilt != last_tilt:
            tilt_servo.value = current_tilt
            last_tilt = current_tilt
            
        # Eğer son hareketin üzerinden 0.4 saniye geçtiyse ve motorlar hala aktifse
        # titremeyi önlemek için enerjiyi kes (detach et)
        if time.time() - current_last_moved > 0.4:
            pan_servo.detach()
            tilt_servo.detach()
            # Değerleri sıfırlayarak bir sonraki tuş basımında tetiklenmesini sağla
            last_pan = None
            last_tilt = None
            
        time.sleep(0.02) # Arka plan döngüsünü rahatlatmak için 20ms bekleme

def getch():
    """ SSH üzerinden gecikmesiz karakter okuma """
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

# Motor döngüsünü arka planda başlatıyoruz (Gecikmeyi sıfıra indirir)
motor_thread = threading.Thread(target=motor_control_thread)
motor_thread.start()

print("=== Pan-Tilt W-A-S-D Gecikmesiz & Akıcı Kontrol ===")
print("A / D -> Pan  |  W / S -> Tilt  |  Q -> Çıkış")
print("-----------------------------------------------------")

step = 0.1 # Hareket mesafesi (Az gelirse 0.15 yapabilirsin)

try:
    while True:
        char = getch().lower()
        
        if char == 'q':
            print("\nÇıkılıyor...")
            break
            
        with data_lock:
            if char == 'a':
                pan_pos = min(1.0, pan_pos + step)
                last_moved_time = time.time()
                print(f"\r[PAN] Sola: {pan_pos:.2f}     ", end="", flush=True)
            elif char == 'd':
                pan_pos = max(-1.0, pan_pos - step)
                last_moved_time = time.time()
                print(f"\r[PAN] Sağa: {pan_pos:.2f}     ", end="", flush=True)
            elif char == 'w':
                tilt_pos = min(1.0, tilt_pos + step)
                last_moved_time = time.time()
                print(f"\r[TILT] Yukarı: {tilt_pos:.2f}   ", end="", flush=True)
            elif char == 's':
                tilt_pos = max(-1.0, tilt_pos - step)
                last_moved_time = time.time()
                print(f"\r[TILT] Aşağı: {tilt_pos:.2f}    ", end="", flush=True)

except KeyboardInterrupt:
    pass

finally:
    # Thread'i ve motorları güvenli kapatma
    is_running = False
    motor_thread.join()
    pan_servo.detach()
    tilt_servo.detach()
    print("\nSistem ve motorlar güvenli bir şekilde kapatıldı.")
