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

# Paylaşılan değişkenler (Thread-safe)
data_lock = threading.Lock()
target_pan = 0.0    # Tuşlarla değiştireceğimiz HEDEF konum
target_tilt = 0.0   # Tuşlarla değiştireceğimiz HEDEF konum

current_pan = 0.0   # Motorun o anki FİZİKSEL konumu
current_tilt = 0.0  # Motorun o anki FİZİKSEL konumu

is_running = True

# Motorları başlangıçta merkeze al ve serbest bırak
pan_servo.value = 0.0
tilt_servo.value = 0.0
time.sleep(0.3)
pan_servo.detach()
tilt_servo.detach()

def motor_smooth_thread():
    """ Motorların hedefe yumuşak bir şekilde süzülmesini (interpolation)
        sağlayan arka plan döngüsü """
    global target_pan, target_tilt, current_pan, current_tilt, is_running
    
    # Yumuşaklık faktörü: Değer küçüldükçe hareket daha "soft" ve yavaş olur.
    # 0.05 ile 0.15 arası idealdir. Akıcılığı artırmak için 0.06 seçtik.
    smooth_factor = 0.06 
    
    pan_active = False
    tilt_active = False
    
    while is_running:
        with data_lock:
            t_pan = target_pan
            t_tilt = target_tilt
            
        # PAN için yumuşak geçiş hesaplama (Lerp mantığı)
        pan_diff = t_pan - current_pan
        if abs(pan_diff) > 0.01: # Eğer hedef ile mevcut konum arasında fark varsa
            current_pan += pan_diff * smooth_factor
            pan_servo.value = current_pan
            pan_active = True
        elif pan_active:
            # Hedefe tamamen varıldığında enerjiyi kes ki titremesin
            pan_servo.detach()
            pan_active = False
            
        # TILT için yumuşak geçiş hesaplama
        tilt_diff = t_tilt - current_tilt
        if abs(tilt_diff) > 0.01:
            current_tilt += tilt_diff * smooth_factor
            tilt_servo.value = current_tilt
            tilt_active = True
        elif tilt_active:
            tilt_servo.detach()
            tilt_active = False
            
        time.sleep(0.01) # Döngü her 10ms'de bir (100 Hz) çalışarak mükemmel akıcılık sağlar

def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

# Akıcı motor kontrol döngüsünü başlat
smooth_thread = threading.Thread(target=motor_smooth_thread)
smooth_thread.start()

print("=== Pan-Tilt Sinematik / Soft W-A-S-D Kontrolü ===")
print("A / D -> Pan  |  W / S -> Tilt  |  Q -> Çıkış")
print("-----------------------------------------------------")

# Adım miktarını biraz daha büyütebiliriz çünkü motor artık aniden zıplamayacak, 
# o hedefe doğru yumuşakça hızlanarak gidecek.
step = 0.2 

try:
    while True:
        char = getch().lower()
        
        if char == 'q':
            print("\nÇıkılıyor...")
            break
            
        with data_lock:
            if char == 'a':
                target_pan = min(1.0, target_pan + step)
                print(f"\r[HEDEF] Sola: {target_pan:.2f}     ", end="", flush=True)
            elif char == 'd':
                target_pan = max(-1.0, target_pan - step)
                print(f"\r[HEDEF] Sağa: {target_pan:.2f}     ", end="", flush=True)
            elif char == 'w':
                target_tilt = min(1.0, target_tilt + step)
                print(f"\r[HEDEF] Yukarı: {target_tilt:.2f}   ", end="", flush=True)
            elif char == 's':
                target_tilt = max(-1.0, target_tilt - step)
                print(f"\r[HEDEF] Aşağı: {target_tilt:.2f}    ", end="", flush=True)

except KeyboardInterrupt:
    pass

finally:
    is_running = False
    smooth_thread.join()
    pan_servo.detach()
    tilt_servo.detach()
    print("\nSistem kapatıldı.")
