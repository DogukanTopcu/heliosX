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

# -------------------------------------------------------------
# HASSASİYET VE DERECE AYARLARI
# -------------------------------------------------------------
# Tuşa her bastığında motorun tam olarak kaç derece dönmesini istiyorsun?
# En yüksek hassasiyet için 1 yapabilirsin. (1, 2, 3, 4... derece)
DERECE_ADIMI = 1  

# Başlangıç açıları (Tam merkez: 90 derece)
target_pan_deg = 90.0
target_tilt_deg = 90.0

current_pan_deg = 90.0
current_tilt_deg = 90.0

is_running = True
data_lock = threading.Lock()

def deg_to_servo_val(deg):
    """ 0-180 derece arasındaki açıyı, gpiozero'nun -1.0 ile 1.0 skalasına çevirir """
    return (deg / 90.0) - 1.0

# Motorları başlangıçta 90 dereceye (merkeze) al ve serbest bırak
pan_servo.value = deg_to_servo_val(90)
tilt_servo.value = deg_to_servo_val(90)
time.sleep(0.3)
pan_servo.detach()
tilt_servo.detach()

def motor_smooth_thread():
    """ Motorların hedefe mikro adımlarla, pürüzsüzce süzülmesini sağlayan döngü """
    global target_pan_deg, target_tilt_deg, current_pan_deg, current_tilt_deg, is_running
    
    # Geçiş yumuşaklığı (Lerp faktörü). 
    # Çok hassas kontrol için 0.12 yaptık; hem akıcı hem de hedef dereceye çok hızlı oturur.
    smooth_factor = 0.12 
    
    pan_active = False
    tilt_active = False
    
    while is_running:
        with data_lock:
            t_pan = target_pan_deg
            t_tilt = target_tilt_deg
            
        # PAN (Sağ-Sol) Kontrolü
        pan_diff = t_pan - current_pan_deg
        if abs(pan_diff) > 0.05: # 0.05 derecelik farkları bile yakala (Ultra hassas)
            current_pan_deg += pan_diff * smooth_factor
            pan_servo.value = deg_to_servo_val(current_pan_deg)
            pan_active = True
        elif pan_active:
            pan_servo.detach() # Tam hedefe oturduğunda titremeyi kes
            pan_active = False
            
        # TILT (Yukarı-Aşağı) Kontrolü
        tilt_diff = t_tilt - current_tilt_deg
        if abs(tilt_diff) > 0.05:
            current_tilt_deg += tilt_diff * smooth_factor
            tilt_servo.value = deg_to_servo_val(current_tilt_deg)
            tilt_active = True
        elif tilt_active:
            tilt_servo.detach()
            tilt_active = False
            
        time.sleep(0.01) # 10ms döngü hızı

def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

# Hassas kontrol thread'ini başlat
smooth_thread = threading.Thread(target=motor_smooth_thread)
smooth_thread.start()

print(f"=== Pan-Tilt {DERECE_ADIMI}° Hassasiyetli Kontrol Programı ===")
print("A / D -> Pan (Sağ / Sol)  |  W / S -> Tilt (Yukarı / Aşağı)  |  Q -> Çıkış")
print(f"Her basışta motorlar tam {DERECE_ADIMI} derece dönecektir.")
print("---------------------------------------------------------------------")

try:
    while True:
        char = getch().lower()
        
        if char == 'q':
            print("\nÇıkılıyor...")
            break
            
        with data_lock:
            if char == 'a': # Sola (Dereceyi artır/azalt yönüne göre ayarla, gerekirse yönü değiştir)
                target_pan_deg = min(180.0, target_pan_deg + DERECE_ADIMI)
                print(f"\r[PAN HEDEF] {target_pan_deg:.0f}°    ", end="", flush=True)
            elif char == 'd': # Sağa
                target_pan_deg = max(0.0, target_pan_deg - DERECE_ADIMI)
                print(f"\r[PAN HEDEF] {target_pan_deg:.0f}°    ", end="", flush=True)
            elif char == 'w': # Yukarı
                target_tilt_deg = min(180.0, target_tilt_deg + DERECE_ADIMI)
                print(f"\r[TILT HEDEF] {target_tilt_deg:.0f}°   ", end="", flush=True)
            elif char == 's': # Aşağı
                target_tilt_deg = max(0.0, target_tilt_deg - DERECE_ADIMI)
                print(f"\r[TILT HEDEF] {target_tilt_deg:.0f}°   ", end="", flush=True)

except KeyboardInterrupt:
    pass

finally:
    is_running = False
    smooth_thread.join()
    pan_servo.detach()
    tilt_servo.detach()
    print("\nSistem güvenli bir şekilde kapatıldı.")
