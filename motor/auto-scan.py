import os
os.environ["GPIOZERO_PIN_FACTORY"] = "lgpio"

from gpiozero import Servo
import threading
import time

# MG90S için pulse genişlik ayarları
min_pw = 0.001
max_pw = 0.002

# Motor tanımlamaları
pan_servo = Servo(12, min_pulse_width=min_pw, max_pulse_width=max_pw)
tilt_servo = Servo(13, min_pulse_width=min_pw, max_pulse_width=max_pw)

# Paylaşılan Değişkenler
data_lock = threading.Lock()
target_pan_deg = 90.0
target_tilt_deg = 90.0
current_pan_deg = 90.0
current_tilt_deg = 90.0
is_running = True

def deg_to_servo_val(deg):
    """ 0-180 dereceyi -1.0 ile 1.0 skalasına çevirir """
    return (deg / 90.0) - 1.0

# İlk başta merkeze al
pan_servo.value = deg_to_servo_val(90)
tilt_servo.value = deg_to_servo_val(90)
time.sleep(0.5)

def motor_smooth_thread():
    """ Minimum gecikme ve yüksek akıcılık için arka plan motor sürücü döngüsü """
    global target_pan_deg, target_tilt_deg, current_pan_deg, current_tilt_deg, is_running
    
    # Hızlı ve akıcı takip için faktörü 0.25'e çıkardık (Gecikmeyi minimize eder)
    smooth_factor = 0.25 
    
    while is_running:
        with data_lock:
            t_pan = target_pan_deg
            t_tilt = target_tilt_deg
            
        # PAN Kontrolü
        pan_diff = t_pan - current_pan_deg
        if abs(pan_diff) > 0.05:
            current_pan_deg += pan_diff * smooth_factor
            pan_servo.value = deg_to_servo_val(current_pan_deg)
        else:
            current_pan_deg = t_pan
            pan_servo.value = deg_to_servo_val(current_pan_deg)
            
        # TILT Kontrolü
        tilt_diff = t_tilt - current_tilt_deg
        if abs(tilt_diff) > 0.05:
            current_tilt_deg += tilt_diff * smooth_factor
            tilt_servo.value = deg_to_servo_val(current_tilt_deg)
        else:
            current_tilt_deg = t_tilt
            tilt_servo.value = deg_to_servo_val(current_tilt_deg)
            
        time.sleep(0.005) # 5ms döngü hızı (Minimum gecikme için ultra hızlı)

# Sürücü thread'ini başlat
smooth_thread = threading.Thread(target=motor_smooth_thread)
smooth_thread.start()

def derece_derece_git(eksen, baslangic, bitis):
    """ Belirtilen ekseni 1'er derece adımlarla hedefe götürür """
    global target_pan_deg, target_tilt_deg
    
    adim = 1 if bitis > baslangic else -1
    
    for aci in range(int(baslangic), int(bitis) + adim, adim):
        with data_lock:
            if eksen == "pan":
                target_pan_deg = float(aci)
            elif eksen == "tilt":
                target_tilt_deg = float(aci)
            elif eksen == "ikisi_de":
                target_pan_deg = float(aci)
                target_tilt_deg = float(aci)
                
        # 1 derecelik fiziksel hareketi tamamlaması için motora gereken çok kısa süre
        # Gecikmeyi minimumda tutmak için 12ms (0.012) idealdir.
        time.sleep(0.012) 

try:
    print("=== Otomatik Rutin Başlatılıyor (Girdisiz) ===")
    
    # -------------------------------------------------------------
    # 1. AŞAMA: ÖNCE TAMAMEN PAN (0 -> 180 -> 90)
    # -------------------------------------------------------------
    print("Aşama 1: Sadece Pan Hareketi Başladı...")
    derece_derece_git("pan", 90, 0)     # Merkezden 0'a
    time.sleep(0.2)
    derece_derece_git("pan", 0, 180)    # 0'dan tamamen 180'e
    time.sleep(0.2)
    derece_derece_git("pan", 180, 90)   # Tekrar merkeze dön
    time.sleep(0.3)
    
    # -------------------------------------------------------------
    # 2. AŞAMA: SONRA TAMAMEN TILT (0 -> 180 -> 90)
    # -------------------------------------------------------------
    print("Aşama 2: Sadece Tilt Hareketi Başladı...")
    derece_derece_git("tilt", 90, 0)    # Merkezden 0'a
    time.sleep(0.2)
    derece_derece_git("tilt", 0, 180)   # 0'dan tamamen 180'e
    time.sleep(0.2)
    derece_derece_git("tilt", 180, 90)  # Tekrar merkeze dön
    time.sleep(0.5)
    
    # -------------------------------------------------------------
    # 3. AŞAMA: AYNI ANDA SENKRONİZE PAN VE TILT (0 -> 180)
    # -------------------------------------------------------------
    print("Aşama 3: Aynı Anda (Senkronize) 0'dan 180'e Geçiş...")
    # Önce ikisini birden hızlıca başlangıç noktası olan 0'a çekiyoruz
    with data_lock:
        target_pan_deg = 0.0
        target_tilt_deg = 0.0
    time.sleep(0.8) # Başlangıç noktasına varmaları için bekleme
    
    # Şimdi aynı anda 1'er derece artarak 180'e gidiyorlar
    derece_derece_git("ikisi_de", 0, 180)
    time.sleep(0.5)
    
    print("Rutin başarıyla tamamlandı!")

except KeyboardInterrupt:
    print("\nKullanıcı tarafından durduruldu.")

finally:
    # Sistemi kapat ve motorları serbest bırak
    is_running = False
    smooth_thread.join()
    pan_servo.detach()
    tilt_servo.detach()
    print("Motor enerjileri kesildi. Güvenli çıkış.")
