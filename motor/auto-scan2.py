import os
os.environ["GPIOZERO_PIN_FACTORY"] = "lgpio"

from gpiozero import Servo, LED
import threading
import time

# -------------------------------------------------------------
# DONANIM TANIMLAMALARI
# -------------------------------------------------------------
# MG90S için pulse genişlik ayarları
min_pw = 0.001
max_pw = 0.002

# Motorlar (GPIO 12 ve GPIO 13)
pan_servo = Servo(12, min_pulse_width=min_pw, max_pulse_width=max_pw)
tilt_servo = Servo(13, min_pulse_width=min_pw, max_pulse_width=max_pw)

# Lazer Modülü (GPIO 14 - Fiziksel 8. Pin)
lazer = LED(14)

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

# -------------------------------------------------------------
# THREAD 1: MOTOR KONTROL DÖNGÜSÜ (SMOOTH/INTERPOLATION)
# -------------------------------------------------------------
def motor_smooth_thread():
    """ Minimum gecikme ve yüksek akıcılık için arka plan motor sürücü döngüsü """
    global target_pan_deg, target_tilt_deg, current_pan_deg, current_tilt_deg, is_running
    
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
            
        time.sleep(0.005)

# -------------------------------------------------------------
# THREAD 2: LAZER KONTROL DÖNGÜSÜ (1 SANİYEDE BİR YAN-SÖN)
# -------------------------------------------------------------
def lazer_blink_thread():
    """ Motorlardan bağımsız olarak lazeri 1 saniyede bir yakıp söndürür """
    global is_running
    while is_running:
        lazer.on()   # Lazeri aç
        time.sleep(0.5) # 0.5 saniye açık kal
        lazer.off()  # Lazeri kapat
        time.sleep(0.5) # 0.5 saniye kapalı kal (Toplam döngü: 1 saniye)

# -------------------------------------------------------------
# ANA RUTİN VE HAREKET FONKSİYONLARI
# -------------------------------------------------------------
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
                
        time.sleep(0.012) # 1 derecelik fiziksel tarama hızı sınırı

# Thread'leri arka planda başlatıyoruz
t_motor = threading.Thread(target=motor_smooth_thread)
t_lazer = threading.Thread(target=lazer_blink_thread)

t_motor.start()
t_lazer.start()

try:
    print("=== Lazerli Otomatik Rutin Başlatılıyor ===")
    
    # 1. AŞAMA: ÖNCE TAMAMEN PAN (0 -> 180 -> 90)
    print("Aşama 1: Sadece Pan Hareketi Başladı (Lazer Flaşör Aktif)...")
    derece_derece_git("pan", 90, 0)
    time.sleep(0.2)
    derece_derece_git("pan", 0, 180)
    time.sleep(0.2)
    derece_derece_git("pan", 180, 90)
    time.sleep(0.3)
    
    # 2. AŞAMA: SONRA TAMAMEN TILT (0 -> 180 -> 90)
    print("Aşama 2: Sadece Tilt Hareketi Başladı...")
    derece_derece_git("tilt", 90, 0)
    time.sleep(0.2)
    derece_derece_git("tilt", 0, 180)
    time.sleep(0.2)
    derece_derece_git("tilt", 180, 90)
    time.sleep(0.5)
    
    # 3. AŞAMA: AYNI ANDA SENKRONİZE PAN VE TILT (0 -> 180)
    print("Aşama 3: Aynı Anda (Senkronize) 0'dan 180'e Geçiş...")
    with data_lock:
        target_pan_deg = 0.0
        target_tilt_deg = 0.0
    time.sleep(0.8)
    
    derece_derece_git("ikisi_de", 0, 180)
    time.sleep(0.5)
    
    print("Rutin başarıyla tamamlandı!")

except KeyboardInterrupt:
    print("\nKullanıcı tarafından durduruldu.")

finally:
    # Tüm sistemi kapat ve güvenliğe al
    is_running = False
    t_motor.join()
    t_lazer.join()
    
    # Donanımları kapat/serbest bırak
    lazer.off()
    pan_servo.detach()
    tilt_servo.detach()
    print("Lazer kapatıldı ve motor enerjileri kesildi. Güvenli çıkış.")
