import os
# Gpiozero'ya pigpio aramayı bırakıp doğrudan yerel lgpio sürücüsünü kullanmasını söylüyoruz
os.environ["GPIOZERO_PIN_FACTORY"] = "lgpio"

from gpiozero import Servo
from time import sleep

# MG90S için güvenli mikro saniye (pulse width) aralıkları
# Eğer motor tam dönmezse min_pw=0.0005, max_pw=0.0025 olarak esnetebilirsin
min_pw = 0.001
max_pw = 0.002

# Motorları tanımlıyoruz (GPIO 12 ve GPIO 13)
pan_servo = Servo(12, min_pulse_width=min_pw, max_pulse_width=max_pw)
tilt_servo = Servo(13, min_pulse_width=min_pw, max_pulse_width=max_pw)

try:
    print("Sürücü başarıyla yüklendi. Motorlar test ediliyor...")
    while True:
        print("Merkez konumu...")
        pan_servo.mid()
        tilt_servo.mid()
        sleep(2)

        print("Maksimum konum...")
        pan_servo.max()
        tilt_servo.max()
        sleep(2)

        print("Minimum konum...")
        pan_servo.min()
        tilt_servo.min()
        sleep(2)

except KeyboardInterrupt:
    pan_servo.detach()
    tilt_servo.detach()
    print("\nProgram kullanıcı tarafından sonlandırıldı.")
