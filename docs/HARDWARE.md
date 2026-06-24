# Hardware Reference

## Bill of Materials

| Component | Model | Notes |
|---|---|---|
| SBC | Raspberry Pi 5 (4 GB or 8 GB) | Pi 5 varsayılmıştır |
| Camera | Raspberry Pi Camera Module 3 | CSI ribbon ile bağlı |
| Pan servo | MG90S | Metal gear tercih edilir |
| Tilt servo | MG90S | Pan ile aynı sınıf |
| Laser | 5 mW 650 nm laser module (KY-008 veya benzeri) | Besleme varyanta göre değişebilir |
| Pan-tilt bracket | 2 eksenli mekanik braket | MG90S horn uyumu gerekli |
| Power supply | 5 V / 5 A USB-C | Servo yükünde pay bırakılmalı |

## GPIO Pinout (BCM numbering)

```text
 3V3  (1) (2)  5V
 SDA  (3) (4)  5V
 SCL  (5) (6)  GND
GPIO4 (7) (8)  GPIO14 ← LASER
 GND  (9) (10) GPIO15
GPIO17(11) (12) GPIO18    ← TRIGGER/RELAY on GPIO17
GPIO27(13) (14) GND
GPIO22(15) (16) GPIO23
 3V3 (17) (18) GPIO24
GPIO10(19) (20) GND
 GPIO9(21) (22) GPIO25
GPIO11(23) (24) GPIO8
 GND (25) (26) GPIO7
 SDA (27) (28) SCL
 GPIO5(29) (30) GND
 GPIO6(31) (32) GPIO12 ← PAN SERVO
GPIO13(33) (34) GND    ← TILT SERVO on GPIO13
GPIO19(35) (36) GPIO16
GPIO26(37) (38) GPIO20
 GND (39) (40) GPIO21
```

## Wiring Summary

| Signal | GPIO (BCM) | Physical Pin | Suggested colour |
|---|---|---|---|
| Pan servo signal | 12 | 32 | Yellow |
| Tilt servo signal | 13 | 33 | Yellow |
| Laser signal | 14 | 8 | Green |
| Trigger/relay | 17 | 11 | Blue |
| Shared ground | GND | 34, 6, 9… | Black |
| Servo / laser VCC | 5 V | 2 or 4 | Red |

Tüm topraklar ortak olmalıdır: Pi GND, servo besleme GND ve lazer GND.

## Servo convention used in this repo

- Kodda kullanılan pulse aralığı: `0.5–2.5 ms`
- Pan gösterge açısı: `-5°..5°`, `0° = merkez`
- Tilt gösterge açısı: `0°..10°`, `0° = fiziksel alt limit`
- Tilt dönüşümünde `TILT_ZERO_OFFSET = 25.0` uygulanır

`new2.py` içindeki `AngularServo(-90°..90°)` dönüşümleri:

```python
pan_servo_angle  = pan_deg
tilt_servo_angle = tilt_deg + 25.0 - 90.0
```

Bu dönüşüm, manuel araçlardaki gpiozero `-1..1` eşleşmesiyle elektriksel olarak aynıdır.

## Servo sürücü: Donanım PWM (Pi 5, önerilen)

MG90S jitter'ının başlıca yazılım kaynağı, lgpio/RPi.GPIO **yazılım PWM**'inin OS zamanlamasına bağlı darbe kaymasıdır. Pi 5'te `pigpio` **çalışmaz** (RP1 I/O çipi); doğru çözüm RP1'in **donanım PWM** kanallarıdır.

- Pan pini **GPIO12 = PWM0**, Tilt pini **GPIO13 = PWM1** — ikisi de donanım PWM kanalı.
- `new2.py` `USE_HARDWARE_PWM = True` iken `HardwarePWMServo` (kütüphane: `rpi-hardware-pwm`) kullanır; başlatılamazsa lgpio `AngularServo`'ya güvenli düşer.
- Açı→darbe eşlemesi AngularServo ile birebir aynıdır (−90°→500µs, 0°→1500µs, +90°→2500µs).
- Kanal eşlemesi: `PWM_CHIP=0` (kernel ≥6.12), `PAN_PWM_CHANNEL=0`, `TILT_PWM_CHANNEL=1`. Reboot sonrası eksenler ters/çalışmıyorsa yalnız bu sabitler değiştirilir.

### Gerekli sistem ayarı (bir kez, reboot ister)

1. `config.txt` overlay'i (GPIO12/13'ü PWM Alt0'a yönlendirir):
   ```
   dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4
   ```
2. `/sys/class/pwm` yazma izni için udev kuralı (`gpio` grubuna) — servis `heliosx` kullanıcısıyla, root olmadan çalıştığından gereklidir.
3. `pip install rpi-hardware-pwm` (sistem python3'üne).

> Not: Overlay aktifken GPIO12/13 PWM donanımına muxlanır; lgpio tabanlı manuel araçlar (`motor/manual-control*.py`) bu pinleri **artık süremez**.

## MG90S notes

| Parameter | Value |
|---|---|
| Operating voltage | 4.8–6.0 V |
| Stall torque | 1.8 kg·cm @ 4.8 V |
| Speed | 0.1 s / 60° @ 4.8 V |
| PWM frequency | 50 Hz |
| Vendor nominal pulse width | 1.0–2.0 ms |
| Angular range | ~180° |

Repo içindeki fiziksel kalibrasyon daha geniş pulse aralığı kullanır:

```python
min_pulse_width = 0.0005
max_pulse_width = 0.0025
```

## Laser module

- Control pin: GPIO 14
- `gpiozero.LED` olarak dijital aç/kapat sürülür
- GPIO yalnızca kontrol sinyaline gider
- Besleme hattı modül tipine göre 3.3 V veya 5 V olabilir
- **Class 2/3A laser — doğrudan göze tutulmamalı**

## Power notes

- Bench test sırasında Pi 5V hattı kullanılabilir
- Uzun süreli kullanımda harici 5V servo rail daha güvenlidir
- Servo ani akımı brownout üretebilir

## Driver requirement

Pi 5 üzerinde `gpiozero` importundan önce `lgpio` zorlanmalıdır:

```python
import os
os.environ["GPIOZERO_PIN_FACTORY"] = "lgpio"
```
