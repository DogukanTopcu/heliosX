# P5 — Systemd Servis / Otomatik Başlatma

**Öncelik:** Orta  
**Etkilenen dosya:** Yeni dosya: `/etc/systemd/system/turret.service`  
**Bağımlılık:** P1–P4 tamamlanmış olmalı (sistem stabil olsun)

---

## Problem

Şu an sistemi başlatmak için SSH ile bağlanıp `python3 v2/new2.py` çalıştırmak gerekiyor.  
Pi yeniden başladığında sistem otomatik olarak devreye girmiyor.  
Uzun süreli kurulum (oda köşesinde sabit montaj) için bu yetersiz.

---

## Çözüm Yaklaşımı

`systemd` user servisi olarak tanımla. Root servisi değil — `heliosx` kullanıcısı olarak çalışsın. Kamera ve GPIO erişimi `video` ve `gpio` grupları üzerinden sağlanıyor, bu gruplar zaten `heliosx` kullanıcısına atanmış olmalı.

### Adım 1 — Grup üyeliğini kontrol et

```bash
groups heliosx
# Çıktıda: video gpio i2c spi görünmeli
# Eksikse: sudo usermod -aG gpio,video heliosx && reboot
```

### Adım 2 — Servis dosyasını oluştur

```bash
sudo nano /etc/systemd/system/turret.service
```

İçeriği:

```ini
[Unit]
Description=Mosquito Laser Turret
After=network.target

[Service]
Type=simple
User=heliosx
WorkingDirectory=/home/heliosx/v2
ExecStart=/usr/bin/python3 /home/heliosx/v2/new2.py
Restart=on-failure
RestartSec=5s
StandardOutput=journal
StandardError=journal
# Güvenli kapatma için SIGTERM gönder, 10 saniye bekle, SIGKILL
TimeoutStopSec=10

[Install]
WantedBy=multi-user.target
```

> **Neden `After=network.target`?** MJPEG stream'i network'e bağımlı. Network hazır olmadan başlarsa stream hata vermez ama log'da `[Errno 98] Address already in use` yerine temiz çalışır.

### Adım 3 — Servisi etkinleştir

```bash
sudo systemctl daemon-reload
sudo systemctl enable turret.service
sudo systemctl start turret.service
```

### Adım 4 — Durum kontrolü

```bash
sudo systemctl status turret.service
# ● turret.service - Mosquito Laser Turret
#    Loaded: loaded (/etc/systemd/system/turret.service; enabled)
#    Active: active (running) ...
```

Log'ları izle:
```bash
journalctl -u turret.service -f
```

---

## Yararlı Komutlar

```bash
# Servisi durdur (lazer güvenli kapatma bloğu devreye girer)
sudo systemctl stop turret.service

# Yeniden başlat (kod değişikliği sonrası)
sudo systemctl restart turret.service

# Otomatik başlatmayı kapat
sudo systemctl disable turret.service

# Son 100 log satırı
journalctl -u turret.service -n 100 --no-pager
```

---

## Kamera Erişim Sorunu (Potansiyel)

Pi 5 + Picamera2 bazen `libcamera` kilidini ilk başlatmada tutabiliyor. Servis yeniden başlatılırsa önceki instance tam kapanmadan yeni instance açılabilir — `TimeoutStopSec=10` bunu önler.

Yine de sorun yaşanırsa servis dosyasına ekle:

```ini
ExecStartPre=/bin/sleep 2
```

---

## GPIO Güvenliği

`new2.py`'nin `finally` bloğu (`satır 822-836`) SIGTERM aldığında da çalışır:

```python
finally:
    is_running = False
    t_motor.join()
    if HAS_GPIO:
        lazer.off()
        pan_servo.detach()
        tilt_servo.detach()
```

`systemctl stop` → SIGTERM → Python finally → lazer off, servo detach → `TimeoutStopSec` içinde temiz kapanış.  
Manuel `kill -9` verilirse finally çalışmaz; lazer açık kalabilir. Bu yüzden asla `kill -9` kullanma.

---

## Test

1. `sudo systemctl start turret.service`
2. `http://heliosx.local:8080/` → stream görünüyor mu?
3. `sudo reboot`
4. Pi açıldıktan 30 saniye sonra stream tekrar görünüyor mu?
5. `sudo systemctl stop turret.service` → lazer kapalı, servoları elle hareket ettirince jitter yok mu?
