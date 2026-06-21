# Mosquito Laser Turret

Raspberry Pi 5 tabanlı, kamera ile küçük koyu hareketli hedefleri tespit edip pan-tilt kafa ve lazer ile takip eden deneysel sistem.

Bu repoda aktif çalışma alanı `v2/` dizinidir. `motor/` ve `camera-stream/` altındaki dosyalar destekleyici test ve legacy araçlarıdır.

## Şu anki durum

- Ana uygulama: `v2/new2.py`
- Canlı arayüz: `http://heliosx.local:8080/`
- Debug maskesi: `http://heliosx.local:8080/debug`
- Kalibrasyon arayüzü: `http://heliosx.local:8080/calibrate`
- Kalibrasyon dosyası: `v2/calibration.json`
- Log dosyası: `v2/detections.log`

## Repo haritası

```text
.
├── v2/
│   ├── new2.py
│   ├── fly_detect.py
│   ├── calibration.json
│   └── README.md
├── motor/
├── camera-stream/
├── docs/
│   ├── WORKFLOW.md
│   ├── HARDWARE.md
│   ├── TUNING.md
│   └── plans/
├── turret.service
└── CLAUDE.md
```

## Hızlı başlangıç

### Bağımlılıklar

```bash
sudo apt update
sudo apt install -y python3-picamera2 python3-opencv python3-numpy python3-gpiozero python3-lgpio
```

### Tam sistemi çalıştırma

```bash
cd ~/v2
python3 new2.py
```

Donanım olmadan kuru çalışma:

```bash
cd ~/v2
python3 new2.py --dry-run --no-stream
```

Tarayıcıdan:

- `http://heliosx.local:8080/`
- `http://heliosx.local:8080/debug`
- `http://heliosx.local:8080/calibrate`

### Servo testleri

```bash
cd ~/motor
python3 manual-control5.py
python3 servo-angle-finder.py
```

### Legacy detector

```bash
cd ~/camera-stream/fly-detector
./fly-detector.sh status
```

## Sistem özeti

```text
Camera -> Picamera2 -> 1280x720 capture
                   -> 640x360 process frame
                   -> motion + blackhat + adaptive masks
                   -> contour/score filters
                   -> track association
                   -> lock / laser / servo control
                   -> MJPEG stream + calibration endpoints
```

## Ana özellikler

- CPU-only tespit hattı
- Pi 5 için `lgpio` tabanlı servo/lazer kontrolü
- Adaptif pozlama mantığı
- MJPEG stream ve debug görüntüsü
- Web tabanlı manuel ve otomatik kalibrasyon akışı
- Döner log dosyası (`RotatingFileHandler`)

## Kaynak dokümanlar

- Genel çalışma zemini: [docs/WORKFLOW.md](docs/WORKFLOW.md)
- Donanım ve pin eşleşmeleri: [docs/HARDWARE.md](docs/HARDWARE.md)
- Tuning ve semptom/fix tablosu: [docs/TUNING.md](docs/TUNING.md)
- Kod ajanı için repo kuralları: [CLAUDE.md](CLAUDE.md)
- `v2` odaklı çalışma notları: [v2/README.md](v2/README.md)

## Güvenlik

Lazerli otomasyon gerçek donanım etkisi üretir. İnsan, hayvan, yansıtıcı yüzey ve kapalı alan güvenliği doğrulanmadan sistemi çalıştırma.
