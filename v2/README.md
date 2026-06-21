# v2

Bu dizin aktif kod tabanıdır.

## Ana dosyalar

- `new2.py` — ana uygulama
- `fly_detect.py` — detection-only varyant
- `new.py` — ara prototip
- `calibration.json` — runtime kalibrasyon verisi

## Gereksinimler

```bash
sudo apt update
sudo apt install -y python3-picamera2 python3-opencv python3-numpy python3-gpiozero python3-lgpio
```

## Çalıştırma

```bash
cd ~/v2
python3 new2.py
```

## Dry-run

Donanım veya Picamera2 olmadan mantık akışını çalıştırmak için:

```bash
cd ~/v2
python3 new2.py --dry-run --no-stream
```

İstersen stream'i açık bırak:

```bash
python3 new2.py --dry-run --stream-port 8091
```

## Endpoint'ler

- `http://heliosx.local:8080/` — ana görüntü
- `http://heliosx.local:8080/debug` — debug mask
- `http://heliosx.local:8080/calibrate` — kalibrasyon arayüzü

## Davranış özeti

- 1280×720 capture
- 640×360 detection
- Motion + blackhat + adaptive threshold birleşimi
- Track association ve trajectory gate
- Adaptif servo gain
- Web tabanlı kalibrasyon
- Döner log dosyası

## Loglar

- Uygulama logu: `~/v2/detections.log`
- Kalibrasyon dosyası: `~/v2/calibration.json`

## Notlar

- Ana geliştirme hedefi `new2.py` olmalı
- `fly_detect.py` masaüstü/servo'suz denemeler için daha uygun
- Kalibrasyon dosyası elle düzenlenebilir ama tercih edilen yol web arayüzüdür
