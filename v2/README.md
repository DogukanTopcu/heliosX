# v2 — Fly Detection (motion-based) + MJPEG Stream

Bagimsiz prototip. Mevcut `~/camera-stream/` projesine dokunmaz.

## Gereksinimler
```bash
sudo apt update
sudo apt install -y python3-picamera2 python3-opencv python3-numpy python3-gpiozero
```

## Calistirma
```bash
cd ~/v2
python3 fly_detect.py
```

## PC'den izleme
Pi calisirken, PC'nin tarayicisindan:

    http://heliosx.local:8080/

Ayni agda olmasi yeterli. Tarayici acilinca canli MJPEG stream ve uzerinde:
- Kirmizi daire: hareketli sinek adayi (tetikleme yapilan)
- Sari daire: sabit/yavas aday (henuz tetiklemiyor)
- Sol ust: FPS ve aday sayisi
- "FLY" yazisi: aktif tespit

Loglar `~/v2/detections.log` dosyasinda.

## Arka planda calistirma
```bash
cd ~/v2
nohup python3 fly_detect.py > /dev/null 2>&1 &
```
Durdurmak icin: `pkill -f "v2/fly_detect.py"`

## Port degistirme
`fly_detect.py` icinde `STREAM_PORT = 8080`. Mevcut `camera-stream` baska bir portu kullaniyor olabilir, cakisma olursa burayi degistir (orn. 8090).

## Kalibrasyon
- `MIN_AREA` / `MAX_AREA`: kameradan sinegin piksel alani
- `MIN_SPEED`: frame'ler arasi minimum piksel kaymasi
- `TRIGGER_COOLDOWN`: arka arkaya tetiklemeyi engeller
- `TRIGGER_PIN`: GPIO BCM numarasi (varsayilan 17)
- `STREAM_QUALITY`: 1-100 arasi JPEG kalitesi (dusur -> ag/CPU rahatlar)

Streamde "aday" sayisi ve sahnedeki gercek sinekleri karsilastirarak esikleri ayarla.
