# Detection Tuning Guide

Ana parametreler `v2/new2.py` başındaki sabitler bloğundadır. Bir seferde tek değişken oynat ve sonucu `http://heliosx.local:8080/debug` üzerinden izle.

Kalibrasyon akışı ayrı bir konudur; `http://heliosx.local:8080/calibrate` arayüzünü de kullan.

## Workflow

1. `python3 ~/v2/new2.py`
2. `http://heliosx.local:8080/debug` aç
3. Gerekirse `http://heliosx.local:8080/calibrate` aç
4. Tek parametre değiştir, restart et, tekrar bak

## Related runtime surfaces

- Main view: `http://heliosx.local:8080/`
- Debug view: `http://heliosx.local:8080/debug`
- Calibration UI: `http://heliosx.local:8080/calibrate`
- Log file: `~/v2/detections.log`

## Camera Parameters

| Constant | Default | Tuning guidance |
|---|---|---|
| `CAPTURE_W/H` | 1280×720 | Ana capture çözünürlüğü |
| `PROCESS_W/H` | 640×360 | Detection çözünürlüğü; düşürmek CPU kazandırır |
| `TARGET_FPS` | 30 | 30 pratik üst sınırdır |
| `EXPOSURE_TIME_US` | 5000 µs | Düşürmek hareketi dondurur, yükseltmek karanlıkta görünürlük sağlar |
| `ANALOGUE_GAIN` | 6.0 | Fazla yükselirse noise artar |

## Adaptive exposure

Bu blok loş veya değişken ışıkta shutter/gain güncellemesi yapar. Motion-diff temelli pipeline için agresif poz değişimi sahneyi bozabilir.

| Constant | Default | Effect |
|---|---|---|
| `BRIGHTNESS_TARGET` | 80 | Hedef ortalama parlaklık |
| `BRIGHTNESS_WINDOW` | 90 | Kaç frame'de bir pozlama güncellenecek |
| `EXPOSURE_MIN_US` | 3000 | Alt shutter sınırı |
| `EXPOSURE_MAX_US` | 25000 | Üst shutter sınırı |
| `GAIN_MIN` | 2.0 | Alt gain sınırı |
| `GAIN_MAX` | 12.0 | Üst gain sınırı |

## Preprocessing

| Constant | Default | Effect |
|---|---|---|
| `BLUR_KERNEL` | 5 | Yüksek değer küçük hedefi silebilir |
| `CLAHE_CLIP` | 2.0 | Yükselirse lokal kontrast artar |
| `CLAHE_GRID` | (8, 8) | Daha küçük grid daha lokal davranır |

## Motion Mask

| Constant | Default | Effect |
|---|---|---|
| `MOTION_THRESHOLD` | 20 | Düşerse hassasiyet artar, false positive de artar |
| `MOTION_DILATE` | 2 | Hızlı hedef etrafındaki boşlukları doldurur |

## Dark Blob Mask

| Constant | Default | Effect |
|---|---|---|
| `BLACKHAT_KERNEL` | 17 | Büyük kernel motion blur'lü küçük hedeflerde daha toleranslıdır |
| `BLACK_THRESHOLD` | 18 | Düşerse daha çok aday çıkar |

## Adaptive Threshold

| Constant | Default | Effect |
|---|---|---|
| `ADAPTIVE_BLOCK` | 21 | Odd olmalı |
| `ADAPTIVE_C` | 9 | Yükselirse şart sıkılaşır |

## Blob Size and Shape Gates

| Constant | Default | Tuning guidance |
|---|---|---|
| `MIN_AREA` | 8 px² | Uzak küçük hedefler kaçıyorsa düşür |
| `MAX_AREA` | 200 px² | Yakın büyük hedefler kaçıyorsa artır |
| `MIN_WH` | 2 px | Çok küçük bbox filtrelemesi |
| `MAX_WH` | 22 px | Fazla büyük bbox filtrelemesi |
| `MIN_ASPECT` | 0.25 | Aşırı ince blobları keser |
| `MAX_ASPECT` | 4.0 | Aşırı yatay blobları keser |

## Per-Detection Scoring Gates

| Constant | Default | Effect |
|---|---|---|
| `MIN_MOTION_SCORE` | 0.12 | Bbox içindeki motion oranı |
| `MIN_DARK_SCORE` | 18.0 | Bbox içindeki blackhat yoğunluğu |
| `MAX_LOCAL_MEAN` | 200.0 | Çok parlak lokal bölgeleri reddeder |

## Global Motion Suppression

| Constant | Default | Effect |
|---|---|---|
| `MAX_GLOBAL_MOTION_RATIO` | 0.04 | Frame'in büyük kısmı hareket ediyorsa tüm frame bastırılır |
| `MAX_TOTAL_DETECTIONS` | 25 | Kaotik sahnelerde güvenlik supabı |

## Exclusion Zone

| Constant | Default | Effect |
|---|---|---|
| `LARGE_MOTION_AREA` | 4000 px | Büyük hareket bileşenini insan/parmak adayı sayar |
| `LARGE_MOTION_DIM` | 110 px | Genişlik/yükseklik tabanlı büyük nesne kontrolü |
| `EXCLUSION_PADDING` | 20 px | Büyük bölge çevresine güvenlik boşluğu |
| `EXCLUSION_DILATE_ITERS` | 4 | Motion birleşimini güçlendirir |
| `EXCLUSION_DILATE_KERNEL` | 5 px | Dilation kernel boyutu |

## Tracking Parameters

| Constant | Default | Effect |
|---|---|---|
| `MATCH_DISTANCE` | 90 px | Frame'ler arası eşleştirme yarıçapı |
| `MAX_MISSED` | 12 frames | Track ömrü |
| `MIN_HITS` | 2 | Lock öncesi minimum eşleşme |
| `CONFIRM_FRAMES` | 1 | Ek yaş filtresi |
| `TRAJECTORY_WINDOW` | 10 | Geçmiş pencere uzunluğu |
| `MIN_PATH_LENGTH` | 4.0 px | Statik titreşimi elemek için minimum yol |
| `MIN_DIR_CHANGES` | 0 | İstenirse yön değişimi şartı eklenebilir |

## Servo Tracking

| Constant | Default | Effect |
|---|---|---|
| `KP_BASE` | 0.020 | Küçük hatalarda hassas takip |
| `KP_BOOST` | 0.060 | Büyük hatalarda hızlı yakalama |
| `KP_BOOST_ERROR_PX` | 80.0 | Boost etkisinin doygunlaştığı hata büyüklüğü |
| `smooth_factor` | 0.22 | Servo thread içindeki lerp oranı |

## Calibration notes

- Dosya: `~/v2/calibration.json`
- Açılışta otomatik yüklenir
- Web UI manuel ve otomatik kalibrasyon modları içerir
- Polynomial fit kullanılıyorsa kalite `rms_pan_deg` / `rms_tilt_deg` ile okunur
- Dosyanın var olması doğru kalibrasyon anlamına gelmez
- Kötü otomatik kalibrasyon haritası artık reddedilir; pratik kabul eşiği yaklaşık `n_samples >= 12` ve `rms_pan_deg`, `rms_tilt_deg <= 6`
- Yalnız RMS değil, örneklerin görüntüde yeterince yayılmış olması da gerekir; dar kümelenmiş örnekler reddedilir

## Symptom → Likely Fix

| Symptom | Try |
|---|---|
| Too many false positives | Raise `MOTION_THRESHOLD`, raise `MIN_MOTION_SCORE`, raise `MIN_PATH_LENGTH` |
| Mosquito not detected | Lower `MIN_AREA`, lower `BLACK_THRESHOLD`, lower `MIN_DARK_SCORE`, tune exposure |
| Hand triggers detection | Lower `LARGE_MOTION_AREA`, lower `LARGE_MOTION_DIM`, lower `MAX_GLOBAL_MOTION_RATIO` |
| Tracks die too quickly | Raise `MAX_MISSED`, raise `MATCH_DISTANCE` |
| Servo oscillates | Lower `KP_BASE` / `KP_BOOST`, lower `smooth_factor` |
| Servo too slow | Raise `smooth_factor`, raise `KP_BOOST` |
| Exposure hunting | Raise `BRIGHTNESS_WINDOW`, narrow exposure/gain range |
| High CPU / low FPS | Lower `PROCESS_W/H`, lower `STREAM_QUALITY`, reduce overlays |
