# Çalışma Zemini

Bu dosya repo üzerinde güvenli ve tutarlı çalışmak için tek referans noktasıdır.

## Source of truth

- Aktif uygulama: `v2/new2.py`
- Kalibrasyon verisi: `v2/calibration.json`
- Servis tanımı: `turret.service`
- Donanım gerçekleri: `docs/HARDWARE.md`
- Parametre ve tuning gerçekleri: `docs/TUNING.md`

`camera-stream/` ve `motor/` altındaki dosyalar faydalıdır ama ana davranışın kaynağı değildir.

## Değişiklik sınırları

- Yeni özellik veya bugfix önce `v2/new2.py` üzerinde değerlendirilir.
- Legacy/prototip dosyalara ancak açık ihtiyaç varsa dokunulur.
- Donanım davranışı değiştirilirse ilgili `.md` dosyası aynı turda güncellenir.
- Kalibrasyon formatı değişirse `v2/calibration.json` şeması belgelenir.

## Başlamadan önce kontrol listesi

1. `git status --short`
2. `README.md`, `CLAUDE.md`, `docs/WORKFLOW.md`
3. Gerekliyse `docs/TUNING.md` ve `docs/HARDWARE.md`
4. Değişiklik `v2/new2.py` içindeki mevcut davranışla doğrulanır

## Çalıştırma

```bash
cd ~/v2
python3 new2.py
```

Donanım yoksa:

```bash
cd ~/v2
python3 new2.py --dry-run --no-stream
```

Arayüzler:

- `/` canlı görüntü
- `/debug` mask ve adaylar
- `/calibrate` kalibrasyon arayüzü

## Doğrulama standardı

Bir değişiklikten sonra mümkün olan en az şu kontrol yapılır:

1. Uygulama açılıyor mu?
2. Stream endpoint cevap veriyor mu?
3. Debug görüntüsü geliyor mu?
4. Kalibrasyon akışı bozuldu mu?
5. Shutdown yolunda lazer kapatma ve servo `detach()` korunuyor mu?
6. Gerekirse `/status` snapshot'ında `mode`, `suppressed`, `aim_error_x/y`, `lock_visible_s` mantıklı mı?

## Mevcut teknik notlar

- Servo aralığı kodda `0.5–2.5 ms`
- Pan: `-5°..5°`, `0° = merkez`
- Tilt: `0°..10°`, `0° = fiziksel alt limit`
- Pi 5 için `GPIOZERO_PIN_FACTORY=lgpio` zorunlu
- Log dosyası `v2/detections.log`

## Bilinen dikkat noktaları

- Repo temiz değilse önce mevcut kullanıcı değişiklikleri korunur.
- Kalibrasyon çıktısı sadece var olmasıyla değil, kalite metriğiyle de değerlendirilmelidir.
- Otomatik kalibrasyonda yüksek RMS'li haritalar güvenilmez kabul edilir; yüklenmeden önce kalite kapısından geçmelidir.
- Legacy README'ler ana sistemin güncel davranışını temsil etmeyebilir.
