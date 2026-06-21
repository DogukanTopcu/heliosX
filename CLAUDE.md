# CLAUDE.md — Çalışma Kuralları

Bu dosya repo üzerinde çalışan kod ajanı için kısa operasyon kılavuzudur.

## Öncelik sırası

1. Kod gerçekliği: `v2/new2.py`
2. Operasyon kılavuzu: `docs/WORKFLOW.md`
3. Donanım gerçekliği: `docs/HARDWARE.md`
4. Parametre gerçekliği: `docs/TUNING.md`
5. Legacy/prototip dosyalar: yalnızca referans

## Aktif yüzeyler

- Ana uygulama: `v2/new2.py`
- Stream: `/`, `/stream.mjpg`
- Debug: `/debug`, `/debug.mjpg`
- Kalibrasyon: `/calibrate` ve ilgili endpoint'ler
- Servis: `turret.service`

## Değişiklik prensipleri

- Ana davranış `v2/new2.py` dışında kopyalanmaz.
- Donanım davranışı veya tuning değişirse ilgili `.md` dosyası aynı turda güncellenir.
- Legacy dosyada bir düzeltme gerekiyorsa neden ayrı tutulduğu not edilir.
- Kullanıcının mevcut çalışma ağacındaki değişiklikleri korunur.

## Doğrulanmış donanım gerçekleri

- Pan servo: GPIO 12
- Tilt servo: GPIO 13
- Lazer: GPIO 14
- Trigger/relay: GPIO 17
- GPIO backend: `lgpio`
- Servo pulse aralığı: `0.5–2.5 ms`
- Pan aralığı: `-45°..45°`, `0° = merkez`
- Tilt aralığı: `0°..75°`, `0° = fiziksel alt limit`

## Kod gerçekleri

- Capture: `1280x720`
- Process: `640x360`
- Log: `v2/detections.log`
- Kalibrasyon: `v2/calibration.json`
- Stream portu: `8080`
- Servo komutları yalnız `motor_smooth_thread()` üzerinden gider

## Değişiklikten önce bakılacaklar

- `git status --short`
- `README.md`
- `docs/WORKFLOW.md`
- İlgiliyse `docs/TUNING.md` ve `docs/HARDWARE.md`

## Kırmızı çizgiler

1. Lazer kapatma yolu bozulmayacak
2. Çıkışta servo `detach()` korunacak
3. `data_lock` altında sleep veya I/O yapılmayacak
4. Servo kontrolü yan yollardan yazılmayacak
5. Calibration dosya formatı sessizce değiştirilmeyecek

## Şu anki pratik not

Bu repo aktif geliştirme halinde. Özellikle `v2/new2.py` ve `v2/calibration.json` üzerinde yerel değişiklikler olabilir; yeni iş bu zemini ezmeden ilerlemeli.
