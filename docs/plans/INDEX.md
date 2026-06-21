# Implementasyon Planları — Genel İndeks

Bu dizin, mosquito laser turret projesinin geliştirme yol haritasını içerir.
Her plan kendi `.md` dosyasında ayrıntılı olarak açıklanmıştır.

## Öncelik Sırası

### Kritik — Sistemi gerçekten çalıştırmak için şart

| # | Plan | Dosya | Durum |
|---|---|---|---|
| P1 | Lazer-Kamera Offset (kod değişikliği + config yükleme) | [P1-laser-offset.md](P1-laser-offset.md) | Bekliyor |
| P2 | Log Dosyasına Yazma (Kod Var, Çalışmıyor) | [P2-log-fix.md](P2-log-fix.md) | Bekliyor |
| P3 | FPS Hesabı Sliding Window ile Düzeltme | [P3-fps-fix.md](P3-fps-fix.md) | Bekliyor |
| P4 | Servo Mekanik Güvenlik Sınırları | [P4-servo-limits.md](P4-servo-limits.md) | Bekliyor |
| P11 | Kalibrasyon Sistemi (interaktif, tekrarlanabilir, kalıcı) | [P11-calibration-system.md](P11-calibration-system.md) | Bekliyor |

### Orta Öncelik — Uzun vadeli kullanım için önemli

| # | Plan | Dosya | Durum |
|---|---|---|---|
| P5 | Systemd Servis / Otomatik Başlatma | [P5-systemd.md](P5-systemd.md) | Bekliyor |
| P6 | Çoklu Hedef Önceliklendirme | [P6-multi-target.md](P6-multi-target.md) | Bekliyor |
| P7 | Adaptif Servo Kazanımı (Mesafeye Göre KP) | [P7-adaptive-gain.md](P7-adaptive-gain.md) | Bekliyor |

### Nice-to-Have — Sistem olgunlaştıkça

| # | Plan | Dosya | Durum |
|---|---|---|---|
| P8 | Web Kontrol Paneli | [P8-web-control.md](P8-web-control.md) | Bekliyor |
| P9 | Kalman Filtresi ile Gelişmiş Tahmin | [P9-kalman.md](P9-kalman.md) | Bekliyor |
| P10 | Gece Modu / Adaptif Pozlama | [P10-night-mode.md](P10-night-mode.md) | Bekliyor |

## Plan Bağımlılık Sırası

```
P1 → P11 → P8   (offset kodu → kalibrasyon akışı → web entegrasyonu)
P2              (bağımsız)
P3              (bağımsız)
P4 → P7         (servo sınırları → adaptif gain)
P1 + P4 → P7   (her ikisi de gerekli)
P5              (bağımsız, P1-P4 stabil olduktan sonra)
P6              (bağımsız)
P9              (bağımsız, en karmaşık)
P10             (bağımsız)
```

## Uygulama Prensibi

- Her planı implement etmeden önce ilgili `.md` dosyasını oku.
- Bir adımı bitirdiğinde INDEX.md'deki durumu "Tamamlandı" olarak güncelle.
- Her plan bağımsız şekilde uygulanabilir; sıranın önemi olmadığı durumlarda not edilmiştir.
- Tüm değişiklikler `v2/new2.py` üzerinde yapılır — prototip dosyaları (`new.py`, `fly_detect.py`) dokunulmadan kalır.
