# Implementasyon Planları — Genel İndeks

Bu dizin, mosquito laser turret projesinin geliştirme yol haritasını içerir.
Her plan kendi `.md` dosyasında ayrıntılı olarak açıklanmıştır.

## Not

Bu klasör artık ağırlıklı olarak tarihsel tasarım kaydıdır. Gerçek mevcut davranış için önce `v2/new2.py`, ardından `docs/WORKFLOW.md`, `docs/TUNING.md` ve `docs/HARDWARE.md` okunmalıdır.

## Öncelik Sırası

### Kritik — Sistemi gerçekten çalıştırmak için şart

| # | Plan | Dosya | Durum |
|---|---|---|---|
| P1 | Lazer-Kamera Offset (kod değişikliği + config yükleme) | [P1-laser-offset.md](P1-laser-offset.md) | Tamamlandı |
| P2 | Log Dosyasına Yazma (Kod Var, Çalışmıyor) | [P2-log-fix.md](P2-log-fix.md) | Tamamlandı |
| P3 | FPS Hesabı Sliding Window ile Düzeltme | [P3-fps-fix.md](P3-fps-fix.md) | Tamamlandı |
| P4 | Servo Mekanik Güvenlik Sınırları | [P4-servo-limits.md](P4-servo-limits.md) | Tamamlandı |
| P11 | Kalibrasyon Sistemi (interaktif, tekrarlanabilir, kalıcı) | [P11-calibration-system.md](P11-calibration-system.md) | Tamamlandı |

### Orta Öncelik — Uzun vadeli kullanım için önemli

| # | Plan | Dosya | Durum |
|---|---|---|---|
| P5 | Systemd Servis / Otomatik Başlatma | [P5-systemd.md](P5-systemd.md) | Tamamlandı |
| P6 | Çoklu Hedef Önceliklendirme | [P6-multi-target.md](P6-multi-target.md) | Tamamlandı |
| P7 | Adaptif Servo Kazanımı (Mesafeye Göre KP) | [P7-adaptive-gain.md](P7-adaptive-gain.md) | Tamamlandı |

### Nice-to-Have — Sistem olgunlaştıkça

| # | Plan | Dosya | Durum |
|---|---|---|---|
| P8 | Web Kontrol Paneli | [P8-web-control.md](P8-web-control.md) | Tamamlandı |
| P9 | Kalman Filtresi ile Gelişmiş Tahmin | [P9-kalman.md](P9-kalman.md) | Tamamlandı |
| P10 | Gece Modu / Adaptif Pozlama | [P10-night-mode.md](P10-night-mode.md) | Tamamlandı |

## Plan Bağımlılık Sırası

```text
P1 → P11 → P8
P4 → P7
P5, P6, P9, P10 bağımsız uygulanabilir
```

## Uygulama prensibi

- Planlar karar geçmişi sağlar; kod ile çelişirse kod kazanır.
- Yeni işte önce tarihsel plan değil mevcut runtime davranışı doğrulanır.
- Tüm değişiklikler mümkünse `v2/new2.py` üzerinde toplanır.
