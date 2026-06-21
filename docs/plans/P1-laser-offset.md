# P1 — Lazer-Kamera Offset Kalibrasyonu

**Öncelik:** Kritik  
**Etkilenen dosya:** `v2/new2.py`  
**Bağımlılık:** Yok (ilk yapılmalı)

---

## Problem

Şu an servo kontrolü şu varsayıma dayanıyor:

```python
# new2.py satır 692-693
error_x = tr.cx - (PROCESS_W / 2)   # 320 piksel merkez
error_y = tr.cy - (PROCESS_H / 2)   # 180 piksel merkez
```

Bu kod, lazer ışınının kamera merkeziyle tam olarak aynı noktayı gösterdiğini varsayıyor.  
Oysa kamera ve lazer modülü braket üzerinde fiziksel olarak farklı konumlarda — aralarında sabit bir açısal fark var.

**Sonuç:** Sistem sineği doğru takip ediyor gibi görünse de lazer ona isabet etmiyor.

---

## Mevcut Durum Analizi

`new2.py` içinde offset için hiçbir sabit yok. `TUNING.md`'de şu not var:
> "Calibrate the offset between camera centre and laser dot after assembly"

Ama bunu destekleyen kod eksik.

Servo kontrol satırları (`new2.py` satır 699-701):
```python
with data_lock:
    target_pan_deg = max(0.0, min(180.0, target_pan_deg - (error_x * kp_x)))
    target_tilt_deg = max(0.0, min(180.0, target_tilt_deg - (error_y * kp_y)))
```

---

## Çözüm Yaklaşımı

> **Kritik not:** Offset'i `target_pan_deg`'e eklemek yanlış. Servo kontrol kodu bir integratör gibi çalışıyor: hata sıfırlanana kadar her frame `target_pan_deg`'i biraz kaydırıyor. Oraya offset eklenirse her frame'de `offset` değeri birikir → servo saniyede `offset × 30` derece kayar ve duvara çarpar. Doğru yaklaşım: "sineğin gitmesi gereken hedef piksel"i kaydırmak, yani offset'i error hesabına uygulamak.

### Adım 1 — Sabit bloğuna offset değerleri ekle

`new2.py` satır 109-119 arasındaki sabitler bloğuna şunu ekle:

```python
# Lazer-Kamera fiziksel offset (kalibrasyon gerektiriyor)
# Birim: piksel (process çözünürlüğünde, 640×360)
# Pozitif LASER_OFFSET_PX_X: lazer, kamera merkezinin sağında görünüyor
# Pozitif LASER_OFFSET_PX_Y: lazer, kamera merkezinin altında görünüyor
LASER_OFFSET_PX_X = 0.0   # <-- kalibrasyon sonrası doldurulacak
LASER_OFFSET_PX_Y = 0.0   # <-- kalibrasyon sonrası doldurulacak
```

### Adım 2 — Error hesabını güncelle

`new2.py` satır 692-693'teki error hesabını değiştir:

**Önce:**
```python
error_x = tr.cx - (PROCESS_W / 2)
error_y = tr.cy - (PROCESS_H / 2)
```

**Sonra:**
```python
# Hedef piksel: kamera merkezi değil, lazerin düştüğü nokta
error_x = tr.cx - (PROCESS_W / 2 + LASER_OFFSET_PX_X)
error_y = tr.cy - (PROCESS_H / 2 + LASER_OFFSET_PX_Y)
```

> **Neden bu şekilde?** Servo kontrolü "error → 0 ol" diye çalışıyor. Error tanımını değiştirerek "0" noktasını lazer konumuna taşıyoruz. Bu şekilde offset her frame'de yalnızca bir kez (error hesabında) etkili oluyor, birikmez.

---

## Adım 3 — Startup'ta kalibrasyon dosyasını yükle

`LASER_OFFSET_PX_X/Y` sabitleri artık kod içinde sabit değil — `v2/calibration.json` dosyasından okunacak. Bu sayede sistem yeniden başlatılsa bile kalibrasyon korunur ve web'den yeniden kalibrasyon yapılabilir.

`new2.py`'nin `main()` fonksiyonunun başına (donanım başlatmadan önce) şunu ekle:

```python
_load_calibration()   # calibration.json varsa LASER_OFFSET_PX_X/Y'yi günceller
```

Fonksiyon tanımı (P11'de detaylandırılıyor):

```python
import json as _json

CALIB_PATH = "/home/heliosx/v2/calibration.json"

def _load_calibration() -> None:
    global LASER_OFFSET_PX_X, LASER_OFFSET_PX_Y
    if not os.path.exists(CALIB_PATH):
        log("Kalibrasyon dosyası yok, offset=(0,0) kullanılıyor.")
        return
    try:
        with open(CALIB_PATH) as f:
            data = _json.load(f)
        LASER_OFFSET_PX_X = float(data.get("laser_offset_px_x", 0.0))
        LASER_OFFSET_PX_Y = float(data.get("laser_offset_px_y", 0.0))
        log(f"Kalibrasyon yüklendi: offset=({LASER_OFFSET_PX_X:.1f}, {LASER_OFFSET_PX_Y:.1f})px "
            f"[{data.get('calibrated_at', '?')}]")
    except Exception as e:
        log(f"Kalibrasyon dosyası okunamadı: {e} — offset=(0,0) kullanılıyor.")
```

> Kalibrasyon prosedürünün tamamı (web'den tetikleme, interaktif akış, dosyaya kayıt, parallax tartışması) **[P11](P11-calibration-system.md)** planında.

---

## Test

1. `v2/calibration.json` dosyası yokken sistemi başlat → log'da "Kalibrasyon dosyası yok" görmeli
2. P11 prosedürüyle kalibre et → `calibration.json` oluştu mu?
3. Sistemi yeniden başlat → "Kalibrasyon yüklendi: offset=(X, Y)px" logu görmeli
4. Statik bir hedefe kilitle → lazer üstüne düşüyor mu?

---

## Riskler

- Offset piksel cinsindendir; parallax nedeniyle kalibrasyon mesafesinde doğru, farklı mesafelerde hata büyür. Detay ve pratik çözüm için **P11**'e bak.
- Braket sökülüp takılırsa kalibrasyonun yenilenmesi gerekir — web'den P11 akışıyla yapılır.
