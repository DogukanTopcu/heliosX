# P10 — Gece Modu / Adaptif Pozlama

**Öncelik:** Nice-to-Have  
**Etkilenen dosya:** `v2/new2.py`  
**Bağımlılık:** Yok (bağımsız değişiklik)

---

## Problem

Şu an kamera ayarları sabit:

```python
EXPOSURE_TIME_US = 5000    # 5ms — gündüz iç mekân için optimize
ANALOGUE_GAIN    = 6.0     # sabit kazanım
```

Gece veya düşük ışıkta:
- 5ms shutter çok kısa → görüntü çok karanlık → sinek blob'u kaybolur
- `AnalogueGain = 6.0` yeterli olmayabilir → gürültü artar
- CLAHE biraz kompanse eder ama sınırı var

Sabah/gündüz güçlü ışıkta:
- Aynı ayarlarla görüntü çok parlak gelebilir → `MAX_LOCAL_MEAN` çok tetiklenir

---

## Çözüm Yaklaşımı

İki seçenek:

### Seçenek A — Manuel Gece/Gündüz Toggle (Önerilen, Basit)

Web arayüzü (P8) veya sabit saatlere göre farklı profil kullan:

```python
# Sabitlere iki profil ekle
PROFILE_DAY = {
    "exposure_us": 5000,
    "gain":        6.0,
    "min_dark_score": 18.0,
    "max_local_mean": 200.0,
}
PROFILE_NIGHT = {
    "exposure_us": 15000,   # 15ms — daha uzun pozlama
    "gain":        10.0,    # daha yüksek kazanım
    "min_dark_score": 12.0, # düşük ışıkta dark_score düşer, eşiği indir
    "max_local_mean": 160.0,# düşük ışıkta parlak bölge daha nadir
}

ACTIVE_PROFILE = PROFILE_DAY  # başlangıç
```

### Seçenek B — Otomatik Adaptasyon

Her N frame'de frame ortalama parlaklığını ölç, kamera kontrollerini güncelle:

```python
BRIGHTNESS_TARGET = 80     # hedef ortalama parlaklık (0-255)
BRIGHTNESS_WINDOW = 90     # her 90 frame'de (3 sn) bir güncelle
EXPOSURE_MIN_US   = 3000
EXPOSURE_MAX_US   = 25000
GAIN_MIN          = 2.0
GAIN_MAX          = 12.0
```

---

## Implementasyon Adımları (Seçenek B — Otomatik)

### Adım 1 — Parlaklık ölçümü

Ana döngüde, her `BRIGHTNESS_WINDOW` frame'de:

```python
if frame_idx % BRIGHTNESS_WINDOW == 0 and frame_idx > WARMUP_FRAMES:
    mean_brightness = float(np.mean(gray))
    _adjust_exposure(picam2, mean_brightness)
```

### Adım 2 — Adaptasyon fonksiyonu

```python
_current_exposure = EXPOSURE_TIME_US
_current_gain     = ANALOGUE_GAIN

def _adjust_exposure(cam, brightness: float) -> None:
    global _current_exposure, _current_gain

    if not (EXPOSURE_TIME_US > 0):   # otomatik pozlama açıksa dokunma
        return

    error = BRIGHTNESS_TARGET - brightness   # pozitif = çok karanlık

    # Önce exposure'ı ayarla, gain son çare
    if error > 10:
        _current_exposure = min(_current_exposure * 1.15, EXPOSURE_MAX_US)
    elif error < -10:
        _current_exposure = max(_current_exposure * 0.87, EXPOSURE_MIN_US)

    # Exposure sınırına takıldıysa gain'e geç
    if _current_exposure >= EXPOSURE_MAX_US and error > 15:
        _current_gain = min(_current_gain * 1.1, GAIN_MAX)
    elif _current_exposure <= EXPOSURE_MIN_US and error < -15:
        _current_gain = max(_current_gain * 0.9, GAIN_MIN)

    cam.set_controls({
        "ExposureTime": int(_current_exposure),
        "AnalogueGain": _current_gain,
    })
    log(f"[EXPOSURE] mean={brightness:.0f} exp={_current_exposure:.0f}us gain={_current_gain:.1f}")
```

> **`picamera2.set_controls()`** çalışan kamera üzerinde anında güncelleme yapar, durdurup yeniden başlatmak gerekmez.

### Adım 3 — Log satırı ekle

`_adjust_exposure` fonksiyonu zaten loglama yapıyor. P2 tamamlanmışsa bu otomatik dosyaya yazılacak.

---

## Detection Parametrelerini Profille Bağlama

Gece modunda `MIN_DARK_SCORE` ve `MAX_LOCAL_MEAN` de ayarlanmalı. Seçenek A'da profil dict'i `main()` başında okunarak global sabitler gibi kullanılabilir:

```python
# main() başında
MIN_DARK_SCORE_ACTIVE = ACTIVE_PROFILE["min_dark_score"]
MAX_LOCAL_MEAN_ACTIVE = ACTIVE_PROFILE["max_local_mean"]
```

Ve detection pipeline içinde (`new2.py` satır 627-638) global sabitleri bu değişkenlerle değiştir.

---

## Riskler

- Exposure çok yüksek giderse (25ms × gain 12) görüntü doygunlaşır, sivrisinek blob'u kaybolur (paradoks). `BRIGHTNESS_TARGET = 80` makul bir tavan.
- `set_controls()` her çağrıda kameraya IPC gönderiyor. 90 frame'de bir = 3 saniyede bir → overhead ihmal edilebilir.
- Gece modunda gain artışı gürültüyü artırır. `MOTION_THRESHOLD`'u gece profilinde biraz yükselt (örn: 25) yoksa salt gürültü motion olarak algılanabilir.
- Seçenek B, Seçenek A'dan daha karmaşık. Eğer sistem sabit bir ışık ortamında kullanılacaksa Seçenek A yeterli.
