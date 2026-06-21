# P7 — Adaptif Servo Kazanımı (Mesafeye Göre KP)

**Öncelik:** Orta  
**Etkilenen dosya:** `v2/new2.py`  
**Bağımlılık:** P1 (offset kalibrasyonu) + P4 (servo sınırları) tamamlanmış olmalı

---

## Problem

`new2.py` satır 695-701:

```python
kp_x = 0.040
kp_y = 0.040

with data_lock:
    target_pan_deg = max(..., target_pan_deg - (error_x * kp_x))
    target_tilt_deg = max(..., target_tilt_deg - (error_y * kp_y))
```

`KP = 0.040` sabit. Bu şu anlama gelir:
- `error_x = 100 px` → servo 4° kayar
- `error_x = 10 px`  → servo 0.4° kayar (orantılı, iyi)

Ama sorun farklı bir yerde: **kazanımın hız ve mesafe tutarsızlığı.**

- Sinek hızla hareket ediyorsa (büyük error) → küçük KP tepkisi yavaş kalır, sinek kaçar
- Sinek merkeze çok yakınsa (küçük error) → aynı KP ile sürekli salınım (oscillation) riski

**İdeal davranış:** Hata büyükse hızlı tepki (büyük KP), hata küçükse hassas ince ayar (küçük KP). Bu "proportional + feed-forward" veya basit "kazanım çizelgesi" ile sağlanır.

---

## Çözüm Yaklaşımı

Sabit KP yerine **dinamik kazanım** kullan. İki yaklaşım sunulmuştur; biri seçilecek.

### Yaklaşım A — Lineer Ölçekleme (Önerilen, Basit)

Hata büyüdükçe KP büyür, küçüldükçe KP küçülür:

```python
BASE_KP   = 0.020   # küçük hatalarda kullanılan temel kazanım
BOOST_KP  = 0.060   # büyük hatalarda kullanılan maksimum kazanım
BOOST_ERR = 80.0    # bu piksel hatasından büyüklerde BOOST_KP kullanılır

def adaptive_kp(error_px: float) -> float:
    t = min(abs(error_px) / BOOST_ERR, 1.0)   # 0-1 arası lineer interpolasyon
    return BASE_KP + t * (BOOST_KP - BASE_KP)
```

Kullanım (`new2.py` satır 695-701):

```python
# P1 offset error'a uygulanıyor
error_x = tr.cx - (PROCESS_W / 2 + LASER_OFFSET_PX_X)
error_y = tr.cy - (PROCESS_H / 2 + LASER_OFFSET_PX_Y)
kp_x = adaptive_kp(error_x)
kp_y = adaptive_kp(error_y)

# P4 sabitleri ile clamp
with data_lock:
    target_pan_deg  = max(PAN_MIN_DEG,  min(PAN_MAX_DEG,
                          target_pan_deg  - (error_x * kp_x)))
    target_tilt_deg = max(TILT_MIN_DEG, min(TILT_MAX_DEG,
                          target_tilt_deg - (error_y * kp_y)))
```

### Yaklaşım B — Blob Alanından Mesafe Tahmini

Sinek yakına gelirse blob alanı büyür → KP'yi düşür (hassas mod):

```python
NEAR_AREA_THRESHOLD = 80    # px² — bu alandan büyükse sinek yakında
KP_NEAR = 0.020             # yakın menzil: hassas
KP_FAR  = 0.055             # uzak menzil: hızlı

def distance_kp(blob_area: float) -> float:
    t = min(blob_area / NEAR_AREA_THRESHOLD, 1.0)
    return KP_FAR - t * (KP_FAR - KP_NEAR)   # uzakta büyük, yakında küçük
```

> **Öneri:** Yaklaşım A ile başla. Blob alanı tabanlı tahmin (B) çalışabilir ama gürültülü — sineğin alanı frame'e göre büyük sapma gösterir.

---

## Implementasyon Adımları (Yaklaşım A)

### Adım 1 — Sabitleri ekle

`new2.py` sabitler bloğuna (`KP_X = KP_Y = 0.040` yerine):

```python
# Adaptif servo kazanımı
KP_BASE  = 0.020   # küçük hata — hassas ince ayar modu
KP_BOOST = 0.060   # büyük hata — hızlı yakalama modu
KP_BOOST_ERROR_PX = 80.0   # bu piksel hatasında BOOST_KP'ye geç
```

### Adım 2 — `adaptive_kp` fonksiyonunu ekle

`deg_to_servo_val` fonksiyonundan sonra (satır 144):

```python
def adaptive_kp(error_px: float) -> float:
    t = min(abs(error_px) / KP_BOOST_ERROR_PX, 1.0)
    return KP_BASE + t * (KP_BOOST - KP_BASE)
```

### Adım 3 — Servo kontrol bloğunu güncelle

`new2.py` satır 691-701 (lock altındaki tracking):

**Önce:**
```python
error_x = tr.cx - (PROCESS_W / 2)
error_y = tr.cy - (PROCESS_H / 2)
kp_x = 0.040
kp_y = 0.040
with data_lock:
    target_pan_deg  = max(0.0, min(180.0, target_pan_deg  - (error_x * kp_x)))
    target_tilt_deg = max(0.0, min(180.0, target_tilt_deg - (error_y * kp_y)))
```

**Sonra:**
```python
# P1'den: offset error hesabına uygulanıyor (target_pan_deg'e değil)
error_x = tr.cx - (PROCESS_W / 2 + LASER_OFFSET_PX_X)
error_y = tr.cy - (PROCESS_H / 2 + LASER_OFFSET_PX_Y)
kp_x = adaptive_kp(error_x)
kp_y = adaptive_kp(error_y)
# P4'ten: PAN_MIN_DEG / PAN_MAX_DEG sabitleri gerekiyor
with data_lock:
    target_pan_deg  = max(PAN_MIN_DEG,  min(PAN_MAX_DEG,
                          target_pan_deg  - (error_x * kp_x)))
    target_tilt_deg = max(TILT_MIN_DEG, min(TILT_MAX_DEG,
                          target_tilt_deg - (error_y * kp_y)))
```

---

## Parametre Ayarı

Oscillation (salınım) gözlenirse:
- `KP_BOOST` değerini düşür (örn: 0.060 → 0.045)
- `smooth_factor` (motor thread'de 0.22) düşürülebilir

Sinek kaçıyor (tepki çok yavaş):
- `KP_BOOST` değerini artır (örn: 0.060 → 0.080)
- `KP_BOOST_ERROR_PX` değerini düşür (daha erken büyük kazanıma geç)

---

## Test

```bash
python3 v2/new2.py
```

1. Sineği frame'in köşesine koy → servo hızla ortaya doğru gelmeli
2. Sineği merkezde tut → servo artık salınım yapmamalı (küçük KP ile hassas)
3. Overlay'deki `LASER LOCK X.Xs` süresi uzuyor mu? → iyi tracking
4. `TUNING.md`'ye eklenen değerleri kaydet
