# P4 — Servo Mekanik Güvenlik Sınırları

**Öncelik:** Kritik  
**Etkilenen dosya:** `v2/new2.py`  
**Bağımlılık:** Yok

---

## Problem

Şu an servo değeri şu şekilde kısıtlanıyor (`new2.py` satır 700-701):

```python
target_pan_deg  = max(0.0, min(180.0, target_pan_deg  - (error_x * kp_x)))
target_tilt_deg = max(0.0, min(180.0, target_tilt_deg - (error_y * kp_y)))
```

0–180° tam aralık kullanılıyor. Ancak:

1. **Mekanik fiziksel kısıt:** Pan-tilt braket çoğu zaman tüm 180°'yi kullanamaz. Kablolar gerilir, braket kendi yapısına çarpar.
2. **Lazer güvenliği:** 0° veya 180° uç noktalarda lazer odanın tehlikeli bir köşesini gösterebilir.
3. **`motor_smooth_thread`'de kısıt yok:** Thread hedef açıya doğru lerp yapıyor; hedef zaten kısıtlı olsa bile thread bağımsız çalıştığı için ekstra savunma katmanı faydalı.

---

## Mevcut Durum Analizi

Sabitler bloğunda (`new2.py` satır 105-119) servo sınırları için hiçbir değer yok.  
`motor_smooth_thread` (`new2.py` satır 453-487) doğrudan `target_pan_deg` / `target_tilt_deg` okuyor, herhangi bir clamp uygulamıyor.

---

## Çözüm Yaklaşımı

### Adım 1 — Sabit bloğuna mekanik sınırları ekle

`new2.py` satır 105-119 arasındaki pin sabitlerinden sonra:

```python
# Servo mekanik hareket sınırları (braketin fiziksel durumuna göre ayarla)
# Başlangıç değerleri güvenli orta bölgeyi tanımlıyor; gerçek sınırları test ederek bul
PAN_MIN_DEG  = 30.0   # 0° uç noktadan kaçın: kablo gerilmesi
PAN_MAX_DEG  = 150.0  # 180° uç noktadan kaçın: braket çarpması
TILT_MIN_DEG = 45.0   # Aşağı bakış sınırı (lazer yere değmesin)
TILT_MAX_DEG = 135.0  # Yukarı bakış sınırı
```

### Adım 2 — Servo kontrol satırlarında yeni sabitleri kullan

`new2.py` satır 699-701:

**Önce:**
```python
with data_lock:
    target_pan_deg  = max(0.0, min(180.0, target_pan_deg  - (error_x * kp_x)))
    target_tilt_deg = max(0.0, min(180.0, target_tilt_deg - (error_y * kp_y)))
```

**Sonra:**
```python
with data_lock:
    target_pan_deg  = max(PAN_MIN_DEG,  min(PAN_MAX_DEG,
                          target_pan_deg  - (error_x * kp_x)))
    target_tilt_deg = max(TILT_MIN_DEG, min(TILT_MAX_DEG,
                          target_tilt_deg - (error_y * kp_y)))
```

### Adım 3 — `motor_smooth_thread`'e savunma katmanı ekle

`new2.py` satır 468-474 (pan kontrol bölümü):

**Önce:**
```python
pan_diff = t_pan - current_pan_deg
if abs(pan_diff) > 0.05:
    current_pan_deg += pan_diff * smooth_factor
    pan_servo.value = deg_to_servo_val(current_pan_deg)
```

**Sonra:**
```python
pan_diff = t_pan - current_pan_deg
if abs(pan_diff) > 0.05:
    current_pan_deg += pan_diff * smooth_factor
    current_pan_deg = max(PAN_MIN_DEG, min(PAN_MAX_DEG, current_pan_deg))
    pan_servo.value = deg_to_servo_val(current_pan_deg)
```

Aynı pattern'i tilt için de uygula (satır 479-484).

### Adım 4 — Başlangıç pozisyonu sınırlar içinde mi kontrol et

`new2.py` satır 511-512 (kalibrasyon başlatma):

```python
pan_servo.value  = deg_to_servo_val(90)   # 90° her zaman PAN_MIN..PAN_MAX içinde
tilt_servo.value = deg_to_servo_val(90)   # 90° her zaman TILT_MIN..TILT_MAX içinde
```

90° her iki eksen için de merkez değer; yukarıdaki varsayılan sınırlar içinde kalıyor. Sorun yok.

---

## Sınır Değerlerini Fiziksel Olarak Belirleme

Bu işlemi bir kez yapman gerekiyor:

```bash
python3 motor/manual-control5.py   # WASD kontrolü
```

1. Pan ekseninde (A/D tuşları) braket kablosunun gerildiği veya mekanik tıkırdadığı açıyı bul → `PAN_MIN_DEG` ve `PAN_MAX_DEG`
2. Tilt ekseninde (W/S tuşları) aynısını yap → `TILT_MIN_DEG` ve `TILT_MAX_DEG`
3. Güvenlik payı olarak bulunan limit değerlerinin 5–10° içerisinde dur

### Tipik Değer Aralıkları (yaygın braketler)
| Eksen | Tipik Min | Tipik Max |
|---|---|---|
| Pan | 20–40° | 140–160° |
| Tilt | 40–60° | 120–140° |

---

## Test

```bash
python3 v2/new2.py
# Sineği (veya elinle simüle et) frame kenarına doğru sürekli hareket ettir
# Servo uç noktaya gitmeye çalışıyor ama tanımlı sınırda durmalı
# Kablo gerilmemeli, braket çarpmış ses çıkarmamalı
```

---

## Riskler

- Başlangıç değerleri (`30–150° / 45–135°`) conservative. İzleme alanı daralır ama güvenli.
- Gerçek braket sınırlarını test etmeden çok dar bir aralık seçmek, sineğin izlenebileceği alanı kısıtlar.
- `motor_smooth_thread`'deki ikinci clamp gereksiz görünse de, lock sırasında başka bir thread `target_pan_deg`'i sınır dışı bir değere yazarsa ikinci clamp koruma sağlar.
