# P3 — FPS Hesabı Sliding Window ile Düzeltme

**Öncelik:** Kritik  
**Etkilenen dosya:** `v2/new2.py`  
**Bağımlılık:** Yok

---

## Problem

`new2.py` satır 556 ve 812-813:

```python
t0 = time.time()
fps = 0.0
...
if frame_idx % 30 == 0:
    fps = frame_idx / (time.time() - t0)
```

Bu formül **başlangıçtan geçen toplam süreye** bakıyor. Sistem birkaç saat çalışırsa:
- Sistem ani bir CPU spike'ı yaşasa ve FPS düşse bile ekranda "29.8 FPS" gösterir
- Gerçek anlık performansı ölçmüyor
- WARMUP_FRAMES sırasında frame_idx sayılmaya devam ediyor ama o frameler işlenmiyor — ufak bir saptırma daha

---

## Mevcut Durum Analizi

`new2.py` içinde iki FPS kullanımı var:

1. **Ekranda gösterim** (satır 774): `f"FPS:{fps:4.1f} ..."`
2. **Log satırı** (satır 815): `f"FPS~{fps:.1f} ..."`

Her ikisi de aynı `fps` değişkenini okuyor.

---

## Çözüm Yaklaşımı

Son N frame'in toplam süresini tutan bir `deque` kullan. Her frame'de timestamp ekle, N frame öncesini çıkar. Basit, doğru, düşük overhead.

### Adım 1 — `t0` ve `fps` değişkenlerini değiştir

`new2.py` satır 554-557 arasındaki:

```python
frame_idx = 0
last_trigger = 0.0
t0 = time.time()
fps = 0.0
```

Şununla değiştir:

```python
frame_idx = 0
last_trigger = 0.0
fps = 0.0
_fps_window = 60          # son 60 frame üzerinden hesapla (~2 sn @ 30fps)
_frame_times: deque = deque(maxlen=_fps_window + 1)
```

### Adım 2 — Warmup sonrasında timestamp ekle

Timestamp'i `capture_array`'in hemen sonrasına değil, warmup bloğunun **sonrasına** koy. Aksi hâlde warmup sırasındaki yavaş frame'ler (kamera stabilizan oluyor) FPS hesabına karışır ve ilk ölçüm düşük çıkar.

```python
# Warmup bloğu
if frame_idx < WARMUP_FRAMES:
    frame_idx += 1
    continue

_frame_times.append(time.time())   # YENİ — warmup sonrası, gerçek frame'ler için
```

### Adım 3 — FPS hesabını güncelle

`new2.py` satır 812-813'teki:

```python
if frame_idx % 30 == 0:
    fps = frame_idx / (time.time() - t0)
```

Şununla değiştir:

```python
if frame_idx % 30 == 0 and len(_frame_times) >= 2:
    elapsed = _frame_times[-1] - _frame_times[0]
    fps = (len(_frame_times) - 1) / elapsed if elapsed > 0 else 0.0
```

### Adım 4 — Eski `t0` referanslarını temizle

`new2.py`'de `t0` başka bir yerde kullanılmıyor (sadece FPS için tanımlanmıştı). Satır 556'daki `t0 = time.time()` silinebilir.

---

## Test

```bash
python3 v2/new2.py
# Tarayıcıdan stream aç: http://heliosx.local:8080/
# Overlay'de FPS değeri 29-30 civarında olmalı
# CPU'ya yük bindirince (örn: başka bir işlem çalıştır) FPS'in düştüğünü görmeli
```

Eski davranış: FPS saatler sonra hâlâ "29.9" gösterirdi çünkü ortalama donup kalırdı.  
Yeni davranış: Son 2 saniyedeki gerçek throughput.

---

## Notlar

- `_fps_window = 60` → 30 fps'de ~2 saniyelik pencere. Daha stabil görüntü için 90 yapılabilir.
- Timestamp warmup sonrasına taşındı: warmup frame'leri (kamera ısınıyor, yavaş olabilir) FPS hesabını etkilemiyor.
- `deque(maxlen=_fps_window + 1)` boyutu: en eski ve en yeni arasındaki `_fps_window` aralık olsun diye +1.
