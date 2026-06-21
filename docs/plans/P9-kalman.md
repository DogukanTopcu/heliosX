# P9 — Kalman Filtresi ile Gelişmiş Tahmin

**Öncelik:** Nice-to-Have  
**Etkilenen dosya:** `v2/new2.py`  
**Bağımlılık:** P6 (çoklu hedef) tamamlanmış olursa daha verimli çalışır

---

## Problem

Şu anki `Track` sınıfı basit bir velocity smoother kullanıyor:

```python
# Track.predict() — new2.py satır 182-183
def predict(self) -> tuple[float, float]:
    return (self.cx + self.vx, self.cy + self.vy)
```

Ve miss durumunda hız azalıyor:
```python
# Tracker.update() — satır 273-274
tr.vx *= 0.7
tr.vy *= 0.7
```

Bu yaklaşım:
- Gürültülü ölçümü filtrelemiyor (her detection doğrudan konum olarak alınıyor)
- Miss frame'lerinde tahminin güvenilirliği zayıf
- Hızlı sinek (büyük `vx/vy`) için tahmin sapması büyük

Kalman filtresi hem gürültüyü hem de belirsizliği modelleyerek daha iyi tahmin sağlar.

---

## Kalman Modeli

**Durum vektörü:** `[cx, cy, vx, vy]` (4 boyut)  
**Ölçüm:** `[cx, cy]` (2 boyut — sadece konum gözlemleniyor)  
**Varsayım:** Sabit hız modeli (constant velocity)

OpenCV `cv2.KalmanFilter` kullanacağız — zaten kurulu, pip gerektirmiyor.

```
State transition matrix (F):
[1 0 dt 0]     dt = 1 frame (1/30 s)
[0 1 0 dt]
[0 0 1  0]
[0 0 0  1]

Measurement matrix (H):
[1 0 0 0]
[0 1 0 0]
```

---

## Kritik Tasarım Kısıtı: predict() Tek Çağrılmalı

`cv2.KalmanFilter.predict()` çağrıldığında filtrenin iç durumu ilerliyor — geri alınamaz. `Tracker.update()` şu an matching için `tr.predict()` çağırıyor (satır 231). Eğer bu metod `kf.predict()`'e bağlanırsa, ardından unmatched track propagation'ında bir kez daha `kf.predict()` çağrılır → tek frame'de iki kez ilerleme → filtre patlar.

**Çözüm:** `update()` başında tüm track'lerin predicted konumunu cache'le, sonra hem matching'de hem propagation'da bu cache'i kullan. `predict()` metodu artık `kf.predict()` çağırmaz — yalnızca cached değeri döner.

## Implementasyon Adımları

### Adım 1 — `Track` dataclass'ına Kalman ekle

`new2.py` dosyasının import bloğuna `from typing import Optional` ekle (zaten yoksa).

`Track` dataclass'ına `kf` field'ı ve `__post_init__` ekle — `cv2` zaten import edilmiş, alias gerekmez:

```python
@dataclass
class Track:
    track_id: int
    cx: float
    cy: float
    vx: float = 0.0
    vy: float = 0.0
    hits: int = 1
    misses: int = 0
    age: int = 1
    first_frame: int = 0
    triggered: bool = False
    history: deque = field(default_factory=lambda: deque(maxlen=TRAJECTORY_WINDOW))
    last_score: float = 0.0
    kf: object = field(default=None, repr=False, compare=False)   # YENİ

    def __post_init__(self):
        self.kf = cv2.KalmanFilter(4, 2)   # 4 state, 2 measurement
        dt = 1.0
        self.kf.transitionMatrix = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1,  0],
            [0, 0, 0,  1],
        ], dtype=np.float32)
        self.kf.measurementMatrix = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
        ], dtype=np.float32)
        self.kf.processNoiseCov = np.eye(4, dtype=np.float32) * 1e-2
        self.kf.processNoiseCov[2, 2] = 5.0
        self.kf.processNoiseCov[3, 3] = 5.0
        self.kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * 4.0
        self.kf.errorCovPost = np.eye(4, dtype=np.float32)
        self.kf.statePost = np.array(
            [[self.cx], [self.cy], [0.0], [0.0]], dtype=np.float32
        )
```

### Adım 2 — `predict()` metodunu kaldır, yerine `kalman_predict()` ekle

Mevcut `predict()` metodu matching'de kullanılıyor ama artık Kalman durumunu ilerletmemeli. Metodu yeniden adlandır ve semantiğini ayır:

```python
def kalman_predict(self) -> tuple[float, float]:
    """Kalman filtresini bir adım ilerlet, predicted konumu döndür."""
    pred = self.kf.predict()
    return (float(pred[0]), float(pred[1]))

def predict(self) -> tuple[float, float]:
    """Cache'lenmiş predicted konumu döndür (kf.predict çağırmaz)."""
    # Bu metod artık sadece _predicted cache'ini okuyor.
    # Cache Tracker.update() başında doldurulur.
    return getattr(self, "_pred_cx", self.cx), getattr(self, "_pred_cy", self.cy)
```

### Adım 3 — `Tracker.update()`'in başında predict cache'i doldur

`Tracker.update()` metodunun ilk satırına (satır 224 — `unmatched_tracks = ...`'dan önce):

```python
def update(self, dets: list[Detection], frame_idx: int) -> None:
    # Her track için Kalman'ı bir adım ilerlet, sonucu cache'le
    # predict() çift çağrısını önlemek için burada tek seferlik yapılıyor
    for tr in self.tracks.values():
        px, py = tr.kalman_predict()
        tr._pred_cx = px
        tr._pred_cy = py

    unmatched_tracks = set(self.tracks.keys())
    ...
```

### Adım 4 — Matched track'lerde `kf.correct()` çağır

`new2.py` satır 248-253:

```python
# ESKİ
tr.vx = 0.5 * tr.vx + 0.5 * new_vx
tr.vy = 0.5 * tr.vy + 0.5 * new_vy
tr.cx, tr.cy = float(d.cx), float(d.cy)
```

```python
# YENİ
meas = np.array([[float(d.cx)], [float(d.cy)]], dtype=np.float32)
corrected = tr.kf.correct(meas)
tr.cx = float(corrected[0])
tr.cy = float(corrected[1])
tr.vx = float(corrected[2])
tr.vy = float(corrected[3])
```

### Adım 5 — Unmatched track'lerde cache'i kullan (predict çağırma)

`new2.py` satır 270-276:

```python
# ESKİ
for tid in unmatched_tracks:
    tr = self.tracks[tid]
    tr.misses += 1
    tr.age += 1
    tr.vx *= 0.7
    tr.vy *= 0.7
    tr.cx += tr.vx
    tr.cy += tr.vy
```

```python
# YENİ — kf.predict() zaten Adım 3'te çağrıldı, cache'i oku
for tid in unmatched_tracks:
    tr = self.tracks[tid]
    tr.misses += 1
    tr.age += 1
    tr.cx = tr._pred_cx   # Adım 3'te hesaplanan Kalman tahmini
    tr.cy = tr._pred_cy
    # vx/vy Kalman state'inden güncellendi, manuel azaltma yok
    tr.vx = float(tr.kf.statePost[2])
    tr.vy = float(tr.kf.statePost[3])
    if tr.misses > MAX_MISSED:
        expired.append(tid)
```

---

## Gürültü Parametreleri

| Parametre | Değer | Etki |
|---|---|---|
| `processNoiseCov[0,0]` / `[1,1]` | 1e-2 | Konum modeli güveni (düşük = modele güven) |
| `processNoiseCov[2,2]` / `[3,3]` | 5.0 | Hız değişim belirsizliği (yüksek = hızlanmaya açık) |
| `measurementNoiseCov` | 4.0 | Kamera gürültüsü (~2 piksel std) |

Sinek çok hızlı dönüş yapıyorsa `processNoiseCov[2,2]` ve `[3,3]`'ü artır.  
Track pozisyonu çok sıçrıyorsa `measurementNoiseCov`'u artır.

---

## Riskler ve Notlar

- `cv2.KalmanFilter` her `Track` oluşturulduğunda initialize ediliyor. Birkaç track için overhead ihmal edilebilir.
- `field(default=None, repr=False, compare=False)`: `repr=False` dataclass print'ini temiz tutar; `compare=False` iki Track'i KF state'e göre karşılaştırmaktan kaçınır.
- `_pred_cx` / `_pred_cy` underscore-prefix attribute'ları dataclass field'ı değil, `__post_init__`'te oluşturulan runtime attribute. Bu Python'da geçerli.
- Kalman "sabit hız" varsayımı sivrisinek için yeterli; çok keskin dönüşlerde sapabilir ama `measurementNoiseCov` bunu dengeliyor.
- Bu değişiklik mevcut tracking mantığını derinden değiştiriyor. Eğer mevcut sistem iyi çalışıyorsa bu plan atlanabilir — P1-P7 gerçek öncelik taşıyor.
