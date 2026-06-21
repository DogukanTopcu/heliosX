# P2 — Log Dosyasına Yazma (Kod Var, Çalışmıyor)

**Öncelik:** Kritik  
**Etkilenen dosya:** `v2/new2.py`  
**Bağımlılık:** Yok

---

## Problem

`new2.py` satır 117-118'de log dosyası sabitleri var:

```python
LOG_TO_FILE = True
LOG_PATH = "/home/heliosx/v2/detections.log"
```

Ve satır 39-41'de `log()` fonksiyonu:

```python
def log(msg: str) -> None:
    logging.info(msg)
```

Ama `logging.basicConfig` sadece `StreamHandler` (stdout) kullanıyor — `FileHandler` hiç eklenmemiş.  
`LOG_TO_FILE` ve `LOG_PATH` sabitleri tamamen işlevsiz. Sistem çalışırken ne olduğunu geriye dönük inceleme imkânı yok.

---

## Mevcut Durum Analizi

`new2.py` satır 33-41:

```python
logging.basicConfig(
    level=logging.INFO,
    format='[%(asctime)s] %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)

def log(msg: str) -> None:
    logging.info(msg)
```

`basicConfig()` çağrıldıktan sonra handler eklemek mümkün değil (Python logging'in bilinen sınırlaması).  
`LOG_TO_FILE` sabiti hiçbir yerde kontrol edilmiyor.

---

## Çözüm Yaklaşımı

`basicConfig` yerine handler'ları elle kur. Bu sayede stdout ve dosya aynı anda çalışır.

### Adım 1 — Mevcut `logging.basicConfig` bloğunu kaldır

`new2.py` satır 33-41'deki şu bloğu **tamamen sil**:

```python
logging.basicConfig(
    level=logging.INFO,
    format='[%(asctime)s] %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)

def log(msg: str) -> None:
    logging.info(msg)
```

### Adım 2 — Sabit bloğunun sıralaması (kritik)

> **Uyarı:** `logging.basicConfig` şu an **satır 33**'te, `LOG_TO_FILE` sabiti ise **satır 117**'de. Yeni handler setup'ı `LOG_TO_FILE`'ı okuyacak — bu değişkenden önce çalışırsa `NameError` alırsın.
>
> Çözüm: Yeni logging bloğunu sabitler bloğundan **sonra** (satır 119'dan sonra) yerleştir. `log()` fonksiyonu da aynı yere taşınacak.

### Adım 3 — Yeni logging bloğunu sabitlerden sonra ekle

Sabitler bloğunun hemen altına (satır 119 sonrası) şunu ekle:

```python
# =========================================================================
# LOGGING KURULUMU (sabitlerden sonra — LOG_TO_FILE ve LOG_PATH burada okunuyor)
# =========================================================================
import logging.handlers as _log_handlers

_log_fmt = logging.Formatter('[%(asctime)s] %(message)s', datefmt='%Y-%m-%d %H:%M:%S')
_logger = logging.getLogger("turret")
_logger.setLevel(logging.INFO)

_sh = logging.StreamHandler()
_sh.setFormatter(_log_fmt)
_logger.addHandler(_sh)

if LOG_TO_FILE:
    _fh = _log_handlers.RotatingFileHandler(
        LOG_PATH, maxBytes=5 * 1024 * 1024, backupCount=3, encoding="utf-8"
    )
    _fh.setFormatter(_log_fmt)
    _logger.addHandler(_fh)

def log(msg: str) -> None:
    _logger.info(msg)
```

> **Neden RotatingFileHandler?** Sistem günlerce çalışırsa düz dosya şişer. 5 MB × 3 yedek = max 15 MB disk kullanımı. `import logging.handlers` dosyanın üstüne taşınmak yerine burada yapıldı — yalnızca `LOG_TO_FILE = True` ise kullanılıyor, gereksiz yükleme yok.

### Adım 3 — Detection event'lerini logla

Şu an sadece sistem mesajları loglanıyor. Tespit olaylarını da ekle.

`new2.py` satır 684 civarında "KİLİTLENDİ" logu var. Bunlara ek olarak her confirmed detection'ı da logla:

`new2.py` satır 726-733 arasındaki trigger bloğuna ekle:

```python
for tr in confirmed:
    if not tr.triggered and (now - last_trigger) > TRIGGER_COOLDOWN:
        if trigger is not None:
            trigger.on()
            time.sleep(0.05)
            trigger.off()
        last_trigger = now
        tr.triggered = True
        # YENİ: detection event logla
        log(f"[DETECTION] id={tr.track_id} cx={tr.cx:.0f} cy={tr.cy:.0f} "
            f"hits={tr.hits} path={tr.path_length():.1f}px speed={tr.speed():.1f}px/f")
```

---

## Test

```bash
# Sistemi başlat
python3 v2/new2.py &

# Birkaç saniye bekle, sonra log dosyasını kontrol et
tail -f /home/heliosx/v2/detections.log
```

Beklenen çıktı:
```
[2026-06-21 14:32:01] Kamera: 1280x720 @ 30fps, process 640x360
[2026-06-21 14:32:02] Stream: http://heliosx.local:8080/
[2026-06-21 14:32:15] [KİLİTLENDİ] Hedef ID: 3 yakalandı! Lazer AÇIK.
[2026-06-21 14:32:15] [DETECTION] id=3 cx=287.0 cy=201.0 hits=4 path=12.3px speed=2.1px/f
```

---

## Riskler

- `LOG_TO_FILE = False` yaparsan hiçbir şey değişmez (eski davranış korunur)
- SD kart ömrü: Pi üzerinde sürekli yazma SD kartı yorabilir. `LOG_PATH`'i `/tmp/` altına almak bunu engeller ama reboot'ta silinir. `/home/heliosx/v2/` makul bir tercih (sık yazma değil, her event'te bir satır).
- `logging.handlers` import'u yalnızca `LOG_TO_FILE = True` olduğunda yapılıyor — `False` durumunda gereksiz import yok.
