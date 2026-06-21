# P11 — Kalibrasyon Sistemi

**Öncelik:** Kritik  
**Etkilenen dosyalar:** `v2/new2.py`, yeni: `v2/calibration.json`  
**Bağımlılık:** P1 (offset kod değişikliği), P8 (web endpoint altyapısı)

---

## Problem

P1 lazer-kamera offset'ini kod içinde sabit olarak tutuyor ve kalibrasyonu "tek seferlik, manuel" olarak tanımlıyor. Pratikte:

- Braket sökülüp takılırsa kalibrasyon bozulur
- Isıl genleşme veya mekanik sürüklenme (drift) zamanla birikir
- Şu an kalibrasyon için SSH + kod düzenleme + restart gerekiyor
- Kalibrasyon sonucu kayıt altına alınmıyor — restart'ta sıfırlanıyor
- **Derinlik (parallax) etkisi:** lazer ve kamera optik eksenleri arasındaki fiziksel mesafe (~2–5 cm), farklı hedef mesafelerinde farklı piksel offset'e yol açar

Bu plan bunların tamamını çözer: web'den tetiklenebilir, interaktif, kalıcı kalibrasyon akışı.

---

## Parallax Etkisi ve Pratik Yaklaşım

### Neden piksel offset mesafeye bağlı?

Kamera ve lazer aynı noktayı göremiyor — aralarında fiziksel bir mesafe (`d`) var:

```
d = 3 cm fiziksel offset varsayımı

1 m mesafede: arctan(0.03 / 1.0) ≈ 1.72°  → ~17 px (process res'te)
2 m mesafede: arctan(0.03 / 2.0) ≈ 0.86°  → ~8 px
3 m mesafede: arctan(0.03 / 3.0) ≈ 0.57°  → ~6 px
```

Sabit bir piksel offset ile sadece kalibrasyon mesafesinde tam isabet sağlanır.

### Pratik çözüm: referans mesafede kalibrasyon

Sivrisinek avı için tipik mesafe 0.5–2.5 m. **1.5 m referans mesafesinde** kalibrasyon yapılırsa:
- 1 m'deki hata: ~4 px → 0.4° → kabul edilebilir
- 2.5 m'deki hata: ~3 px → 0.3° → kabul edilebilir
- 0.3 m'deki hata: ~12 px → 1.2° → belirgin ama sivrisinek bu kadar yakına nadiren gelir

Bu nedenle sabit referans mesafede kalibrasyon yeterli. Daha gelişmiş çözüm (blob alanından mesafe tahmini + dinamik ölçekleme) P7 Yaklaşım B ile birleştirilebilir, bu plan kapsamı dışı.

**Kalibrasyon sırasında referans mesafeyi kaydet** — kullanıcıya hangi mesafede doğru olduğunu bildirmek için.

---

## Kalibrasyon Durum Makinesi

Sisteme yeni bir mod ekleniyor: `CALIBRATING`. Bu moddayken:
- Normal detection/tracking **duraklatılır**
- Tracker sıfırlanır
- Servolar merkeze (90°/90°) götürülür
- Lazer açılır
- Kullanıcı stream'de lazer noktasını tıklar
- Offset hesaplanır, `calibration.json`'a yazılır
- Sistem normal moda döner

```
NORMAL ──(POST /calibrate/start)──→ CALIBRATING
CALIBRATING ──(POST /calibrate/confirm)──→ SAVING──→ NORMAL
CALIBRATING ──(POST /calibrate/cancel)──→ NORMAL
```

---

## Implementasyon Adımları

### Adım 1 — Global durum değişkenleri ekle

`new2.py` globals bloğuna (satır 123-135 civarı):

```python
# Kalibrasyon modu
calibrating       = False          # True iken detection durur
calib_click_px: tuple[float, float] | None = None  # kullanıcının tıkladığı nokta
```

### Adım 2 — Kalibrasyon config sabitleri

`new2.py` sabitler bloğuna:

```python
CALIB_PATH = "/home/heliosx/v2/calibration.json"
# Camera Module 3, 1280×720 modunda yatay FoV yaklaşık 66°
# Process res (640px) başına derece: 66/640 ≈ 0.103
CAMERA_FOV_H_DEG = 66.0
CAMERA_FOV_V_DEG = 49.0
```

### Adım 3 — `_load_calibration()` fonksiyonu

(P1'de gösterildi, buraya referans olarak tam hali:)

```python
import json as _json

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
        log(f"Kalibrasyon yüklendi: "
            f"offset=({LASER_OFFSET_PX_X:.1f}, {LASER_OFFSET_PX_Y:.1f})px "
            f"@ {data.get('calibration_distance_cm', '?')}cm "
            f"[{data.get('calibrated_at', '?')}]")
    except Exception as e:
        log(f"Kalibrasyon okunamadı: {e} — offset=(0,0).")
```

### Adım 4 — `_save_calibration()` fonksiyonu

```python
import datetime as _dt

def _save_calibration(offset_px_x: float, offset_px_y: float,
                      distance_cm: int = 150) -> None:
    deg_per_px_h = CAMERA_FOV_H_DEG / PROCESS_W
    deg_per_px_v = CAMERA_FOV_V_DEG / PROCESS_H
    data = {
        "laser_offset_px_x":  round(offset_px_x, 2),
        "laser_offset_px_y":  round(offset_px_y, 2),
        "laser_offset_deg_x": round(offset_px_x * deg_per_px_h, 3),
        "laser_offset_deg_y": round(offset_px_y * deg_per_px_v, 3),
        "calibration_distance_cm": distance_cm,
        "calibrated_at": _dt.datetime.now().isoformat(timespec="seconds"),
        "process_resolution": f"{PROCESS_W}x{PROCESS_H}",
    }
    with open(CALIB_PATH, "w") as f:
        _json.dump(data, f, indent=2)
    log(f"Kalibrasyon kaydedildi: offset=({offset_px_x:.1f}, {offset_px_y:.1f})px "
        f"= ({data['laser_offset_deg_x']:.2f}°, {data['laser_offset_deg_y']:.2f}°) "
        f"@ {distance_cm}cm")
```

> Hem piksel hem derece kaydediliyor. Piksel runtime'da kullanılır; derece dokümantasyon ve hata ayıklama içindir.

### Adım 5 — `main()` döngüsüne kalibrasyon modu entegre et

Ana `while True:` döngüsünde, `frame_idx < WARMUP_FRAMES` bloğundan sonra:

```python
# KALIBRASYON MODU — detection ve tracking duraklatılır
if calibrating:
    # Servolar merkeze, lazer açık (web'den /calibrate/start tetikledi)
    overlay = frame.copy()
    cx_f, cy_f = CAPTURE_W // 2, CAPTURE_H // 2
    cv2.drawMarker(overlay, (cx_f, cy_f), (0, 255, 255),
                   cv2.MARKER_CROSS, 40, 2)
    cv2.putText(overlay, "KAL BRASYON MODU — Lazer noktasina tiklayin",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    if calib_click_px is not None:
        # Tıklanan nokta (capture res) → process res'e çevir
        click_proc_x = calib_click_px[0] * (PROCESS_W / CAPTURE_W)
        click_proc_y = calib_click_px[1] * (PROCESS_H / CAPTURE_H)
        cv2.drawMarker(overlay,
                       (int(calib_click_px[0]), int(calib_click_px[1])),
                       (0, 0, 255), cv2.MARKER_TILTED_CROSS, 30, 2)
        cv2.putText(overlay,
                    f"Secilen: ({click_proc_x:.0f}, {click_proc_y:.0f})px [process res]",
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 1)
    if ENABLE_STREAM:
        ok, jpg = cv2.imencode(".jpg", overlay, encode)
        if ok:
            buf_main.update(jpg.tobytes())
    frame_idx += 1
    continue   # detection ve state machine'i atla
```

### Adım 6 — `_load_calibration()` çağrısını `main()` başına ekle

```python
def main() -> None:
    global ...
    _load_calibration()   # ilk satır
    ...
```

---

## Web Endpoint'leri (P8 güncellemesi)

### Yeni endpoint tablosu

| Method | Yol | İşlev |
|---|---|---|
| GET | `/calibrate` | Kalibrasyon HTML sayfası |
| POST | `/calibrate/start` | Kalibrasyon moduna gir |
| POST | `/calibrate/click` | Kullanıcı tıkladığı piksel koordinatını gönderir |
| POST | `/calibrate/confirm` | Offset hesapla, kaydet, normal moda dön |
| POST | `/calibrate/cancel` | İptal et, normal moda dön |
| GET | `/calibrate/status` | Mevcut kalibrasyon değerlerini döndür |

### Handler implementasyonu

```python
elif self.path == "/calibrate/start":
    global calibrating, calib_click_px, current_target_id
    with data_lock:
        calibrating = True
        calib_click_px = None
        current_target_id = None      # aktif kilidi düşür
        tracker.tracks.clear()        # track'leri sıfırla
        if HAS_GPIO and lazer:
            lazer.on()                # kalibrasyonda lazer açık
        target_pan_deg  = 90.0        # merkeze götür
        target_tilt_deg = 90.0
    log("[KAL BR] Kalibrasyon modu başlatıldı.")
    self._json_ok({"status": "calibrating"})

elif self.path == "/calibrate/click":
    content_len = int(self.headers.get("Content-Length", 0))
    body = _json.loads(self.rfile.read(content_len))
    # body: {"x": 654, "y": 312}  — capture res piksel koordinatı
    with data_lock:
        calib_click_px = (float(body["x"]), float(body["y"]))
    self._json_ok({"status": "click_received", "x": body["x"], "y": body["y"]})

elif self.path == "/calibrate/confirm":
    content_len = int(self.headers.get("Content-Length", 0))
    body = _json.loads(self.rfile.read(content_len)) if content_len else {}
    distance_cm = int(body.get("distance_cm", 150))
    with data_lock:
        click = calib_click_px
        calibrating = False
        if HAS_GPIO and lazer:
            lazer.off()
    if click is None:
        self._json_ok({"status": "error", "msg": "Önce lazer noktasına tıkla"})
        return
    # Capture res koordinatını process res'e çevir
    proc_x = click[0] * (PROCESS_W / CAPTURE_W)
    proc_y = click[1] * (PROCESS_H / CAPTURE_H)
    offset_x = proc_x - PROCESS_W / 2
    offset_y = proc_y - PROCESS_H / 2
    _save_calibration(offset_x, offset_y, distance_cm)
    # Runtime'da da güncelle (restart gerekmeden aktif olsun)
    global LASER_OFFSET_PX_X, LASER_OFFSET_PX_Y
    LASER_OFFSET_PX_X = offset_x
    LASER_OFFSET_PX_Y = offset_y
    self._json_ok({
        "status": "calibrated",
        "offset_px_x": round(offset_x, 1),
        "offset_px_y": round(offset_y, 1),
        "distance_cm": distance_cm,
    })

elif self.path == "/calibrate/cancel":
    with data_lock:
        calibrating = False
        calib_click_px = None
        if HAS_GPIO and lazer:
            lazer.off()
    log("[KAL BR] Kalibrasyon iptal edildi.")
    self._json_ok({"status": "cancelled"})

elif self.path == "/calibrate/status":
    self._json_ok({
        "laser_offset_px_x": LASER_OFFSET_PX_X,
        "laser_offset_px_y": LASER_OFFSET_PX_Y,
        "calibrating": calibrating,
        "has_calibration_file": os.path.exists(CALIB_PATH),
    })
```

### Kalibrasyon HTML sayfası

```python
CALIBRATE_PAGE = """<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>Kalibrasyon</title>
<style>
body{background:#111;color:#eee;font-family:system-ui;margin:0;padding:16px}
button{background:#333;color:#eee;border:1px solid #555;padding:8px 16px;
       border-radius:4px;cursor:pointer;margin:4px}
button:hover{background:#444}
.ok{border-color:#3c3;color:#8f8} .danger{border-color:#c33;color:#f88}
#canvas-wrap{position:relative;display:inline-block;margin-top:12px}
#stream{max-width:640px;cursor:crosshair;border:2px solid #444;display:block}
#marker{position:absolute;width:20px;height:20px;pointer-events:none;
        border:2px solid red;border-radius:50%;display:none;
        transform:translate(-50%,-50%)}
#info{font-size:13px;color:#aaa;margin-top:8px}
#steps{color:#8cf;margin:12px 0;font-size:14px}
a{color:#6cf}
</style></head><body>
<h2>Lazer-Kamera Kalibrasyonu</h2>
<a href="/control">← Kontrol Paneli</a>

<div id="steps">
  Adım 1: "Kalibrasyonu Başlat"a tıkla → servolar merkeze gider, lazer açılır<br>
  Adım 2: Hedefi ~150 cm mesafeye koy, stream'de lazer noktasına tıkla<br>
  Adım 3: Mesafeyi gir ve "Onayla"ya bas → kalibrasyon kaydedilir
</div>

<div>
  <button class="ok" onclick="startCalib()">Kalibrasyonu Başlat</button>
  <button class="danger" onclick="cancelCalib()">İptal</button>
</div>

<div id="canvas-wrap">
  <img id="stream" src="/stream.mjpg" onclick="onStreamClick(event)"/>
  <div id="marker"></div>
</div>

<div id="info">Henüz tıklanmadı.</div>

<div style="margin-top:12px">
  Kalibrasyon mesafesi (cm):
  <input id="dist" type="number" value="150" min="30" max="500"
         style="background:#222;color:#eee;border:1px solid #555;
                padding:4px 8px;border-radius:4px;width:80px">
  <button class="ok" onclick="confirm()">Onayla ve Kaydet</button>
</div>

<div id="status" style="margin-top:12px;font-size:13px;color:#aaa;white-space:pre"></div>

<script>
let clickX = null, clickY = null;

function startCalib() {
  fetch('/calibrate/start', {method:'POST'})
    .then(r=>r.json())
    .then(d=>{ document.getElementById('info').textContent = 'Kalibrasyon modu aktif. Lazer açık. Lazer noktasına tıklayın.'; refreshStatus(); });
}

function onStreamClick(e) {
  const img = document.getElementById('stream');
  const rect = img.getBoundingClientRect();
  // Gösterilen boyut ile gerçek boyut arasındaki ölçek
  const scaleX = img.naturalWidth / rect.width;
  const scaleY = img.naturalHeight / rect.height;
  clickX = Math.round((e.clientX - rect.left) * scaleX);
  clickY = Math.round((e.clientY - rect.top)  * scaleY);

  // İşaretçiyi göster
  const marker = document.getElementById('marker');
  marker.style.left = (e.clientX - rect.left) + 'px';
  marker.style.top  = (e.clientY - rect.top)  + 'px';
  marker.style.display = 'block';

  document.getElementById('info').textContent =
    `Seçilen: (${clickX}, ${clickY}) [capture res]. Onayla ya da farklı bir noktaya tıkla.`;

  fetch('/calibrate/click', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({x: clickX, y: clickY})
  });
}

function confirm() {
  if (clickX === null) { alert('Önce lazer noktasına tıklayın.'); return; }
  const dist = parseInt(document.getElementById('dist').value) || 150;
  fetch('/calibrate/confirm', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({distance_cm: dist})
  }).then(r=>r.json()).then(d=>{
    document.getElementById('info').textContent =
      `Kaydedildi! offset=(${d.offset_px_x}, ${d.offset_px_y})px @ ${d.distance_cm}cm`;
    refreshStatus();
  });
}

function cancelCalib() {
  fetch('/calibrate/cancel', {method:'POST'})
    .then(r=>r.json())
    .then(()=>{ document.getElementById('info').textContent = 'İptal edildi.'; refreshStatus(); });
}

function refreshStatus() {
  fetch('/calibrate/status').then(r=>r.json()).then(d=>{
    document.getElementById('status').textContent = JSON.stringify(d, null, 2);
  });
}

refreshStatus();
setInterval(refreshStatus, 3000);
</script></body></html>"""
```

---

## Kalibrasyon Prosedürü (Kullanıcı Adımları)

1. Tarayıcıdan `http://heliosx.local:8080/calibrate` aç
2. "Kalibrasyonu Başlat" → servolar merkeze gider, lazer açılır
3. Hedefi **~150 cm** mesafeye koy (kağıt, bant veya düz yüzey)
4. Stream'de lazer noktasının tam üstüne tıkla — kırmızı daire işaret eder
5. Mesafe kutusuna `150` yaz, "Onayla ve Kaydet" → `calibration.json` oluşur
6. Normal moda dönülür, offset anında aktif olur (restart gerekmez)
7. Doğrulama: sineği (veya parmağı) frame'e getir, sistemi kilitlet, lazer üstüne düşüyor mu bak

### Ne Zaman Yeniden Kalibre Et?

- Braket sökülüp takılırsa
- İsabet oranı gözle görülür şekilde düştüyse
- Pi ve mekanik montaj yeni bir konuma taşındıysa
- `calibrate/status` endpoint'i `laser_offset_deg_x` değeri ≥ 3° gösteriyorsa (referans)

---

## `calibration.json` Örneği

```json
{
  "laser_offset_px_x": 14.5,
  "laser_offset_px_y": -6.2,
  "laser_offset_deg_x": 2.34,
  "laser_offset_deg_y": -0.99,
  "calibration_distance_cm": 150,
  "calibrated_at": "2026-06-21T16:45:03",
  "process_resolution": "640x360"
}
```

---

## Test

```bash
# Kalibrasyon dosyası yokken başlat
rm -f /home/heliosx/v2/calibration.json
python3 v2/new2.py
# Log: "Kalibrasyon dosyası yok, offset=(0,0) kullanılıyor."

# Web'den kalibrasyon yap
# (tarayıcıda http://heliosx.local:8080/calibrate)

# Dosya oluştu mu?
cat /home/heliosx/v2/calibration.json

# Sistemi yeniden başlat, dosya yüklenip yüklenmediğini kontrol et
python3 v2/new2.py
# Log: "Kalibrasyon yüklendi: offset=(14.5, -6.2)px @ 150cm [2026-06-21T16:45:03]"
```

---

## Riskler

- Web sayfasındaki stream görüntüsü tarayıcı tarafından ölçekleniyor olabilir. JavaScript `img.naturalWidth` kullanarak ölçeklemeyi telafi ediyor — img tag'inde `width` attribute'u varsa bu çalışmaz; `max-width: CSS` ile sınırlandırılmalı.
- Kalibrasyon sırasında lazer açık. Kullanıcı hedefi yakın tutmalı, lazer insana/hayvana yönelmemeli.
- `global LASER_OFFSET_PX_X` web handler thread'inden yazılıyor. Python'da basit float ataması GIL altında atomik, race condition riski yok. Yine de ana döngü aynı frame'de eski ve yeni değeri karışık kullanabilir — bu geçici ve tek frame etkili, kabul edilebilir.
