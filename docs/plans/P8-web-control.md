# P8 — Web Kontrol Paneli

**Öncelik:** Nice-to-Have  
**Etkilenen dosya:** `v2/new2.py`  
**Bağımlılık:** P1–P4 tamamlanmış olmalı, P11 (kalibrasyon akışı bu plana entegre edilmeli)

---

## Problem

Şu an web arayüzü salt okunur:
- `/` → canlı stream
- `/debug` → mask görüntüsü

Sistemi durdurmak için SSH gerekiyor.  
Parametreleri değiştirmek için kodu düzenleyip yeniden başlatmak gerekiyor.  
Lazer durumunu göremiyorsun (stream'de yazı var ama dedicated bir indicator yok).

---

## Çözüm Yaklaşımı

Mevcut `ThreadedHTTPServer`'a yeni endpoint'ler ekle. Ayrı bir framework (Flask, FastAPI) kullanma — `python3-flask` sistem paketleri arasında değil, pip gerektirir. Mevcut `http.server` genişletilecek.

### Endpoint Tablosu

| Method | Yol | İşlev |
|---|---|---|
| GET | `/status` | JSON: FPS, track sayısı, lazer durumu, hedef ID |
| POST | `/laser/off` | Lazer'i zorla kapat, mevcut kilidi sıfırla |
| POST | `/laser/on` | Lazer'i manuel aç (test için) |
| POST | `/reset` | `killed_flies` setini temizle (yeni seans) |
| GET | `/control` | Ana kontrol sayfası |
| GET | `/calibrate` | **Kalibrasyon sayfası (P11)** |
| POST | `/calibrate/start` | **Kalibrasyon moduna gir** |
| POST | `/calibrate/click` | **Kullanıcı tıkladığı koordinatı gönder** |
| POST | `/calibrate/confirm` | **Kaydet, normal moda dön** |
| POST | `/calibrate/cancel` | **Kalibrasyon iptali** |
| GET | `/calibrate/status` | **Mevcut kalibrasyon değerleri (JSON)** |

> Kalibrasyon endpoint'lerinin tam implementasyonu **[P11](P11-calibration-system.md)**'de. Bu plan sadece kontrol arayüzünü kapsar.

---

## Implementasyon Adımları

### Thread Güvenliği (kritik ön not)

HTTP handler'lar ayrı thread'lerde çalışıyor. `current_target_id`, `killed_flies`, `current_pan_deg`, `current_tilt_deg` gibi değişkenlere doğrudan erişmek race condition yaratır. Tüm paylaşılan durum okuma/yazmaları `data_lock` altında yapılmalı.

Ayrıca `POST /laser/on` komutunun bir çakışma riski var: lazer açılsa bile state machine bir sonraki frame'de `current_target_id is None` kontrolü yapıp `lazer.off()` çağırır. Bu endpointi sadece `current_target_id` da set edilerek kullanılabilir hale getiriyoruz — aşağıya bak.

### Adım 0 — Paylaşılan durum için global değişkenler ekle

`new2.py` globallerinin tanımlandığı bölüme (satır 123-135 civarı) şunu ekle:

```python
# Web handler thread'inin thread-safe okuyabileceği snapshot'lar
# data_lock altında güncelleniyor
web_status: dict = {}
```

Ana döngüde her frame'de, state machine bloğundan sonra `data_lock` altında güncelle:

```python
with data_lock:
    target_pan_deg  = ...   # zaten var
    target_tilt_deg = ...   # zaten var
    # YENİ: web snapshot
    web_status = {
        "fps":       fps,
        "tracks":    len(tracker.tracks),
        "confirmed": len(confirmed),
        "laser":     bool(lazer.is_lit) if HAS_GPIO and lazer else False,
        "target_id": current_target_id,
        "killed":    len(killed_flies),
        "pan_deg":   round(current_pan_deg, 1),
        "tilt_deg":  round(current_tilt_deg, 1),
    }
```

### Adım 1 — JSON status endpoint

`make_handler` fonksiyonuna yeni bir `do_GET` dalı ekle:

```python
elif self.path == "/status":
    import json
    with data_lock:
        snapshot = dict(web_status)   # kopyala, lock dışında serileştir
    body = json.dumps(snapshot).encode()
    self.send_response(200)
    self.send_header("Content-Type", "application/json")
    self.send_header("Content-Length", str(len(body)))
    self.end_headers()
    self.wfile.write(body)
```

### Adım 2 — POST handler ekle

```python
def do_POST(self):
    global current_target_id, lock_start_time, killed_flies

    if self.path == "/laser/off":
        # data_lock altında hem lazer kapat hem state'i sıfırla
        with data_lock:
            if HAS_GPIO and lazer:
                lazer.off()
            current_target_id = None
        log("[WEB] Lazer zorla kapatıldı.")
        self._json_ok({"status": "laser_off"})

    elif self.path == "/laser/on":
        # Sadece test amaçlı — state machine override eder, dikkatli kullan
        # Bu endpoint lazer açar ama track kilitleme yapmaz.
        # State machine bir sonraki frame'de zaten aktif track yoksa kapatır.
        # Gerçekten açık tutmak istiyorsan önce sineği kilitle (normal flow).
        with data_lock:
            if HAS_GPIO and lazer:
                lazer.on()
        log("[WEB] Lazer manuel açıldı (test modu — state machine override edebilir).")
        self._json_ok({"status": "laser_on", "warning": "state machine may override"})

    elif self.path == "/reset":
        with data_lock:
            killed_flies.clear()
            current_target_id = None
        log("[WEB] killed_flies sıfırlandı.")
        self._json_ok({"status": "reset"})

    else:
        self.send_error(404)

def _json_ok(self, payload: dict):
    import json
    body = json.dumps(payload).encode()
    self.send_response(200)
    self.send_header("Content-Type", "application/json")
    self.send_header("Content-Length", str(len(body)))
    self.end_headers()
    self.wfile.write(body)
```

### Adım 3 — HTML kontrol sayfası

Kontrol sayfası üç bölümden oluşur: sistem durumu, operasyon kontrolleri ve kalibrasyon linki.

```python
CONTROL_PAGE = """<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>Turret Control</title>
<style>
body{background:#111;color:#eee;font-family:system-ui;margin:0;padding:16px}
button{background:#333;color:#eee;border:1px solid #555;padding:8px 16px;
       border-radius:4px;cursor:pointer;margin:4px}
button:hover{background:#444}
.danger{border-color:#c33;color:#f88}
.ok{border-color:#3c3;color:#8f8}
.section{margin-top:20px;padding:12px;border:1px solid #333;border-radius:6px}
h3{margin:0 0 10px;font-size:14px;color:#aaa;text-transform:uppercase;letter-spacing:.05em}
#status{font-size:13px;color:#aaa;white-space:pre}
a{color:#6cf}
</style></head><body>
<h2>Turret Kontrol Paneli</h2>
<a href="/">← Stream</a> | <a href="/debug">Debug</a> | <a href="/calibrate">Kalibrasyon</a>

<div class="section">
  <h3>Sistem Durumu</h3>
  <div id="status">Yükleniyor...</div>
</div>

<div class="section">
  <h3>Operasyon</h3>
  <button class="danger" onclick="post('/laser/off')">Lazer KAPAT</button>
  <button onclick="post('/laser/on')">Lazer AÇ (test)</button>
  <button onclick="post('/reset')">Hedef Listesini Sıfırla</button>
</div>

<div class="section">
  <h3>Kalibrasyon</h3>
  <a href="/calibrate">
    <button class="ok">Kalibrasyon Arayüzünü Aç</button>
  </a>
  <div id="calib-status" style="font-size:12px;color:#888;margin-top:8px"></div>
</div>

<script>
function post(url){fetch(url,{method:'POST'}).then(r=>r.json()).then(d=>console.log(d))}
function refresh(){
  fetch('/status').then(r=>r.json()).then(d=>{
    document.getElementById('status').textContent = JSON.stringify(d,null,2)
  })
  fetch('/calibrate/status').then(r=>r.json()).then(d=>{
    const hasFile = d.has_calibration_file;
    const ox = d.laser_offset_px_x?.toFixed(1);
    const oy = d.laser_offset_px_y?.toFixed(1);
    document.getElementById('calib-status').textContent =
      hasFile ? `Aktif offset: (${ox}, ${oy})px` : 'Kalibrasyon dosyası yok — offset=(0,0)';
  })
}
setInterval(refresh, 1000)
refresh()
</script></body></html>"""
```

Ve `do_GET`'e:
```python
elif self.path == "/control":
    self._send_page(CONTROL_PAGE)
elif self.path == "/calibrate":
    self._send_page(CALIBRATE_PAGE)   # P11'den alınan sayfa
```

---

## Güvenlik Notu

Bu endpoint'ler ağ üzerinden herkese açık. Lazer'i uzaktan açabilecek bir API tehlikeli.  
Eğer ağda başkaları varsa en azından basit bir token kontrolü ekle:

```python
# POST handler başında:
token = self.headers.get("X-Turret-Token", "")
if token != "my-secret-token-123":
    self.send_error(403)
    return
```

Komutları token ile gönder:
```bash
curl -X POST http://heliosx.local:8080/laser/off -H "X-Turret-Token: my-secret-token-123"
```

---

## Test

```bash
# Status kontrol
curl http://heliosx.local:8080/status

# Lazer kapat
curl -X POST http://heliosx.local:8080/laser/off

# Web kontrol sayfası
open http://heliosx.local:8080/control
```
