# P6 — Çoklu Hedef Önceliklendirme

**Öncelik:** Orta  
**Etkilenen dosya:** `v2/new2.py`  
**Bağımlılık:** Yok (P1 tamamlanmış olursa daha anlamlı test edilebilir)

---

## Problem

`new2.py` satır 714-723'te hedef seçimi:

```python
if current_target_id is None:
    for tr in active_tracks:
        if tr.track_id not in killed_flies and tr.hits >= MIN_HITS:
            current_target_id = tr.track_id
            lock_start_time = now
            ...
            break
```

`active_tracks` `tracker.tracks.values()` sıralamasına bağlı — ekleme sırasına göre.  
Birden fazla sinek varken:
- En yakın sineği değil, en eski track'i hedef alıyor
- Az isabet almış (yeni, belirsiz) bir track önce gelip kilitlenebiliyor
- Daha güvenilir (uzun trajektorili, merkeze yakın) sinek görmezden geliniyor

---

## Çözüm Yaklaşımı

Adayları bir skor fonksiyonuyla sırala, en yüksek skorluya kilitle.

### Skor Kriterleri

| Kriter | Mantık | Ağırlık |
|---|---|---|
| `hits` | Daha çok onaylanan track daha güvenilir | Yüksek |
| `path_length()` | Uzun yol = gerçek hareket, kamera gürültüsü değil | Orta |
| Merkeze uzaklık | Frame merkezine yakın = servo daha az hareket eder = isabet oranı yüksek | Orta |
| `last_score` | Blackhat + motion skoru = gerçekten sinek | Düşük-orta |

### Skor Formülü

```python
def candidate_score(tr: Track) -> float:
    dist_to_center = math.hypot(tr.cx - PROCESS_W / 2, tr.cy - PROCESS_H / 2)
    max_dist = math.hypot(PROCESS_W / 2, PROCESS_H / 2)  # köşeye uzaklık
    proximity = 1.0 - (dist_to_center / max_dist)         # 0-1, merkeze yakın = 1

    return (
        tr.hits        * 3.0 +
        tr.path_length() * 0.5 +
        proximity      * 20.0 +
        tr.last_score  * 5.0
    )
```

Ağırlıklar deneysel — `hits` ve `proximity`yi öne çıkardım.

---

## Implementasyon Adımları

### Adım 1 — `candidate_score` fonksiyonunu ekle

`new2.py`'de `Tracker` sınıfından sonra (satır 296'dan sonra) ekle:

```python
def candidate_score(tr: "Track") -> float:
    dist = math.hypot(tr.cx - PROCESS_W / 2, tr.cy - PROCESS_H / 2)
    max_dist = math.hypot(PROCESS_W / 2, PROCESS_H / 2)
    proximity = 1.0 - (dist / max_dist)
    return tr.hits * 3.0 + tr.path_length() * 0.5 + proximity * 20.0 + tr.last_score * 5.0
```

### Adım 2 — Hedef seçim bloğunu güncelle

`new2.py` satır 714-723:

**Önce:**
```python
if current_target_id is None:
    for tr in active_tracks:
        if tr.track_id not in killed_flies and tr.hits >= MIN_HITS:
            current_target_id = tr.track_id
            lock_start_time = now
            target_still_visible = True
            if HAS_GPIO:
                lazer.on()
            log(f"[KİLİTLENDİ] Hedef ID: {tr.track_id} yakalandı! Lazer AÇIK.")
            break
```

**Sonra:**
```python
if current_target_id is None:
    candidates = [
        tr for tr in active_tracks
        if tr.track_id not in killed_flies and tr.hits >= MIN_HITS
    ]
    if candidates:
        best = max(candidates, key=candidate_score)
        best_score = candidate_score(best)   # bir kez hesapla, iki yerde kullan
        current_target_id = best.track_id
        lock_start_time = now
        target_still_visible = True
        if HAS_GPIO:
            lazer.on()
        log(f"[KİLİTLENDİ] Hedef ID: {best.track_id} "
            f"skor={best_score:.1f} "
            f"hits={best.hits} path={best.path_length():.1f}px. Lazer AÇIK.")
```

### Adım 3 — Overlay'de skorları göster (opsiyonel, debug için)

`new2.py` satır 749-758 arasındaki track çizim döngüsünde:

```python
for tr in tracker.tracks.values():
    px = int(tr.cx * sx)
    py = int(tr.cy * sy)
    if tr.track_id == current_target_id:
        color = (255, 0, 0)
        # ...
    else:
        color = (0, 200, 200) if tr.hits < MIN_HITS else (0, 255, 0)
    cv2.circle(frame, (px, py), 12, color, 1)
    # YENİ: skor göster (debug için, istersen kaldır)
    if tr.hits >= MIN_HITS and tr.track_id not in killed_flies:
        cv2.putText(frame, f"{candidate_score(tr):.0f}",
                    (px + 14, py + 4), cv2.FONT_HERSHEY_SIMPLEX,
                    0.35, color, 1, cv2.LINE_AA)
```

---

## Test

1. Birden fazla hareket noktası oluştur (iki parmak veya iki cisim)
2. Her ikisi de track'e alınıyor mu? → tracker.tracks sayısı > 1 olmalı
3. Kilitlenen hedef gerçekten merkeze daha yakın mı veya daha fazla hit almış mı? → log'a `skor=` yazılıyor
4. Hedef imha edildikten (`killed_flies`) sonra ikinci hedefe geçiyor mu?

---

## Riskler

- `candidate_score` ağırlıkları (3.0, 0.5, 20.0, 5.0) deneysel. Aynı anda iki sinek varsa hangisinin seçilmesi daha iyi? Bunu sahada test ederek ağırlıkları ayarla.
- `proximity` ağırlığı yüksek tutuldu çünkü servonun az hareket etmesi = daha iyi isabet oranı. Ama uzak köşedeki sinek hiç hedef alınmayabilir — bu trade-off kabul edilebilir.
