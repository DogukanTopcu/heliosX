# Fly Detector

Bu klasör legacy detector akışıdır. Ana sistem artık `v2/new2.py` üzerinden ilerliyor. Buradaki araçlar izole detector denemeleri veya eski çalışma biçimini tekrar etmek için tutuluyor.

## Stream annotated output to a Mac or PC

Open the receiver first on the computer:

```bash
ffplay -fflags nobuffer -flags low_delay -framedrop udp://@:6000
```

Then run on the Pi with full-frame tracking:

```bash
cd ~/camera-stream/fly-detector
./fly-detector.sh start --stream-ip 192.168.222.112 --stream-port 6000 --mirror --min-area 14 --max-area 120 --motion-threshold 20 --black-threshold 26 --min-hits 3 --min-track-speed 1.2 --min-motion-score 0.22 --min-dark-score 30 --max-local-mean 170 --display-delay-ms 250 --print-every 30
```

## Control

```bash
./fly-detector.sh status
./fly-detector.sh logs
./fly-detector.sh stop
```

## Useful tuning flags

- `--min-area`, `--max-area`: reject blobs that are too small or too large
- `--motion-threshold`: lower is more sensitive to movement
- `--black-threshold`: lower is more sensitive to dark spots
- `--min-track-speed`: reject slow flicker and static false positives
- `--min-motion-score`: reject weak motion candidates
- `--min-dark-score`: reject weak dark-blob candidates
- `--max-local-mean`: reject blobs in overly bright local patches
- `--display-delay-ms`: only show a track after it survives this many milliseconds
- `--mirror`: mirror the final preview and streamed output horizontally
- `--debug-mask`: overlay the detection mask on the video

## Scope note

- Bu akış servo kontrolü, yeni kalibrasyon UI'si ve `v2` içindeki state machine davranışının kaynağı değildir.
- Ana sistemle çelişki varsa `v2` dokümantasyonu esas alınır.
