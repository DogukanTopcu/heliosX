# Mosquito Laser Turret

An autonomous mosquito-detection and laser-targeting system built on Raspberry Pi 5. A camera module captures live video; OpenCV detects small dark moving blobs (mosquitoes); two MG90S servo motors steer a pan-tilt mount toward the target; a laser module fires and holds on the mosquito for 3 seconds.

## System Overview

```
Camera → [picamera2] → Frame Buffer
                              │
                    ┌─────────▼──────────┐
                    │   Detection Pipeline│
                    │  CLAHE + Blackhat  │
                    │  Motion Diff + AND  │
                    └─────────┬──────────┘
                              │ confirmed tracks
                    ┌─────────▼──────────┐
                    │  Laser State Machine│
                    │  lock → track → kill│
                    └──┬──────────────┬──┘
                       │              │
               [Pan Servo]      [Tilt Servo]
               GPIO 12          GPIO 13
                                      │
                               [Laser Module]
                               GPIO 14
```

## Hardware

| Component | Detail |
|---|---|
| SBC | Raspberry Pi 5 |
| Camera | Camera Module 3 |
| Pan servo | MG90S on GPIO 12 |
| Tilt servo | MG90S on GPIO 13 |
| Laser | 5 mW module on GPIO 14 |
| Trigger/relay | GPIO 17 |

Full wiring and GPIO reference: [docs/HARDWARE.md](docs/HARDWARE.md)

## Project Structure

```
.
├── v2/                        ← Active codebase
│   ├── new2.py                ← Main program: detection + servo tracking + laser
│   ├── fly_detect.py          ← Detection-only variant (no servo control)
│   └── new.py                 ← Intermediate prototype
│
├── camera-stream/             ← Raw camera stream utility
│   ├── camera-stream.sh       ← Start/stop rpicam-vid UDP stream
│   └── fly-detector/          ← v1 detection (no servo, UDP ffmpeg stream)
│       ├── fly_detector.py
│       └── fly-detector.sh
│
├── motor/                     ← Servo development and test scripts
│   ├── pan-tilt.py            ← Basic min/mid/max sweep test
│   ├── manual-control5.py     ← WASD keyboard control (smooth lerp)
│   ├── auto-scan.py           ← Automated full-range sweep
│   └── auto-scan2.py          ← Sweep + laser blink
│
└── docs/
    ├── HARDWARE.md            ← Wiring, GPIO map, BOM
    └── TUNING.md              ← Detection parameter reference
```

## Quick Start

### Dependencies

```bash
sudo apt update
sudo apt install -y python3-picamera2 python3-opencv python3-numpy python3-gpiozero python3-lgpio
```

### Run the full system

```bash
cd ~/v2
python3 new2.py
```

Open a browser on any device on the same network:

- `http://heliosx.local:8080/` — live annotated stream
- `http://heliosx.local:8080/debug` — detection mask view

### Run detection-only (no servos)

```bash
cd ~/v2
python3 fly_detect.py
```

### Manual servo control (keyboard)

```bash
cd ~/motor
python3 manual-control5.py
# W/S = tilt up/down   A/D = pan left/right   Q = quit
```

### Raw camera stream to another machine

```bash
# Receiving machine (Mac/PC):
ffplay -fflags nobuffer -flags low_delay -framedrop udp://@:5000

# Pi:
./camera-stream/camera-stream.sh start <receiver-ip> 5000
```

## Detection Algorithm

The pipeline runs at 640×360 for CPU efficiency and scales results back to 1280×720 for display.

1. **CLAHE** — local contrast enhancement to reveal dark blobs on bright backgrounds
2. **Frame diff** → motion mask — rejects static edges and lighting drift
3. **Blackhat morphology** → dark blob mask — highlights objects darker than surroundings
4. **Adaptive threshold** → local dark mask
5. **AND(motion, blackhat, adaptive)** → combined detection mask — requires all three conditions simultaneously; hands/arms pass motion but fail blackhat/adaptive
6. **Contour filtering** — area, width, height, and aspect ratio gates
7. **Scoring** — each candidate is scored on motion pixel ratio, blackhat mean, and local brightness
8. **Global suppression** — if >4% of the frame moves, the whole frame is skipped (large-object rejection)
9. **Exclusion zones** — large connected motion components are masked out (finger/arm regions)
10. **Velocity tracker** — Kalman-lite: predict position from velocity, match nearest detection, exponential velocity smoothing
11. **Trajectory confirmation** — track must accumulate minimum path length before being considered confirmed

## Laser State Machine (`new2.py`)

```
IDLE ──(new track with hits≥2)──→ LOCKED
  ↑                                  │
  │                           proportional servo
  │                           tracking toward center
  │                                  │
  ├──(target lost)────────────────── │
  │                                  ▼
  └──(3 seconds elapsed)──── KILLED (add to killed set)
```

- `TRIGGER_COOLDOWN = 2.0 s` — minimum time between laser firings
- `KP_X = KP_Y = 0.040` — proportional gain for pan/tilt error correction
- Servo smoothing: lerp factor 0.22, 5 ms loop (200 Hz effective)

## Calibration

See [docs/TUNING.md](docs/TUNING.md) for the full parameter reference.

Key constants in `v2/new2.py`:

| Constant | Default | Effect |
|---|---|---|
| `MIN_AREA` / `MAX_AREA` | 8 / 200 px² | Reject too-small or too-large blobs |
| `BLACKHAT_KERNEL` | 17 | Larger = catches motion-blurred mosquitoes |
| `MOTION_THRESHOLD` | 20 | Lower = more sensitive to movement |
| `MIN_MOTION_SCORE` | 0.12 | Fraction of bbox pixels that must be in motion |
| `MIN_DARK_SCORE` | 18.0 | Blackhat mean inside bbox |
| `MAX_LOCAL_MEAN` | 200.0 | Reject blobs in bright patches |
| `MATCH_DISTANCE` | 90 px | Max inter-frame displacement for track association |
| `MIN_PATH_LENGTH` | 4.0 px | Minimum total trajectory length to confirm |

## Safety

**The laser is a Class 2/3A device. Never point it at people or animals. Always ensure the physical environment is safe before enabling autonomous firing.**

- The laser is off by default and only activates after a track is confirmed
- `TRIGGER_COOLDOWN` prevents rapid repeat firing
- Killing the process (`Ctrl+C`) immediately runs the safe-shutdown block: laser off, servos detached, camera stopped
