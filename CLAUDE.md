# CLAUDE.md — Mosquito Laser Turret

## What this project does

Autonomous mosquito detection and laser targeting on Raspberry Pi 5.
Camera Module 3 → OpenCV detection pipeline → MG90S pan-tilt servos + laser module.

The active codebase is `v2/`. The `camera-stream/` and `motor/` directories contain earlier prototypes and utility scripts that are still useful for isolated testing.

## Repository layout

```
v2/new2.py            Main program (detection + servo tracking + laser state machine)
v2/fly_detect.py      Detection-only variant (no servos)
camera-stream/        rpicam-vid raw UDP stream + v1 fly detector
motor/                Servo test and calibration scripts
docs/HARDWARE.md      GPIO wiring reference
docs/TUNING.md        Detection parameter guide
```

## Hardware facts (don't get wrong)

- **Pan servo** → GPIO 12 (BCM), MG90S, pulse 1–2 ms
- **Tilt servo** → GPIO 13 (BCM), MG90S, pulse 1–2 ms
- **Laser** → GPIO 14 (BCM), controlled as `gpiozero.LED`
- **Trigger/relay** → GPIO 17 (BCM)
- GPIO factory **must** be `lgpio` on Pi 5: `os.environ["GPIOZERO_PIN_FACTORY"] = "lgpio"` before any gpiozero import
- Servo value formula: `(degrees / 90.0) - 1.0` maps 0–180° to –1.0–1.0

## Python environment

No virtualenv. System packages only:

```bash
sudo apt install -y python3-picamera2 python3-opencv python3-numpy python3-gpiozero python3-lgpio
```

Do not add pip-installed packages without noting the constraint.

## Camera

- `Picamera2` for capture (not the legacy `picamera`)
- Capture resolution: 1280×720 @ 30 fps, format RGB888
- Processing resolution: 640×360 (resize with `cv2.INTER_AREA`)
- Short exposure: `ExposureTime=5000` µs, `AnalogueGain=6.0` (disable AE)

## Detection pipeline (read before touching)

All processing is on the 640×360 grayscale frame.
Pipeline order matters — do not reorder without understanding consequences:

1. GaussianBlur (kernel 5)
2. CLAHE (clip 2.0, grid 8×8)
3. Frame diff → motion mask (threshold 20, dilate ×2)
4. Blackhat (kernel 17) → dark mask (threshold 18)
5. Adaptive threshold (block 21, C 9) → local dark mask
6. AND(motion_dilated, dark, adaptive) → combined
7. Contour filter: area 8–200 px², w/h 2–22 px, aspect 0.25–4.0
8. Score gates: motion_score ≥ 0.12, dark_score ≥ 18, local_mean ≤ 200
9. Global suppression: skip frame if motion pixels > 4% of frame area
10. Exclusion zones: large connected components (area ≥ 4000 or dim ≥ 110 px) are masked
11. Velocity tracker: exponential smoothing α=0.5, miss decay 0.7
12. Trajectory gate: path_length ≥ 4.0 px

## Laser state machine (`v2/new2.py`)

Three states: IDLE → LOCKED → KILLED
- Lock on first confirmed track with `hits ≥ MIN_HITS`
- While locked: proportional servo control with `KP_X = KP_Y = 0.040`
- After 3 continuous seconds: laser off, track ID added to `killed_flies`, return to IDLE
- If target lost before 3 s: laser off, return to IDLE immediately

## Servo control thread

`motor_smooth_thread()` runs at 5 ms intervals (200 Hz) in a background thread.
Lerp factor 0.22 — fast enough for mosquito tracking, smooth enough to avoid servo chatter.
Servos are `detach()`ed when idle to eliminate jitter.

## Stream

MJPEG over HTTP on port 8080:
- `/` and `/stream.mjpg` — annotated live feed
- `/debug` and `/debug.mjpg` — combined detection mask

## Common tasks

### Add or tune a detection parameter

Edit the constants block at the top of `v2/new2.py` (lines ~44–119).
Document the change in `docs/TUNING.md`.

### Add a new servo behavior

Control is via `target_pan_deg` / `target_tilt_deg` globals protected by `data_lock`.
Never write to servo hardware directly outside `motor_smooth_thread`.

### Test servos without running detection

```bash
python3 motor/manual-control5.py   # WASD keyboard
python3 motor/auto-scan2.py        # full-range sweep with laser blink
```

### Run detection without servo/laser hardware (e.g. on a desktop)

`fly_detect.py` and `fly_detector.py` handle missing `gpiozero` gracefully via try/except.
`new2.py` also has a `HAS_GPIO` guard — set GPIO imports to fail and it runs in software-only mode.

### Check if detector is already running

```bash
./camera-stream/fly-detector/fly-detector.sh status
```

## Safety rules — enforce these in all code changes

1. Always turn laser off before `picam2.stop()` in the finally block
2. Always `servo.detach()` before exit — energized servos left in stop() state will jitter
3. Never hold `data_lock` while sleeping or doing I/O
4. SIGINT/SIGTERM must reach the finally block cleanly — don't swallow signals

## Code style

- Python 3.10+ features are fine (Pi 5 ships Python 3.11+)
- Type hints on function signatures where non-obvious
- No comments explaining what the code does; only comments explaining non-obvious constraints
- Constants at the top of the file in an all-caps block, not scattered
- Threads are background daemons except `motor_smooth_thread` (must join for safe shutdown)
