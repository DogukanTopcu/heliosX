# Hardware Reference

## Bill of Materials

| Component | Model | Notes |
|---|---|---|
| SBC | Raspberry Pi 5 (4 GB or 8 GB) | Requires Pi 5 — lgpio not available on Pi 4 without extra config |
| Camera | Raspberry Pi Camera Module 3 | Connected via CSI ribbon cable |
| Pan servo | MG90S | Metal gear recommended for durability |
| Tilt servo | MG90S | Same model as pan |
| Laser | 5 mW 650 nm laser module (KY-008 or similar) | Operates at 3.3–5 V |
| Pan-tilt bracket | Two-axis 3D-printed or off-the-shelf bracket | Must fit MG90S horns |
| Power supply | 5 V / 5 A USB-C | Servos draw ~250 mA each under load |

## GPIO Pinout (BCM numbering)

```
 3V3  (1) (2)  5V
 SDA  (3) (4)  5V
 SCL  (5) (6)  GND
GPIO4 (7) (8)  GPIO14 ← LASER
 GND  (9) (10) GPIO15
GPIO17(11) (12) GPIO18    ← TRIGGER/RELAY on GPIO17
GPIO27(13) (14) GND
GPIO22(15) (16) GPIO23
 3V3 (17) (18) GPIO24
GPIO10(19) (20) GND
 GPIO9(21) (22) GPIO25
GPIO11(23) (24) GPIO8
 GND (25) (26) GPIO7
 SDA (27) (28) SCL
 GPIO5(29) (30) GND
 GPIO6(31) (32) GPIO12 ← PAN SERVO
GPIO13(33) (34) GND    ← TILT SERVO on GPIO13
GPIO19(35) (36) GPIO16
GPIO26(37) (38) GPIO20
 GND (39) (40) GPIO21
```

## Wiring Summary

| Signal | GPIO (BCM) | Physical Pin | Wire colour (suggested) |
|---|---|---|---|
| Pan servo signal | 12 | 32 | Yellow |
| Tilt servo signal | 13 | 33 | Yellow |
| Laser signal | 14 | 8 | Green |
| Trigger/relay | 17 | 11 | Blue |
| Servo / laser GND | GND | 34, 6, 9… | Black |
| Servo / laser VCC | 5 V | 2 or 4 | Red |

> **Servo power:** MG90S servos can be powered directly from the Pi's 5 V rail for bench testing, but a dedicated 5 V BEC or USB supply on the servo rail is recommended for prolonged use to avoid brownouts.

## MG90S Servo Specification

| Parameter | Value |
|---|---|
| Operating voltage | 4.8–6.0 V |
| Stall torque | 1.8 kg·cm @ 4.8 V |
| Speed | 0.1 s / 60° @ 4.8 V |
| PWM frequency | 50 Hz |
| Pulse width range | 1.0–2.0 ms (physical safe range) |
| Angular range | ~180° |

gpiozero mapping used in all scripts:

```python
min_pulse_width = 0.001   # 1 ms → 0° (servo min)
max_pulse_width = 0.002   # 2 ms → 180° (servo max)
```

Value formula: `servo.value = (degrees / 90.0) - 1.0`

## Laser Module (KY-008)

- Control pin: signal wire → GPIO 14
- Driven as `gpiozero.LED` (digital on/off)
- The module has an onboard current-limiting resistor; connect signal directly to GPIO
- Supply voltage: 5 V (VCC pin) or 3.3 V (some variants)
- **Class 2/3A laser — avoid direct eye exposure**

## Camera Mounting

The Camera Module 3 is mounted on the pan-tilt bracket so its optical axis aligns with the laser axis. Calibrate the offset between camera centre and laser dot after assembly (see `docs/TUNING.md`).

## Driver Requirements (Pi 5)

Pi 5 uses the `lgpio` backend — the old `pigpio` daemon is not required:

```bash
sudo apt install -y python3-lgpio
```

Force `lgpio` in all scripts before importing gpiozero:

```python
import os
os.environ["GPIOZERO_PIN_FACTORY"] = "lgpio"
```
