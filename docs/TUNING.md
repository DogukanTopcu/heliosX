# Detection Tuning Guide

All constants are in the block at the top of `v2/new2.py` (lines ~44–119). Change one at a time and observe the debug stream at `http://heliosx.local:8080/debug`.

## Workflow

1. Start `new2.py`
2. Open the debug view in a browser: `http://heliosx.local:8080/debug`
3. The debug mask shows:
   - **Blue pixels** — motion detected
   - **Green pixels** — pixels that pass all three masks (motion + blackhat + adaptive)
   - **Red rectangles** — candidate detections before tracking
4. Adjust one constant, restart, observe

## Camera Parameters

| Constant | Default | Tuning guidance |
|---|---|---|
| `CAPTURE_W/H` | 1280×720 | Do not reduce — lower resolution makes small mosquitoes disappear |
| `PROCESS_W/H` | 640×360 | Can lower to 480×270 for faster processing on weaker hardware |
| `TARGET_FPS` | 30 | 30 is the practical max for detection; lower if CPU is saturated |
| `EXPOSURE_TIME_US` | 5000 µs | Lower (e.g. 3000) to freeze motion; raise in dim light. Set to 0 for auto-exposure |
| `ANALOGUE_GAIN` | 6.0 | Increase in low light; above 8 adds significant noise |

## Preprocessing

| Constant | Default | Effect |
|---|---|---|
| `BLUR_KERNEL` | 5 | Must be odd. Higher = smoother but blurs small blobs |
| `CLAHE_CLIP` | 2.0 | Higher = more aggressive local contrast. Increase if mosquito is barely visible |
| `CLAHE_GRID` | (8, 8) | Tile size for CLAHE. Smaller tiles = more local but slower |

## Motion Mask

| Constant | Default | Effect |
|---|---|---|
| `MOTION_THRESHOLD` | 20 | Pixel brightness change to count as motion. Lower = more sensitive (more false positives from noise) |
| `MOTION_DILATE` | 2 | Dilation iterations on motion mask. Higher fills gaps around fast-moving blobs |

## Dark Blob Mask (Blackhat)

| Constant | Default | Effect |
|---|---|---|
| `BLACKHAT_KERNEL` | 17 | Must be odd. Larger kernel catches motion-blurred or larger mosquitoes. Must be significantly larger than the mosquito blob |
| `BLACK_THRESHOLD` | 18 | Blackhat response threshold. Lower = more candidates, more noise |

## Adaptive Threshold

| Constant | Default | Effect |
|---|---|---|
| `ADAPTIVE_BLOCK` | 21 | Must be odd. Local neighbourhood size for adaptive threshold |
| `ADAPTIVE_C` | 9 | Constant subtracted from local mean. Higher = stricter (fewer detections) |

## Blob Size and Shape Gates

These are applied after contour extraction on the 640×360 processing frame.

| Constant | Default | Tuning guidance |
|---|---|---|
| `MIN_AREA` | 8 px² | Reduce if mosquito is very small or far away |
| `MAX_AREA` | 200 px² | Increase if mosquito is close and appears large |
| `MIN_WH` | 2 px | Minimum bounding box dimension |
| `MAX_WH` | 22 px | Maximum bounding box dimension |
| `MIN_ASPECT` | 0.25 | Minimum w/h ratio (very tall blobs rejected) |
| `MAX_ASPECT` | 4.0 | Maximum w/h ratio (very wide blobs rejected) |

## Per-Detection Scoring Gates

| Constant | Default | Effect |
|---|---|---|
| `MIN_MOTION_SCORE` | 0.12 | Fraction of bbox pixels that must be active in the motion mask. Raise to reject weak motion |
| `MIN_DARK_SCORE` | 18.0 | Mean blackhat response inside bbox. Raise to reject light-coloured blobs |
| `MAX_LOCAL_MEAN` | 200.0 | CLAHE mean inside bbox. Rejects blobs in overly bright regions. Lower if mosquito flies near lights |

## Global Motion Suppression

| Constant | Default | Effect |
|---|---|---|
| `MAX_GLOBAL_MOTION_RATIO` | 0.04 | If more than 4% of frame pixels show motion, skip entire frame. Prevents false detections when a hand enters the frame. Raise if legitimate tracking gets suppressed during normal use |
| `MAX_TOTAL_DETECTIONS` | 25 | If more than 25 blobs pass all gates, treat as chaotic scene and discard. Adjust with `MAX_GLOBAL_MOTION_RATIO` |

## Exclusion Zone (Hand/Arm Rejection)

The system aggressively dilates the raw motion mask (`motion_clean`) and finds connected components. Large components become exclusion zones — any detection or track centred inside them is discarded.

| Constant | Default | Effect |
|---|---|---|
| `LARGE_MOTION_AREA` | 4000 px | Connected component area threshold to become exclusion zone. Observed: single mosquito merges to ~1700 px, hand ~16000 px after dilation |
| `LARGE_MOTION_DIM` | 110 px | Bounding box width or height threshold. Provides a shape-based check alongside area |
| `EXCLUSION_PADDING` | 20 px | Extra margin added around each exclusion zone |
| `EXCLUSION_DILATE_ITERS` | 4 | Dilation iterations used to merge nearby hand fragments before component analysis |
| `EXCLUSION_DILATE_KERNEL` | 5 px | Kernel size for exclusion dilation |

## Tracking Parameters

| Constant | Default | Effect |
|---|---|---|
| `MATCH_DISTANCE` | 90 px | Maximum inter-frame centroid distance for track association. Increase for faster mosquitoes; decrease in dense detection scenarios |
| `MAX_MISSED` | 12 frames | A track can survive this many consecutive missed frames before expiry (~0.4 s at 30 fps) |
| `MIN_HITS` | 2 | Detections required before a track is eligible for lock-on |
| `CONFIRM_FRAMES` | 1 | Additional age gate after `MIN_HITS` is reached |
| `TRAJECTORY_WINDOW` | 10 | Position history length for path/direction analysis |
| `MIN_PATH_LENGTH` | 4.0 px | Minimum accumulated distance across trajectory window. Prevents static edge flicker from confirming |
| `MIN_DIR_CHANGES` | 0 | Minimum direction changes required. Set to 0 to allow straight-flying insects. Increase to 1–2 to require erratic motion characteristic of mosquitoes |

## Servo Tracking (Laser Lock)

| Constant | Default | Effect |
|---|---|---|
| `KP_X` | 0.040 | Proportional gain for pan axis. `target_pan_deg -= error_x * KP_X`. Increase for faster tracking, decrease if oscillation observed |
| `KP_Y` | 0.040 | Proportional gain for tilt axis |
| `smooth_factor` | 0.22 | Servo lerp factor in `motor_smooth_thread`. 1.0 = instant (no smoothing), 0.05 = very slow |

## Symptom → Likely Fix

| Symptom | Try |
|---|---|
| Too many false positives (static objects detected) | Raise `MOTION_THRESHOLD`, raise `MIN_MOTION_SCORE`, raise `MIN_PATH_LENGTH` |
| Mosquito not detected | Lower `MIN_AREA`, lower `BLACK_THRESHOLD`, lower `MIN_DARK_SCORE`, lower `EXPOSURE_TIME_US` |
| Hand triggers detection | Lower `LARGE_MOTION_AREA`, raise `MAX_GLOBAL_MOTION_RATIO` (more aggressive suppression) |
| Tracks die too quickly | Raise `MAX_MISSED`, raise `MATCH_DISTANCE` |
| Servo oscillates around target | Lower `KP_X`/`KP_Y`, lower `smooth_factor` |
| Servo too slow to track | Raise `smooth_factor`, raise `KP_X`/`KP_Y` |
| High CPU / low FPS | Lower `PROCESS_W/H`, raise `TARGET_FPS` limit, lower `STREAM_QUALITY` |
