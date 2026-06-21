#!/usr/bin/env python3
import argparse
import math
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
from picamera2 import Picamera2


@dataclass
class Detection:
    bbox: Tuple[int, int, int, int]
    centroid: Tuple[int, int]
    area: float
    motion_score: float
    dark_score: float


@dataclass
class Track:
    track_id: int
    centroid: Tuple[float, float]
    bbox: Tuple[int, int, int, int]
    velocity: Tuple[float, float]
    last_seen: int
    misses: int = 0
    hits: int = 1
    age: int = 1
    first_seen: int = 0
    motion_score: float = 0.0
    dark_score: float = 0.0

    def predicted_centroid(self) -> Tuple[float, float]:
        return (self.centroid[0] + self.velocity[0], self.centroid[1] + self.velocity[1])

    def speed(self) -> float:
        return math.hypot(self.velocity[0], self.velocity[1])


class MotionDarkTracker:
    def __init__(self, max_distance: float, max_missed: int, min_hits: int, min_track_speed: float, confirm_frames: int) -> None:
        self.max_distance = max_distance
        self.max_missed = max_missed
        self.min_hits = min_hits
        self.min_track_speed = min_track_speed
        self.confirm_frames = confirm_frames
        self.next_id = 1
        self.tracks: Dict[int, Track] = {}

    def update(self, detections: List[Detection], frame_index: int) -> List[Track]:
        unmatched_tracks = set(self.tracks.keys())
        unmatched_detections = set(range(len(detections)))

        candidate_pairs: List[Tuple[float, int, int]] = []
        for det_index, detection in enumerate(detections):
            for track_id, track in self.tracks.items():
                predicted = track.predicted_centroid()
                distance = math.dist(predicted, detection.centroid)
                if distance <= self.max_distance:
                    quality_bonus = (detection.dark_score * 0.15) + (detection.motion_score * 12.0)
                    candidate_pairs.append((distance - quality_bonus, track_id, det_index))

        candidate_pairs.sort(key=lambda item: item[0])

        for _, track_id, det_index in candidate_pairs:
            if track_id not in unmatched_tracks or det_index not in unmatched_detections:
                continue
            unmatched_tracks.remove(track_id)
            unmatched_detections.remove(det_index)
            detection = detections[det_index]
            track = self.tracks[track_id]
            new_vx = detection.centroid[0] - track.centroid[0]
            new_vy = detection.centroid[1] - track.centroid[1]
            track.velocity = (
                0.45 * track.velocity[0] + 0.55 * new_vx,
                0.45 * track.velocity[1] + 0.55 * new_vy,
            )
            track.centroid = (float(detection.centroid[0]), float(detection.centroid[1]))
            track.bbox = detection.bbox
            track.last_seen = frame_index
            track.misses = 0
            track.hits += 1
            track.age += 1
            track.motion_score = detection.motion_score
            track.dark_score = detection.dark_score

        for det_index in unmatched_detections:
            detection = detections[det_index]
            self.tracks[self.next_id] = Track(
                track_id=self.next_id,
                centroid=(float(detection.centroid[0]), float(detection.centroid[1])),
                bbox=detection.bbox,
                velocity=(0.0, 0.0),
                last_seen=frame_index,
                motion_score=detection.motion_score,
                dark_score=detection.dark_score,
                first_seen=frame_index,
            )
            self.next_id += 1

        expired: List[int] = []
        for track_id in unmatched_tracks:
            track = self.tracks[track_id]
            track.misses += 1
            track.age += 1
            track.centroid = track.predicted_centroid()
            track.velocity = (track.velocity[0] * 0.7, track.velocity[1] * 0.7)
            if track.misses > self.max_missed:
                expired.append(track_id)
        for track_id in expired:
            self.tracks.pop(track_id, None)

        visible_tracks = []
        for track in self.tracks.values():
            if track.hits >= self.min_hits and track.misses <= self.max_missed and track.speed() >= self.min_track_speed and (frame_index - track.first_seen) >= self.confirm_frames:
                visible_tracks.append(track)
        visible_tracks.sort(key=lambda track: track.track_id)
        return visible_tracks


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Track moving black dots using Picamera2 + OpenCV")
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--height", type=int, default=540)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--process-width", type=int, default=640)
    parser.add_argument("--process-height", type=int, default=360)
    parser.add_argument("--blur", type=int, default=5)
    parser.add_argument("--motion-threshold", type=int, default=20)
    parser.add_argument("--motion-dilate", type=int, default=2)
    parser.add_argument("--blackhat-kernel", type=int, default=11)
    parser.add_argument("--black-threshold", type=int, default=26)
    parser.add_argument("--adaptive-block-size", type=int, default=21)
    parser.add_argument("--adaptive-c", type=int, default=9)
    parser.add_argument("--min-area", type=int, default=14)
    parser.add_argument("--max-area", type=int, default=120)
    parser.add_argument("--min-width", type=int, default=2)
    parser.add_argument("--max-width", type=int, default=20)
    parser.add_argument("--min-height", type=int, default=2)
    parser.add_argument("--max-height", type=int, default=20)
    parser.add_argument("--min-aspect", type=float, default=0.25)
    parser.add_argument("--max-aspect", type=float, default=4.0)
    parser.add_argument("--match-distance", type=float, default=34.0)
    parser.add_argument("--max-missed", type=int, default=8)
    parser.add_argument("--min-hits", type=int, default=3)
    parser.add_argument("--min-track-speed", type=float, default=1.2)
    parser.add_argument("--display-delay-ms", type=int, default=250)
    parser.add_argument("--min-motion-score", type=float, default=0.22)
    parser.add_argument("--min-dark-score", type=float, default=30.0)
    parser.add_argument("--max-local-mean", type=float, default=170.0)
    parser.add_argument("--warmup-frames", type=int, default=10)
    parser.add_argument("--roi", type=str, default="", help="x,y,w,h in processing coordinates")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--save-dir", type=Path, default=None)
    parser.add_argument("--debug-mask", action="store_true")
    parser.add_argument("--print-every", type=int, default=30)
    parser.add_argument("--stream-ip", type=str, default="")
    parser.add_argument("--mirror", action="store_true")
    parser.add_argument("--stream-port", type=int, default=6000)
    parser.add_argument("--stream-bitrate", type=str, default="2500k")
    return parser


def parse_roi(raw: str, frame_width: int, frame_height: int) -> Optional[Tuple[int, int, int, int]]:
    if not raw:
        return None
    parts = [int(part.strip()) for part in raw.split(",")]
    if len(parts) != 4:
        raise ValueError("ROI must be x,y,w,h")
    x, y, w, h = parts
    if x < 0 or y < 0 or w <= 0 or h <= 0:
        raise ValueError("ROI values must be positive")
    if x + w > frame_width or y + h > frame_height:
        raise ValueError("ROI exceeds frame bounds")
    return x, y, w, h


def ensure_odd(value: int, minimum: int = 3) -> int:
    value = max(value, minimum)
    return value if value % 2 == 1 else value + 1


def start_stream_process(args: argparse.Namespace) -> Optional[subprocess.Popen]:
    if not args.stream_ip:
        return None
    command = [
        "ffmpeg", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "bgr24",
        "-s", f"{args.width}x{args.height}", "-r", str(args.fps), "-i", "-", "-an",
        "-c:v", "libx264", "-preset", "ultrafast", "-tune", "zerolatency",
        "-pix_fmt", "yuv420p", "-b:v", args.stream_bitrate, "-f", "mpegts",
        f"udp://{args.stream_ip}:{args.stream_port}?pkt_size=1316",
    ]
    return subprocess.Popen(command, stdin=subprocess.PIPE)


def build_detection_mask(gray_roi, prev_gray_roi, args):
    blurred = cv2.GaussianBlur(gray_roi, (args.blur, args.blur), 0)
    enhanced = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(blurred)

    if prev_gray_roi is None:
        motion_mask = np.zeros_like(gray_roi)
    else:
        frame_delta = cv2.absdiff(blurred, prev_gray_roi)
        _, motion_mask = cv2.threshold(frame_delta, args.motion_threshold, 255, cv2.THRESH_BINARY)

    blackhat_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (args.blackhat_kernel, args.blackhat_kernel))
    blackhat = cv2.morphologyEx(enhanced, cv2.MORPH_BLACKHAT, blackhat_kernel)
    _, black_mask = cv2.threshold(blackhat, args.black_threshold, 255, cv2.THRESH_BINARY)

    adaptive_mask = cv2.adaptiveThreshold(
        enhanced, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV,
        args.adaptive_block_size, args.adaptive_c,
    )

    motion_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    motion_mask = cv2.dilate(motion_mask, motion_kernel, iterations=args.motion_dilate)
    dark_mask = cv2.bitwise_and(black_mask, adaptive_mask)
    combined = cv2.bitwise_and(dark_mask, motion_mask)
    combined = cv2.morphologyEx(combined, cv2.MORPH_OPEN, motion_kernel, iterations=1)
    combined = cv2.dilate(combined, motion_kernel, iterations=1)
    return blurred, enhanced, motion_mask, blackhat, dark_mask, combined


def main() -> int:
    args = build_parser().parse_args()
    args.blur = ensure_odd(args.blur)
    args.blackhat_kernel = ensure_odd(args.blackhat_kernel, minimum=5)
    args.adaptive_block_size = ensure_odd(args.adaptive_block_size, minimum=5)
    if args.save_dir:
        args.save_dir.mkdir(parents=True, exist_ok=True)

    picam2 = Picamera2()
    config = picam2.create_video_configuration(
        main={"size": (args.width, args.height), "format": "RGB888"},
        controls={"FrameRate": args.fps}, queue=False,
    )
    picam2.configure(config)
    picam2.start()

    roi = parse_roi(args.roi, args.process_width, args.process_height)
    confirm_frames = max(0, round(args.display_delay_ms * args.fps / 1000))
    tracker = MotionDarkTracker(args.match_distance, args.max_missed, args.min_hits, args.min_track_speed, confirm_frames)
    stream_process = start_stream_process(args)
    prev_gray_roi = None
    running = True

    def stop_handler(_signum, _frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)

    frame_index = 0
    detection_total = 0
    last_print = time.time()

    try:
        while running:
            rgb_frame = picam2.capture_array("main")
            frame = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR)
            process_frame = cv2.resize(frame, (args.process_width, args.process_height), interpolation=cv2.INTER_AREA)
            gray = cv2.cvtColor(process_frame, cv2.COLOR_BGR2GRAY)

            if roi is not None:
                rx, ry, rw, rh = roi
                gray_roi = gray[ry:ry + rh, rx:rx + rw]
            else:
                rx, ry = 0, 0
                rw, rh = args.process_width, args.process_height
                gray_roi = gray

            blurred, enhanced, motion_mask, blackhat, dark_mask, combined_mask = build_detection_mask(gray_roi, prev_gray_roi, args)
            prev_gray_roi = blurred.copy()

            if frame_index < args.warmup_frames:
                frame_index += 1
                continue

            contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            detections: List[Detection] = []
            for contour in contours:
                area = cv2.contourArea(contour)
                if area < args.min_area or area > args.max_area:
                    continue
                x, y, w, h = cv2.boundingRect(contour)
                if not (args.min_width <= w <= args.max_width and args.min_height <= h <= args.max_height):
                    continue
                aspect = w / max(h, 1)
                if not (args.min_aspect <= aspect <= args.max_aspect):
                    continue

                motion_score = float(cv2.mean(motion_mask[y:y + h, x:x + w])[0]) / 255.0
                dark_score = float(cv2.mean(blackhat[y:y + h, x:x + w])[0])
                local_mean = float(cv2.mean(enhanced[y:y + h, x:x + w])[0])
                if motion_score < args.min_motion_score or dark_score < args.min_dark_score:
                    continue
                if local_mean > args.max_local_mean:
                    continue

                detections.append(Detection(
                    bbox=(x + rx, y + ry, w, h),
                    centroid=(x + rx + w // 2, y + ry + h // 2),
                    area=area,
                    motion_score=motion_score,
                    dark_score=dark_score,
                ))

            tracks = tracker.update(detections, frame_index)
            detection_total += len(tracks)

            scale_x = args.width / args.process_width
            scale_y = args.height / args.process_height

            if roi is not None:
                x1 = int(rx * scale_x)
                y1 = int(ry * scale_y)
                x2 = int((rx + rw) * scale_x)
                y2 = int((ry + rh) * scale_y)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 180, 0), 1)

            for track in tracks:
                x, y, w, h = track.bbox
                sx = int(x * scale_x)
                sy = int(y * scale_y)
                sw = max(1, int(w * scale_x))
                sh = max(1, int(h * scale_y))
                color = (0, 255, 0) if track.misses == 0 else (0, 200, 255)
                cv2.rectangle(frame, (sx, sy), (sx + sw, sy + sh), color, 2)
                label = f"ID {track.track_id} s={track.speed():.1f} d={track.dark_score:.0f}"
                cv2.putText(frame, label, (sx, max(18, sy - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1, cv2.LINE_AA)

            status = f"tracks={len(tracks)} detections={len(detections)} frame={frame_index}"
            cv2.putText(frame, status, (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 220, 255), 2, cv2.LINE_AA)

            if args.mirror:
                frame = cv2.flip(frame, 1)

            if args.debug_mask:
                debug_mask = cv2.cvtColor(combined_mask, cv2.COLOR_GRAY2BGR)
                debug_mask = cv2.resize(debug_mask, (frame.shape[1] // 3, frame.shape[0] // 3), interpolation=cv2.INTER_NEAREST)
                mh, mw = debug_mask.shape[:2]
                frame[frame.shape[0] - mh:frame.shape[0], 0:mw] = debug_mask

            if args.save_dir and tracks:
                filename = args.save_dir / f"frame_{frame_index:06d}.jpg"
                cv2.imwrite(str(filename), frame)

            if stream_process and stream_process.stdin:
                try:
                    stream_process.stdin.write(frame.tobytes())
                except BrokenPipeError:
                    print("stream pipe closed", file=sys.stderr)
                    stream_process = None

            if not args.headless:
                cv2.imshow("Fly Detector", frame)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break

            if args.print_every and frame_index % args.print_every == 0:
                now = time.time()
                elapsed = max(now - last_print, 1e-6)
                fps = args.print_every / elapsed if frame_index else 0.0
                print(f"frame={frame_index} tracks={len(tracks)} detections={len(detections)} fps={fps:.1f}")
                last_print = now

            frame_index += 1
            if args.max_frames and frame_index >= args.max_frames:
                break
    finally:
        picam2.stop()
        if stream_process and stream_process.stdin:
            stream_process.stdin.close()
        if stream_process:
            try:
                stream_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                stream_process.kill()
        cv2.destroyAllWindows()

    print(f"done frames={frame_index} total_track_hits={detection_total}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
