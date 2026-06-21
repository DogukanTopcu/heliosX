#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PID_FILE="$SCRIPT_DIR/camera-stream.pid"
LOG_FILE="$SCRIPT_DIR/camera-stream.log"
TARGET_FILE="$SCRIPT_DIR/camera-stream.target"
DEFAULT_PORT="5000"

is_running() {
  if [ -f "$PID_FILE" ]; then
    PID=$(cat "$PID_FILE")
    if ps -p "$PID" > /dev/null 2>&1; then
      return 0
    fi
  fi
  return 1
}

read_target() {
  if [ -f "$TARGET_FILE" ]; then
    . "$TARGET_FILE"
  fi
}

write_target() {
  cat > "$TARGET_FILE" <<TARGET
STREAM_IP="$1"
STREAM_PORT="$2"
TARGET
}

start_stream() {
  local stream_ip="$2"
  local stream_port="${3:-$DEFAULT_PORT}"

  if [ -z "$stream_ip" ]; then
    echo "Usage: $0 start <target-ip> [port]"
    exit 1
  fi

  if is_running; then
    echo "Stream already running. PID: $(cat "$PID_FILE")"
    read_target
    if [ -n "$STREAM_IP" ]; then
      echo "Target: udp://$STREAM_IP:$STREAM_PORT"
    fi
    return 0
  fi

  write_target "$stream_ip" "$stream_port"
  nohup rpicam-vid \
    -t 0 \
    -n \
    --inline \
    --low-latency \
    --width 1280 \
    --height 720 \
    --framerate 30 \
    --bitrate 4000000 \
    -o udp://$stream_ip:$stream_port \
    > "$LOG_FILE" 2>&1 &

  echo $! > "$PID_FILE"
  sleep 1

  if is_running; then
    echo "Stream started. PID: $(cat "$PID_FILE")"
    echo "Target: udp://$stream_ip:$stream_port"
  else
    echo "Failed to start stream."
    exit 1
  fi
}

stop_stream() {
  if is_running; then
    PID=$(cat "$PID_FILE")
    kill "$PID" 2>/dev/null
    sleep 1
    pkill -f rpicam-vid 2>/dev/null
    rm -f "$PID_FILE"
    echo "Stream stopped."
  else
    pkill -f rpicam-vid 2>/dev/null
    rm -f "$PID_FILE"
    echo "Stream is not running."
  fi
}

status_stream() {
  if is_running; then
    read_target
    echo "Stream running. PID: $(cat "$PID_FILE")"
    if [ -n "$STREAM_IP" ]; then
      echo "Target: udp://$STREAM_IP:$STREAM_PORT"
    fi
    ps -p "$(cat "$PID_FILE")" -o pid=,etime=,cmd=
  else
    echo "Stream stopped."
  fi
}

logs_stream() {
  if [ -f "$LOG_FILE" ]; then
    tail -n 50 "$LOG_FILE"
  else
    echo "No log file yet."
  fi
}

case "$1" in
  start)
    start_stream "$@"
    ;;
  stop)
    stop_stream
    ;;
  restart)
    shift
    stop_stream
    start_stream start "$@"
    ;;
  status)
    status_stream
    ;;
  logs)
    logs_stream
    ;;
  *)
    echo "Usage: $0 {start <target-ip> [port]|stop|restart <target-ip> [port]|status|logs}"
    exit 1
    ;;
esac
