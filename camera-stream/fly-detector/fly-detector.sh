#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PID_FILE="$SCRIPT_DIR/fly-detector.pid"
LOG_FILE="$SCRIPT_DIR/fly-detector.log"
ARGS_FILE="$SCRIPT_DIR/fly-detector.args"

is_running() {
  if [ -f "$PID_FILE" ]; then
    PID=$(cat "$PID_FILE")
    if ps -p "$PID" > /dev/null 2>&1; then
      return 0
    fi
  fi
  return 1
}

start_detector() {
  if is_running; then
    echo "Detector already running. PID: $(cat "$PID_FILE")"
    return 0
  fi

  printf '%s\n' "$@" > "$ARGS_FILE"
  nohup python3 "$SCRIPT_DIR/fly_detector.py" --headless "$@" > "$LOG_FILE" 2>&1 &
  echo $! > "$PID_FILE"
  sleep 2

  if is_running; then
    echo "Detector started. PID: $(cat "$PID_FILE")"
  else
    echo "Failed to start detector."
    exit 1
  fi
}

stop_detector() {
  if is_running; then
    kill "$(cat "$PID_FILE")" 2>/dev/null
    sleep 1
    rm -f "$PID_FILE"
    echo "Detector stopped."
  else
    rm -f "$PID_FILE"
    echo "Detector is not running."
  fi
}

status_detector() {
  if is_running; then
    echo "Detector running. PID: $(cat "$PID_FILE")"
    ps -p "$(cat "$PID_FILE")" -o pid=,etime=,cmd=
    if [ -f "$ARGS_FILE" ]; then
      echo "Args:"
      tr '\n' ' ' < "$ARGS_FILE"
      echo
    fi
  else
    echo "Detector stopped."
  fi
}

logs_detector() {
  if [ -f "$LOG_FILE" ]; then
    tail -n 50 "$LOG_FILE"
  else
    echo "No log file yet."
  fi
}

case "$1" in
  start)
    shift
    start_detector "$@"
    ;;
  stop)
    stop_detector
    ;;
  restart)
    shift
    stop_detector
    start_detector "$@"
    ;;
  status)
    status_detector
    ;;
  logs)
    logs_detector
    ;;
  *)
    echo "Usage: $0 {start [detector-args...]|stop|restart [detector-args...]|status|logs}"
    exit 1
    ;;
esac
