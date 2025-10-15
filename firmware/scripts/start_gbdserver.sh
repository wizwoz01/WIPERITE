#!/usr/bin/env bash
set -euo pipefail
BOARD_USER="${BOARD_USER:-wiperite}"
BOARD_HOST="${BOARD_HOST:-<BOARD-IP>}"
BOARD_PORT="${BOARD_PORT:-22}"
REMOTE_DIR="${REMOTE_DIR:-/tmp/fw}"
PORT="${PORT:-3333}"

ssh -p "$BOARD_PORT" "$BOARD_USER@$BOARD_HOST" "\
  killall gdbserver 2>/dev/null || true; \
  /usr/local/bin/gdbserver --once :$PORT $REMOTE_DIR/my_firmware.elf"
