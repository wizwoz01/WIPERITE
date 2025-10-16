#!/usr/bin/env bash
set -euo pipefail

BOARD_USER="${BOARD_USER:-wiperite}"
BOARD_HOST="${BOARD_HOST:-192.168.191.114}"
BOARD_PORT="${BOARD_PORT:-22}"
REMOTE_DIR="${REMOTE_DIR:-/tmp/fw}"
ELF="${1:-build/my_firmware.strip.elf}"

echo "[deploy] to ${BOARD_USER}@${BOARD_HOST}:${REMOTE_DIR}"
ssh -p "$BOARD_PORT" "$BOARD_USER@$BOARD_HOST" "mkdir -p $REMOTE_DIR"
scp -P "$BOARD_PORT" "$ELF" "$BOARD_USER@$BOARD_HOST:$REMOTE_DIR/my_firmware.elf"
echo "[deploy] done"
