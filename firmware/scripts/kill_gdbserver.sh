#!/usr/bin/env bash
set -euo pipefail

BOARD_USER="${BOARD_USER:-wiperite}"
BOARD_HOST="${BOARD_HOST:-192.168.191.114}"
BOARD_PORT="${BOARD_PORT:-22}"

echo "[kill] gdbserver on ${BOARD_HOST}"
ssh -p "$BOARD_PORT" "$BOARD_USER@$BOARD_HOST" "killall gdbserver 2>/dev/null || true"
echo "[kill] done"
