#!/usr/bin/env bash
BOARD_USER="${BOARD_USER:-wiperite}"
BOARD_HOST="${BOARD_HOST:-<BOARD-IP>}"
BOARD_PORT="${BOARD_PORT:-22}"
ssh -p "$BOARD_PORT" "$BOARD_USER@$BOARD_HOST" "killall gdbserver 2>/dev/null || true"
