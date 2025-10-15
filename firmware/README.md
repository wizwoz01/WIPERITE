# Firmware scaffold

Cross-compile on Windows (MSYS2), deploy to the board, debug via gdbserver.

## Prerequisites (host)
- MSYS2 MinGW x64 shell
- ARM Linux toolchain on PATH (e.g., arm-linux-gnueabihf-gcc)
- gdb-multiarch at C:\msys64\mingw64\bin\gdb-multiarch.exe

## Quick start
- Build symbols: make
- Deploy to target: make deploy
- Start gdbserver: make gdbserver
- In VS Code, run the "Attach: gdbserver (Zynq)" launch

## Makefile knobs
- BOARD_USER (default: wiperite)
- BOARD_HOST (default: 192.168.191.114)
- BOARD_PORT (default: 22)
- PORT       (default: 3333)
- REMOTE_DIR (default: /tmp/fw)

Override at call time:
- make PORT=4444 deploy
- make PORT=4444 gdbserver

## Symbol handling
- Host keeps build/my_firmware.elf with full symbols
- Target gets build/my_firmware.strip.elf (copied as .../my_firmware.elf)
- Debug link associates stripped ELF with build/my_firmware.debug

## Troubleshooting
- If port 3333 is in use: make PORT=4444 gdbserver and update VS Code address.
- Ensure SSH keys or password auth works to the target.
- Verify arm-linux-gnueabihf-gcc is on PATH in your shell used by VS Code tasks.
