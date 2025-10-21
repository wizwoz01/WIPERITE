# WIPERITE Base Station (Zynq-7000 / PetaLinux)

This is a simple TCP client that connects to the TM4C/CC3100 robot server (from `main.c`) on port 5000 and sends single-letter commands.

## Features
- Connect by IP and port (default 5000)
- Interactive control with WASD and Arrow keys
- One-shot mode to send a command string and exit (`-c`)
- Prints any responses from the server (e.g., greeting or "Bye")

## Key mappings
- `w` or Up Arrow -> `F` (Forward)
- `s` or Down Arrow -> `B` (Back)
- `a` or Left Arrow -> `L` (Left)
- `d` or Right Arrow -> `R` (Right)
- `space` -> `S` (Stop)
- `u` -> `U` (Speed up / lift)
- `j` -> `D` (Slow down / lower)
- `8` -> `8` (custom)
- `c` -> `C` (custom)
- `z` -> `Z` (custom)
- `q` -> `Q` (Quit)

These map directly to what the TM4C server expects in your `main.c`.

## Build

On the Zynq device (PetaLinux) or in the PetaLinux SDK environment:

```sh
cd basestation
make
```

- If cross-compiling with an SDK toolchain, set `CROSS_COMPILE` (e.g., `export CROSS_COMPILE=arm-linux-gnueabihf-`).
- Otherwise, building directly on the Zynq target uses the system `gcc`.

This produces the `wiperite_client` binary.

## Run
1. Power on the TM4C robot and ensure it connects to your WiFi AP (see LCD/UART for IP and "Connected").
2. On the Zynq base station, run:

```sh
./wiperite_client <tm4c_ip> [port]
```

- Example: `./wiperite_client 192.168.1.47`
- Default port is 5000 (matches TM4C `LISTEN_PORT`).

### One-shot commands
Send a sequence, then exit:

```sh
./wiperite_client 192.168.1.47 -c "FFFS"
```

### Interactive mode
- Use WASD or Arrow keys to drive; space to stop; `q` to quit.
- The client will send `Q` before closing.

## Networking notes
- The Zynq and TM4C must be on the same network (same SSID/subnet).
- The TM4C prints its IP on the LCD/UART when it acquires DHCP:
  - Example shown in your `SimpleLinkNetAppEventHandler`.

## Troubleshooting
- If connection fails: verify IP, port (5000), and that the TM4C shows "Listening on TCP port 5000" over UART.
- If keystrokes don’t move the robot: ensure your terminal has focus and supports arrow keys; try WASD.
- If no greeting appears: that’s okay; the client still sends commands. The TM4C sends a welcome string on connect.
