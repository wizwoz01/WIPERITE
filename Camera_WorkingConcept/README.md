# Board Outline + Robot Tracker

Simple OpenCV-based tool to detect quadrilateral whiteboards in a camera frame and track a colored marker (robot) on the board.

Usage examples:

Run on the default camera index 0 and look for a red marker:

```bash
python tracker.py --camera 0 --color red
```

If your camera is a V4L2 device on Petalinux, pass the device path:

```bash
python tracker.py --camera /dev/video0 --color red
```

# WIPERITE — Board Tracker (Ver.003)

Detects quadrilateral whiteboards in a camera frame and tracks a colored marker (robot). Supports V4L2/USB cameras or raw UYVY frames supplied by an FPGA via `/dev/xillybus_read_32`.

## Requirements

- Python 3.8+
- pip packages: `opencv-python`, `numpy`, `Pillow`
- `mplayer` (used by `liveview.sh` for raw UYVY preview)
- Xillybus kernel modules and device nodes (for raw FPGA capture: `/dev/xillybus_read_32`)
- Framebuffer device `/dev/fb1` when using the ST7789 framebuffer display

## Installation

Install the Python dependencies on the host or target rootfs:

```bash
pip install opencv-python numpy Pillow
```

Install `mplayer` using your distribution package manager if you intend to use `liveview.sh`.

If you plan to use FPGA/raw capture, ensure the Xillybus kernel modules are installed and the device nodes (for example `/dev/xillybus_read_32`) are present.

## Usage examples

- Run using a system camera (index 0):

```bash
python tracker.py --camera 0 --color red
```

- Read raw UYVY frames from the FPGA Xillybus device (UYVY, 2 bytes/pixel):

```bash
python tracker.py --raw-device /dev/xillybus_read_32 --width 640 --height 480 --color red
```

- The `liveview.sh` helper reads `/dev/xillybus_read_32` and pipes to `mplayer` with these exact parameters:

```bash
dd if=/dev/xillybus_read_32 bs=614400 iflag=fullblock count=1 2>/dev/null | \
	mplayer -demuxer rawvideo -rawvideo w=640:h=480:format=uyvy:size=614400 -
```

## Command-line options (defaults from `tracker.py`)

- `--camera` (default `0`) — camera index or device path
- `--raw-device` — path to raw UYVY device (e.g. `/dev/xillybus_read_32`)
- `--width` (default `640`) — frame width for raw device
- `--height` (default `480`) — frame height for raw device
- `--min-area` (default `5000`) — ignore detected board contours smaller than this area
- `--color` (choices `red,green,blue`, default `red`) — marker color to detect
- `--no-display` — disable ST7789/framebuffer or OpenCV windows
- `--debug` — enable debug logging
- `--rotation` (choices `0,90,180,270`, default `0`) — rotate output for display
- `--lcd-order` (choices `BGR,BRG`, default `BGR`) — pixel channel order expected by the LCD

## Color / HSV ranges (exact from `tracker.py`)

- Red: lower1 = [0,120,70], upper1 = [10,255,255]; lower2 = [170,120,70], upper2 = [180,255,255]
- Green: lower = [40,40,40], upper = [80,255,255]
- Blue: lower = [90,50,50], upper = [140,255,255]

Note: masks are post-processed with a (5,5) elliptical kernel using morphological open and dilation.

## Display & framebuffer notes

- `st7789.py` defaults to framebuffer device `/dev/fb1` (see `FB_DEVICE` and `fb_info`).
- `fb_draw.py` is a small demo that uses `ST7789()` and PIL to draw a loading bar and title (`WIPERITE Ver.003`).
- When using the ST7789 display, the code sends BGR numpy arrays; use `--lcd-order BRG` if your display expects a different channel ordering.

## FPGA / raw capture details

- `xillydemo.v` implements an OV7670 framegrabber and exposes captured frames through Xillybus streams (notably `/dev/xillybus_read_32`).
- Frames are expected in UYVY format (2 bytes per pixel). For a 640×480 frame the raw frame size is `640 * 480 * 2 = 614400` bytes (see `liveview.sh` and `uyvy_to_bgr`).

## Files (summary)

- `tracker.py` — main tracker application: board detection, marker detection, raw-device handling, and ST7789 integration.
- `fb_draw.py` — ST7789 demo (loading bar/title) using PIL.
- `st7789.py` — framebuffer helper class `ST7789` (interacts with `/dev/fb1`).
- `liveview.sh` — streams `/dev/xillybus_read_32` to `mplayer` as raw UYVY video.
- `xillydemo.v` — FPGA Verilog demo that implements a framegrabber and Xillybus interface.
- `tacker_backup.txt` — backup of tracker code (filename contains a typo `tacker_...`).

## Controls

- Press `q` in the OpenCV preview window to quit. The program also responds to `KeyboardInterrupt` (Ctrl+C) when running headless.

## Petalinux / deployment notes

- On Petalinux targets, either build OpenCV (and required Python packages) into the image or provide them via a compatible Python environment. Ensure the Xillybus kernel driver and framebuffer device are present if using raw capture and ST7789.

## License & Contact


---
