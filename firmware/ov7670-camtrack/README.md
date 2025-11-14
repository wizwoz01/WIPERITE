# OV7670 Camera Tracking (Userspace, Make) — Zynq‑7000 + PetaLinux

This is a **standalone** userspace project (no Yocto integration). Build it **on your PetaLinux device**
or cross-compile on a host and copy the binary over.

It configures an **OV7670** camera via I²C (SCCB), uses an **AXI DMA S2MM** to capture one frame at a time
to a **reserved DDR buffer** via `/dev/mem`, and runs a very fast color-based centroid tracker on **RGB565**
frames. The app loops continuously and prints the centroid (x, y) each frame.

> Assumes your hardware design exposes:> - AXI DMA (S2MM) lite registers at `0x40400000` (adjustable)> - A reserved, physically contiguous **framebuffer** at `0x1F000000` (adjustable)> - OV7670 SCCB/I²C on `/dev/i2c-0` (bus adjustable), 7‑bit addr `0x21` (adjustable)> - Camera outputs **RGB565**, with HREF/VSYNC standard polarity, and PCLK feeding the DMA stream clock.

---

## Build

### On the Zynq-7000 device (recommended)
```bash
# On your board (PetaLinux shell)
cd ov7670-camtrack-make
make
```

### Cross-compile from host (optional)
```bash
# Example: aarch64 cross (edit for your toolchain)
make CROSS_COMPILE=aarch64-linux-gnu-
```

The build produces `bin/camtrack`.

---

## Run

1) Make sure your device tree reserves enough DDR and that the AXI DMA base address matches your design.   Typical device-tree reserved memory for 2× VGA frames (RGB565) is ~1.25 MiB; we allocate 4 MiB by default.

2) Plug the OV7670, provide XCLK (~24 MHz) if your module needs it, and verify `/dev/i2c-0` exists.

3) Run the app (VGA 640×480 example):
```bash
sudo ./bin/camtrack --width 640 --height 480   --fb-phys 0x1F000000 --dma-phys 0x40400000   --i2c-bus 0 --i2c-addr 0x21 --ppm /run/capture.ppm
```

4) Optional: QVGA (320×240) for higher FPS and lighter CPU:
```bash
sudo ./bin/camtrack --width 320 --height 240 --ppm /run/capture.ppm
```

**Output:** Each frame prints something like:
```
frame=42 mask=12345 centroid=(312,231) ms=23.7
```

The PPM file is a sanity snapshot you can `scp` off the board:
```bash
scp root@<board-ip>:/run/capture.ppm ./
```

---

## CLI Options

```
Usage: camtrack [options]

  --width <W>           Frame width (default 640)
  --height <H>          Frame height (default 480)
  --fb-phys <addr>      Phys address of capture buffer (default 0x1F000000)
  --dma-phys <addr>     Phys address of AXI DMA (S2MM regs base) (default 0x40400000)
  --i2c-bus <n>         I2C bus index for OV7670 (default 0)
  --i2c-addr <0xNN>     I2C 7-bit address for OV7670 (default 0x21)
  --loops <N>           Frames to capture (default 0 = infinite)
  --ppm <path>          Write last frame to PPM (optional)
  --rmin <0-31>         Red min threshold (default 21)
  --gmin <0-63>         Green min threshold (default 10)
  --gmax <0-63>         Green max threshold (default 40)
  --bmax <0-31>         Blue max threshold (default 10)
  --help                Show help
```

Threshold defaults are tuned loosely for an **orange object** under indoor lighting.
Adjust to your color.

---

## Hardware/SoC Assumptions

- AXI DMA **S2MM** is connected to the capture stream (from a CMOS→AXIS bridge) clocked by camera PCLK.
- Userspace drives DMA in **simple mode** (no SG). `tlast` from the capture block is nice-to-have.
- Frame layout is linear RGB565 (little-endian, two bytes/pixel).

If your design uses **VDMA** or a V4L2 path, this app will not work without changes; this is a
minimal DMA S2MM example intended for bare userspace bring-up.

---

## Troubleshooting

- **`DMA never completes`**: Check S2MM DMACR RS bit is set, length = W×H×2, camera PCLK present.- **`I2C write failed`**: Confirm bus number, 7‑bit addr (`0x21` typical), pull-ups present.- **`Random colors / wrong image`**: Check RGB565 byte order and TSLB/COM15 in the OV7670 table.- **`All black`**: Exposure needs light; try pointing to bright scene or adjust sensor AGC/AEC registers.- **`mmapping /dev/mem fails`**: Ensure your reserved-memory region exists and `no-map;` is set in DT.
---

## License

MIT for files in this project.


---

## OpenCV Variant

You can build an OpenCV-enabled binary `camtrack_cv` that wraps the RGB565 buffer as a `cv::Mat`,
converts to BGR, thresholds in **HSV**, and saves an annotated frame.

### Build
```bash
# On device (requires OpenCV runtime + headers) or cross-compile
make cv
# OR
make CROSS_COMPILE=aarch64-linux-gnu- cv
```

The Makefile uses `pkg-config` to find `opencv4` (or falls back to `opencv`).

### Run
```bash
sudo ./bin/camtrack_cv --width 640 --height 480 --rgb565 \
  --fb-phys 0x1F000000 --dma-phys 0x40400000 \
  --i2c-bus 0 --i2c-addr 0x21 \
  --hlo 5 --hhi 25 --slo 80 --shi 255 --vlo 80 --vhi 255 \
  --out /run/annotated.png --loops 10
```

If the colors look swapped (bluish), try `--bgr565` instead of `--rgb565`—it depends on your byte order.


### Getting OpenCV on PetaLinux
- **If you own the rootfs**: add OpenCV via `petalinux-config -c rootfs` (look for `opencv`, `opencv-dev`, `opencv-imgcodecs`).

- **Binary packages/feeds**: If your image has a package manager (rpm/ipk), install `opencv` + `opencv-dev` + `pkgconfig`.

- **Build from source** (host cross-compile): build OpenCV with modules `core,imgproc,imgcodecs` and install to a sysroot; set `PKG_CONFIG_PATH` and `CROSS_COMPILE` when building.

