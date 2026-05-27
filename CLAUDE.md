# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A Raspberry Pi camera application (`picam-capture`) that captures stills on button press. The primary implementation is `main.cpp` (C++17); `main.py` is an older Python prototype that is no longer the active implementation.

## Build & deploy

All development happens on a remote Pi at `dummy@192.168.90.1`. The normal workflow is:

```bash
just deploy        # rsync source → Pi, cmake+make, restart service
just setup         # first-time: install systemd service on Pi
just mount         # sshfs ~/tapes from Pi to ./mpi-mount/
just sync          # rsync mpi-mount → ~/Pictures/mpi/
```

To build locally on the Pi (requires libcamera, libgpiod, libturbojpeg, libexiv2, lgpio):

```bash
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release && make -j$(nproc)
```

To run:
```bash
LIBCAMERA_LOG_LEVELS=V4L2:FATAL ./build/picam-capture
```

The app runs as a systemd service (`mpi.service`); `sudo systemctl restart mpi` to cycle it.

## Architecture

All logic lives in `main.cpp`. Key flows:

**Capture pipeline** (async, two threads):
1. `buttonThread` — monitors GPIO via `libgpiod` for falling-edge events on all input pins. On shutter press it sets `captureCountdown = 3`.
2. `requestComplete` callback (libcamera signal, called from camera thread) — runs every frame. Decrements `captureCountdown`; when it reaches 1 (the last skip frame) it fires the flash (GPIO 25 LOW). When it reaches 0 it mmaps the frame buffer, copies it into a `CaptureJob`, and pushes it onto `captureQueue`.
3. `encoderThreadFunc` — dequeues `CaptureJob`s, encodes YUV420→JPEG via turbojpeg, writes to `~/tapes/mpi_<timestamp>.jpg`, and writes EXIF via exiv2.

**GPIO pin assignments:**
| Pin | Role |
|-----|------|
| 23  | Shutter button (input, pull-up) |
| 24  | LCD backlight (output) |
| 25  | Flash sync (output — LOW during exposure, HIGH otherwise) |
| 12  | LED indicator (output) |
| 19/5/6/26 | Exposure buttons: 1/1000, 1/250, 1/60, 1/15 s |
| 16  | Show last photo button |
| 20  | Gain cycle button (currently disabled) |

**Flash sync timing:** `captureCountdown` starts at 3. Pin 25 goes LOW when countdown drops from 2→1 (the frame *before* the capture frame is re-queued), so the flash fires during the capture frame's exposure. Pin 25 returns HIGH once that frame's buffer is read in `requestComplete`.

**LCD HAT:** The bundled `1.3inch_LCD_HAT_code/c/` library (DEV_Config, LCD_1in3, GUI_Paint) drives a 240×240 SPI display. It is only activated during `showMostRecentPhoto()` — the screen is otherwise off (GPIO 24 LOW).

**Shutter speed persistence:** Last-set exposure time is cached in `~/.mpi_shutter_speed` and reloaded on startup.

**Resolution:** Configured at compile time via `WIDTH`/`HEIGHT` constants (currently 4624×3472). Images are saved to `~/tapes/`.
