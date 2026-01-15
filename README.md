# PiCam Capture

A Raspberry Pi camera capture application that takes photos on button press.

## Dependencies

Install the required libraries on your Raspberry Pi:

```bash
sudo apt update
sudo apt install -y cmake build-essential libcamera-dev libgpiod-dev libjpeg-dev libturbojpeg0-dev
```

## Build

```bash
mkdir -p build
cd build
cmake ..
make
```

## Run

```bash
LIBCAMERA_LOG_LEVELS=V4L2:FATAL ./build/picam-capture
```

The application will:
- Turn off the screen (GPIO 24)
- Initialize the camera at 2312x1736 resolution
- Monitor GPIO 23 for button presses
- Save captured images to `~/tapes/` as `picam_N.jpg`

## Hardware Setup

- **Button**: Connect to GPIO 23 (active low with pull-up)
- **Screen control**: GPIO 24
- **LED indicator**: GPIO 47

## Notes

- Images are saved as JPEG files (quality 90) with automatic YUV420/NV12/RGB conversion
- Supported pixel formats: YUV420, NV12, RGB888, BGR888
- The camera runs in manual exposure mode with:
  - Exposure time: 1/30s
  - Analogue gain: 4.0
  - Auto-exposure disabled

## TODO
- [ ] implement shutter speed selection
- [ ] startup service, and maybe another process to kill/restart manually with a jumper to one of the currently unused pins
- [ ] design and print focus tab
