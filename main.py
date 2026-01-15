import asyncio
import logging
import os
import subprocess
import time
from pprint import *
from time import sleep

from gpiozero import LED, Button
from libcamera import controls
from picamera2 import Picamera2

# Turn off screen
subprocess.call(["raspi-gpio", "set", "24", "op"])
subprocess.call(["raspi-gpio", "set", "24", "dl"])

# --- Camera setup ---
TAPES_DIR = os.path.expanduser("~/tapes")
os.makedirs(TAPES_DIR, exist_ok=True)


def get_file_number(save_dir):
    return len([f for f in os.listdir(save_dir) if f.endswith(".jpg")])


def capture_image():
    filename = f"picam_{get_file_number(TAPES_DIR)}.jpg"
    path = os.path.join(TAPES_DIR, filename)
    cam.capture_file(path)
    logging.info(f"Captured image: {path}")


def set_shutter_speed(speed: int):
    logging.info(f"Setting shutter speed to {speed}")
    cam.set_controls({"ExposureTime": int(speed)})


# --- Button setup ---
BUTTON_PIN = 23  # Change as needed
button = Button(BUTTON_PIN)


LAST_PRESSED = time.time()


def on_button_press():
    global LAST_PRESSED
    now = time.time()
    if now - LAST_PRESSED < 1:
        return
    LAST_PRESSED = now
    print(f"{LAST_PRESSED=}")
    subprocess.call(["raspi-gpio", "set", "47", "dh"])
    logging.info("Button pressed, releasing shutter")
    # capture_image()
    capture_buffer()


# --- Camera initialization ---o
Picamera2.set_logging(Picamera2.DEBUG)
cam = Picamera2()
RESOLUTION = (2400, 1800)
RESOLUTION = (2312, 1736)
# RESOLUTION = (4624, 3472)
# RESOLUTION = (2828, 2120)
STILL_CONFIG_DICT = {
    "size": RESOLUTION,
}
still_config = cam.create_still_configuration(
    main=STILL_CONFIG_DICT,
    raw=STILL_CONFIG_DICT,
    # buffer_count=2,
    buffer_count=4,
    queue=False,
)
cam.still_configuration.size = RESOLUTION
cam.configure(still_config)

cam.set_controls(
    {
        "AfMode": controls.AfModeEnum.Manual,
        "AeEnable": False,
        "AeFlickerMode": controls.AeFlickerModeEnum.Off,
        "ExposureTime": int(1e6 / 30),
        # "AnalogueGain":1.0,
        "AnalogueGain": 4.0,
        # https://docs.arducam.com/Raspberry-Pi-Camera/Native-camera/64MP-OV64A40/#frame-rate-resolution
        "FrameRate": 25,
    }
)
# pprint(cam.sensor_modes)
cam.start("still", show_preview=False)
set_shutter_speed(1e6 / 30)

cam.set_controls(
    {
        "AfMode": controls.AfModeEnum.Manual,
        "AeEnable": False,
        "AeFlickerMode": controls.AeFlickerModeEnum.Off,
        "ExposureTime": int(1e6 / 30),
        # "AnalogueGain":1.0,
        "AnalogueGain": 4.0,
        "FrameRate": 50,
    }
)

button.when_pressed = on_button_press


def capture_buffer():
    global still_config
    global cam

    (buffer,), metadata = cam.capture_buffers(["main"])
    img = cam.helpers.make_image(buffer, still_config["main"])
    filename = f"picam_{get_file_number(TAPES_DIR)}.jpg"
    path = os.path.join(TAPES_DIR, filename)
    cam.helpers.save(img, metadata, path)
    print(f"{path=}")


logging.info("Camera and button initialized. Waiting for button press...")


async def main_loop():
    try:
        while True:
            await asyncio.sleep(0.01)
    except KeyboardInterrupt:
        logging.info("Exiting...")


if __name__ == "__main__":
    asyncio.run(main_loop())
