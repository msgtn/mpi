import asyncio
import os
import logging
from picamera2 import Picamera2
from libcamera import controls
from gpiozero import Button, LED
import time
from time import sleep
import subprocess

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
# RESOLUTION = (4624, 3472)
# RESOLUTION = (2828, 2120)
STILL_CONFIG_DICT = {
    "size": RESOLUTION,
}
still_config = cam.create_still_configuration(*[STILL_CONFIG_DICT] * 3, buffer_count=2, queue=False)
cam.still_configuration.size = RESOLUTION
cam.configure(still_config)
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
    }
)

button.when_pressed = on_button_press

def capture_buffer():
    global still_config
    (buffer,), metadata = Picamera2.capture_buffers(["main"])
    img = Picamera2.helpers.make_image(buffer, still_config["main"])
    filename = f"picam_{get_file_number(TAPES_DIR)}.jpg"
    path = os.path.join(TAPES_DIR, filename)
    Picamera2.helpers.save(img, metadata, path)

logging.info("Camera and button initialized. Waiting for button press...")


async def main_loop():
    try:
        while True:
            await asyncio.sleep(0.01)
    except KeyboardInterrupt:
        logging.info("Exiting...")


if __name__ == "__main__":
    asyncio.run(main_loop())
