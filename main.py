import os
import logging
from picamera2 import Picamera2
from gpiozero import Button, LED
from time import sleep
import subprocess

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

def set_shutter_speed(speed):
    logging.info(f"Setting shutter speed to {speed}")
    cam.set_controls({"ExposureTime": speed})

# --- Button setup ---
BUTTON_PIN = 17  # Change as needed
button = Button(BUTTON_PIN)

def on_button_press():
    subprocess.call(["raspi-gpio", "set", "47", "dh"])
    logging.info("Button pressed, releasing shutter")
    capture_image()

button.when_pressed = on_button_press

# --- Camera initialization ---
cam = Picamera2()
cam.create_still_configuration()
cam.still_configuration.size = (2028, 1520)
cam.start("still")
set_shutter_speed(int(10e5 / 60))  # Default to 1/60

logging.info("Camera and button initialized. Waiting for button press...")

try:
    while True:
        sleep(0.1)
except KeyboardInterrupt:
    logging.info("Exiting...")