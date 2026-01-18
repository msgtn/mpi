#!/usr/bin/env python3
"""Simple script to print button presses on GPIO pins 0-40 using gpiozero."""

from gpiozero import Button
from signal import pause

buttons = {}

def make_press_handler(pin):
    def on_press():
        print(f"Button {pin} pressed!")
    return on_press

def make_release_handler(pin):
    def on_release():
        print(f"Button {pin} released!")
    return on_release

print("Setting up buttons on GPIO 0-40...")

for pin in range(41):
    try:
        btn = Button(pin)
        btn.when_pressed = make_press_handler(pin)
        btn.when_released = make_release_handler(pin)
        buttons[pin] = btn
        print(f"  GPIO {pin}: OK")
    except Exception as e:
        print(f"  GPIO {pin}: skipped ({e})")

print(f"\nListening on {len(buttons)} pins...")
print("Press Ctrl+C to exit")

pause()
