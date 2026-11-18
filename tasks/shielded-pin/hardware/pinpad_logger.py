"""MicroPython for a Raspberry Pi Pico: the PIN pad's key logger for real trials.

Wiring: each of the 15 Choc switches between a GPIO and GND (GP0..GP14 in the order of KEYS below); the Pico's
internal pull-ups, so a key reads 0 when down. Copy this file to the Pico as main.py.

Every key change goes out over USB serial as one line: "<ms since boot>,<key>,<down|up>". A change counts once it has
held for DEBOUNCE_MS. record_trial.py on the host timestamps each line on arrival and keeps the camera's frames on the
same clock.
"""

import time

from machine import Pin

KEYS = ("1", "2", "3", "4", "5", "6", "7", "8", "9", "star", "0", "hash", "red", "yellow", "green")
DEBOUNCE_MS = 5

pins = [Pin(i, Pin.IN, Pin.PULL_UP) for i in range(len(KEYS))]
state = [1] * len(KEYS)  # 1 = up
since = [0] * len(KEYS)

while True:
    now = time.ticks_ms()
    for i, p in enumerate(pins):
        v = p.value()
        if v == state[i]:
            since[i] = now
        elif time.ticks_diff(now, since[i]) >= DEBOUNCE_MS:
            state[i] = v
            print("%d,%s,%s" % (now, KEYS[i], "up" if v else "down"))
    time.sleep_ms(1)
