# Untitled - By: Lenovo - Sat Jul 5 2025

import sensor
import time

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)

clock = time.clock()

while True:
    clock.tick()
    img = sensor.snapshot()
    print(clock.fps())
