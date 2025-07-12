# Untitled - By: Lenovo - Thu Jul 3 2025

import sensor
import time

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)

clock = time.clock()

while True:
    clock.tick()
    img = sensor.snapshot().lens_corr(1.8)

    keypoints = img.find_keypoints(max_keypoints=100, threshold=10)
    if keypoints:
        img.draw_keypoints(keypoints, size=10, color=(255, 0, 0))

    print(clock.fps())
