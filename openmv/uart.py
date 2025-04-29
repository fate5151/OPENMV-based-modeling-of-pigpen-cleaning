import sensor, image, time
import json
from machine import UART
#from pyb import UART
# For color tracking to work really well you should ideally be in a very, very,
# very, controlled enviroment where the lighting is constant...
yellow_threshold   = (50, 68, 0, 80, 24, 84)
# You may need to tweak the above settings for tracking green things...
# Select an area in the Framebuffer to copy the color settings.

sensor.reset() # Initialize the camera sensor.
sensor.set_pixformat(sensor.RGB565) # use RGB565.
sensor.set_framesize(sensor.QQVGA) # use QQVGA for speed.
sensor.skip_frames(10) # Let new settings take affect.
sensor.set_auto_whitebal(False) # turn this off.
clock = time.clock() # Tracks FPS.

# OpenMV4 H7 Plus, OpenMV4 H7, OpenMV3 M7, OpenMV2 M4 的UART(3)是P4-TX P5-RX
uart = UART(3, 9600)   #OpenMV RT 注释掉这一行，用下一行UART(1)
#uart = UART(1, 115200)  #OpenMV RT 用UART(1)这行，注释掉上一行UART(3)
# OpenMV RT 只有串口UART(1)，对应P4-TX P5-RX; OpenMV4 H7 Plus, OpenMV4 H7, OpenMV3 M7 的UART(1)是P0-RX P1-TX

while(True):
    clock.tick() # Track elapsed milliseconds between snapshots().
    img = sensor.snapshot() # Take a picture and return the image.

    blobs = img.find_blobs([yellow_threshold])
    if blobs:
        #print('sum : %d'% len(blobs))
        data=[]
        for b in blobs:
            # Draw a rect around the blob.
            img.draw_rectangle(b.rect()) # rect
            img.draw_cross(b.cx(), b.cy()) # cx, cy
            data.append((b.cx(),b.cy()))

        #{(1,22),(-3,33),(22222,0),(9999,12),(0,0)}
        data_out = json.dumps(set(data))
        uart.write(data_out +'\n')
        print('you send:',data_out)
    else:
        print("not found!")
