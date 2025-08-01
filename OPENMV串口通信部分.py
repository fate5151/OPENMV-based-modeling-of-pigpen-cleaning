import sensor, image, time,math,pyb
from pyb import UART,LED
import json
import ustruct

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.VGA)
sensor.skip_frames(time = 2000)
sensor.set_auto_gain(False) # must be turned off for color tracking
sensor.set_auto_whitebal(False) # must be turned off for color tracking
red_threshold_01=(10, 100, 127, 32, -43, 67)
clock = time.clock()

uart = UART(1,115200)   #定义串口1变量
uart.init(115200, bits=8, parity=None, stop=1) # init with given parameters

def find_max(blobs): #定义寻找色块面积最大的函数
    max_blob = None
    max_size = 0
    for blob in blobs:
        area = blob[2] * blob[3]
        if area > max_size:
            max_blob = blob
            max_size = area
    return max_blob


def sending_data(cx,cy,cw,ch):
    global uart;
    #frame=[0x2C,18,cx%0xff,int(cx/0xff),cy%0xff,int(cy/0xff),0x5B];
    #data = bytearray(frame)
    data = ustruct.pack("<bbhhhhb",      #格式为俩个字符俩个短整型(2字节)
                   0x2C,                      #帧头1
                   0x12,                      #帧头2
                   int(cx), # up sample by 4   #数据1
                   int(cy), # up sample by 4    #数据2
                   int(cw), # up sample by 4    #数据1
                   int(ch), # up sample by 4    #数据2
                   0x5B)
    uart.write(data);   #必须要传入一个字节数组


while(True):
    clock.tick()
    img = sensor.snapshot()
    blobs = img.find_blobs([red_threshold_01])
    max_b = find_max(blobs)
    cx=0;cy=0;
    if blobs:
            #如果找到了目标颜色
            cx=max_b[5]
            cy=max_b[6]
            cw=max_b[2]
            ch=max_b[3]
            img.draw_rectangle(max_b[0:4]) # rect
            img.draw_cross(max_b[5], max_b[6]) # cx, cy
            FH = bytearray([0x2C,0x12,cx,cy,cw,ch,0x5B])
            FH1 = bytearray([cx,cy,cw,ch])
            FH2 = bytearray([cx%0xff,int(cx/0xff)])

            sending_data(cx,cy,cw,ch)
            uart.write(FH)
            data1 = ustruct.pack("<bbhhhhb", 0x2C, 0x12, cx, cy, cw, ch, 0x5B)
            uart.write(data1)
            print(f"FH:{FH}")
            print(f"FH1:{FH1}")
            print(f"FH2:{FH2}")
            print(f"cx:{cx},cy:{cy},cw:{cw},ch:{ch}")
