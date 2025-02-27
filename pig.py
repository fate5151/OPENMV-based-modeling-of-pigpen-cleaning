# Untitled - By: Lenovo - Fri Nov 15 2024

import sensor
import time
from pyb import Pin

p_out = Pin('P7', Pin.OUT_PP)#设置p_out为输出引脚
p_out.high()#设置p_out引脚为高

T0 = 0.01  # 清洁度阈值
threshold=(18, 62, -32, 6, 26, 51)
threshold1 =(0, 9, -128, 127, -128, 127)

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.HD)
sensor.skip_frames(time=2000)

clock = time.clock()

while True:
    clock.tick()
    img = sensor.snapshot()

    img=img.binary([threshold])
    img=img.blur(2)

    # 查找图像中符合白色阈值的区域
    blobs = img.find_blobs([threshold1],invert=True)
    white_pixels = 0
    for blob in blobs:
        white_pixels += blob.pixels()  # 每个blob的像素点数

    # 计算粪尿区域占比
    #white_pixels =img.get_statistics().mean()  # 获取白色像素平均值

    total_pixels = 1280 * 720  # 图像总像素数
    ratio = white_pixels / total_pixels  # 计算白色像素占比
    # 清洁度判断
    cleanliness_status = "Pigsty Clean" if ratio < T0 else "Pigsty Dirty"

    if cleanliness_status == "Pigsty Dirty":
        p_out.low()#设置p_out引脚为低
    else:
        p_out.high()#设置p_out引脚为高




    # 在原始图像上绘制结果
    # x, y, w, h = 100, 100, 400, 300  # 根据实际调整
    # roi = (x, y, w, h)
    # img.draw_rectangle(roi, color=(255, 0, 0))
    img.draw_string(10, 10, cleanliness_status, color=(255, 0, 0), scale=5)
    img.draw_string(10, 50, str(white_pixels), color=(255, 0, 0), scale=5)
    img.draw_string(10, 90, str(ratio), color=(255, 0, 0), scale=5)


    # 显示帧率和结果
    print("FPS:", clock.fps(), "Status:", cleanliness_status)
    print("Cleanliness Ratio:", ratio)
    print("White Pixels:", white_pixels)
    print(clock.fps())
    time.sleep(1)
