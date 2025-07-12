import sensor, image ,time,display



# 初始化摄像头
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)  # 320x240
sensor.skip_frames(time=2000)
lcd = display.SPIDisplay()

# 目标分辨率
TARGET_W = 128
TARGET_H = 160

# 计算缩放参数
scale_x = TARGET_W / 320
scale_y = TARGET_H / 240
scale_ratio = min(scale_x, scale_y)  # 保持宽高比的最小比例

# 主循环
clock = time.clock()
while True:
    clock.tick()

    # 捕获图像
    img = sensor.snapshot()

    # 单行显示解决方案 - 自动缩放和居中
    lcd.write(img, x_scale=scale_x, y_scale=scale_y,  )

    # 打印帧率
    print(clock.fps())
