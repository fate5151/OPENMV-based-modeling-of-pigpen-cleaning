import sensor, image, time

# -------------------- 摄像头初始化 --------------------
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)  # QVGA分辨率
sensor.skip_frames(time=2000)
sensor.set_auto_whitebal(False)
clock = time.clock()

# -------------------- ROI设置函数 --------------------
def update_roi():
    global pig_roi, center_roi
    IMG_WIDTH = int(sensor.width() * (3 / 4))
    IMG_HEIGHT = int(sensor.height() * (3 / 4))
    x_offset = int((sensor.width() - IMG_WIDTH) / 2)
    y_offset = int((sensor.height() - IMG_HEIGHT) / 2)
    pig_roi = (x_offset, y_offset, IMG_WIDTH, IMG_HEIGHT)

    roi_size = int(sensor.width() * (1 / 11))
    center_x = int((sensor.width() - roi_size) / 2)
    center_y = int((sensor.height() - roi_size) / 2)
    center_roi = (center_x, center_y, roi_size, roi_size)

update_roi()  # 初始化ROI

# -------------------- 颜色阈值设置 --------------------
# 根据需要调整阈值，以下为示例阈值
shift_threshold = (28, 77, -13, 59, 31, 127)

# -------------------- 主循环 --------------------
while True:
    clock.tick()
    img = sensor.snapshot()

    # 在设定的ROI区域内查找所有符合颜色阈值的色块
    blobs = img.find_blobs([shift_threshold], merge=True)

    # 绘制ROI区域边框（绿色）
    # 绘制中心区域边框（红色）


    # 对检测到的每个色块绘制矩形框和十字标记中心点
    if blobs:
        for blob in blobs:
            img.draw_rectangle(blob.rect(), color=(255, 0, 0))
            img.draw_cross(blob.cx(), blob.cy(), color=(0, 255, 0))
    else:
        print("没有检测到目标")
