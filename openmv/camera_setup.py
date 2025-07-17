import sensor
# camera_setup.py


# 初始化摄像头传感器
sensor.reset()                         # 重置并初始化摄像头传感器
sensor.set_pixformat(sensor.RGB565)    # 设置像素格式为RGB565（每个像素16位）

sensor.set_framesize(sensor.QVGA)      # 设置帧尺寸为QVGA（320x240分辨率）
#sensor.set_windowing((265, 240))       # 设置240x240的采集窗口（中心裁剪）
sensor.skip_frames(time=3000)          # 等待3秒让相机自动调整（自动曝光/白平衡）
