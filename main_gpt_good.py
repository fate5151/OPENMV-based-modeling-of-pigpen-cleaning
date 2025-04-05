import sensor, image, time
from pyb import Servo, Pin
from pid import PID


# -------------------- 系统参数 --------------------
IMG_WIDTH = 500   # ROI 区宽度（像素）
IMG_HEIGHT = 420  # ROI 区高度（像素）
ROI = (0, 40, IMG_WIDTH, IMG_HEIGHT)  # ROI区域

# -------------------- 舵机参数 --------------------
pan_servo = Servo(1)  # 水平舵机
tilt_servo = Servo(2)  # 垂直舵机

# 舵机校准参数（脉冲宽度范围500-2500μs，误差补偿500μs）
pan_servo.calibration(500, 2500, 500)
tilt_servo.calibration(500, 2500, 500)

# 水泵控制
pump_pin_red = Pin('P4', Pin.OUT_PP)
pump_pin_black = Pin('P5', Pin.OUT_PP)
pump_pin_red.low()
pump_pin_black.low()

# -------------------- 初始化 PID 控制器 --------------------
pan_pid = PID(p=0.05, i=0,d=0.001 ,imax=90)#在线调试使用这个PID
tilt_pid = PID(p=0.05, i=0,d=0.001, imax=90)#在线调试使用这个PID

# -------------------- 角度映射函数 --------------------
def map_angles(target_x, target_y):
    """
    将目标点的图像坐标 (target_x, target_y) 映射到舵机角度。
    """

    # ---- 水平角度计算（pan）----
    # x 从 0 到 500(IMG_WIDTH)，映射到 0° ~ 180°
    pan_angle = 0 + (180 - 0) * (target_x / IMG_WIDTH)

    # ---- 垂直角度计算（tilt）----
    if target_y <= 230:
        # y 从 0 到 230(IMG_HEIGHT)，映射到 120° ~ 50°
        tilt_angle = 120 - (120 - 50) * (target_y / (IMG_HEIGHT//2))
    else:
        # y 从 230(IMG_HEIGHT//2) 到 460(IMG_HEIGHT)，映射到 50° ~ 55°
        tilt_angle = 50 + (55 - 50) * ((target_y - IMG_HEIGHT//2) / (IMG_HEIGHT - IMG_HEIGHT//2))

    return pan_angle, tilt_angle

# -------------------- 初始化摄像头 --------------------
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.VGA)
sensor.set_auto_whitebal(False)
sensor.skip_frames(time=1500)
clock = time.clock()

# -------------------- 主循环 --------------------
while True:
    clock.tick()
    img = sensor.snapshot().lens_corr(1.8)  # 镜头校正

    # 颜色检测（请根据实际调整颜色阈值）
    COLOR_THRESHOLD = (12, 71, 18, 106, -37, 42)
    blobs = img.find_blobs([COLOR_THRESHOLD], roi=ROI, merge=True)

    # 绘制 ROI 区域
    img.draw_rectangle(ROI, color=(0, 255, 0))

    if blobs:
        # 选择面积最大的色块
        target = max(blobs, key=lambda b: b.pixels())
        img.draw_rectangle(target.rect(), color=(255, 0, 0))
        img.draw_cross(target.cx(), target.cy(), color=(0, 255, 255))

        # 获取目标中心坐标
        target_x, target_y = target.cx(), target.cy()

        # 计算对应舵机角度
        pan_target, tilt_target = map_angles(target_x, target_y)


        # 设置舵机角度
        pan_servo.angle(pan_target)
        tilt_servo.angle(tilt_target)

        # 启动水泵
        pump_pin_red.high()
        print(f"目标坐标: ({target_x}, {target_y})")
        print(f"舵机角度 - 水平: {pan_target:.1f}°, 垂直: {tilt_target:.1f}°")
    else:
        pump_pin_red.low()
        print("未检测到目标")

    # print(f"FPS: {clock.fps():.1f}")
