import sensor, image, time
from pyb import Servo, Pin
from pid import PID

# -------------------- 系统参数 --------------------
IMG_WIDTH = 500   # ROI 区宽度（像素）
IMG_HEIGHT = 420  # ROI 区高度（像素）
ROI = (0, 40, IMG_WIDTH, IMG_HEIGHT)  # ROI区域

# -------------------- 舵机参数 --------------------
pan_servo = Servo(1)   # 水平舵机
tilt_servo = Servo(2)  # 垂直舵机

# 舵机校准参数（脉冲宽度范围500-2500μs，误差补偿500μs）
pan_servo.calibration(500, 2500, 500)
tilt_servo.calibration(500, 2500, 500)

# 水泵控制
pump_pin_red = Pin('P5', Pin.OUT_PP)
pump_pin_black = Pin('P6', Pin.OUT_PP)
pump_pin_red.low()
pump_pin_black.low()


# -------------------- 初始化 PID 控制器 --------------------
# PID 参数可根据实际情况进行调整
pan_pid = PID(p=0.05, i=0, d=0.01, imax=90)
tilt_pid = PID(p=0.05, i=0, d=0.01, imax=90)

# 初始舵机角度，假设初始均为90°
pan_current = 90
tilt_current = 90

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
        # y 从 0 到 230(IMG_HEIGHT//2)，映射到 120° ~ 50°
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

# 用于计算时间间隔 dt
last_time = time.ticks_ms()

# -------------------- 主循环 --------------------
while True:
    current_time = time.ticks_ms()
    dt = time.ticks_diff(current_time, last_time) / 1000.0  # dt 单位：秒
    last_time = current_time

    clock.tick()
    img = sensor.snapshot().lens_corr(1.8)  # 镜头校正

    # 颜色检测（根据实际情况调整颜色阈值）
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

        # 计算目标对应的舵机角度
        pan_target, tilt_target = map_angles(target_x, target_y)

        # 计算误差并使用PID控制器获得修正量
        error_pan = pan_target - pan_current
        error_tilt = tilt_target - tilt_current

        # 计算修正量（PID 输出）
        correction_pan = pan_pid.get_pid(error_pan, dt)
        correction_tilt = tilt_pid.get_pid(error_tilt, dt)

        # 更新当前角度（可根据需要加入限制，防止超出范围）
        pan_current += correction_pan
        tilt_current += correction_tilt

        # 将更新后的角度传给舵机
        pan_servo.angle(pan_current)
        tilt_servo.angle(tilt_current)

        # 启动水泵
        pump_pin_red.high()

        print("目标坐标: ({}, {})".format(target_x, target_y))
        print("目标角度 - 水平: {:.1f}°, 垂直: {:.1f}°".format(pan_target, tilt_target))
        print("当前角度 - 水平: {:.1f}°, 垂直: {:.1f}°".format(pan_current, tilt_current))
    else:
        # 当未检测到目标时可以选择停止水泵或恢复到默认角度
        pump_pin_red.low()
        print("未检测到目标")

    # 可选：输出FPS信息
    # print("FPS: {:.1f}".format(clock.fps()))
