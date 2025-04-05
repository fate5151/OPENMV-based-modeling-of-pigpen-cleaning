import sensor, image, time

from pid import PID
from pyb import Servo,Pin

# -------------------- 水泵控制 --------------------
pump_pin_red = Pin('P0', Pin.OUT)
pump_pin_black = Pin('P1', Pin.OUT)
pump_pin_red.low()   # 初始低电平
pump_pin_black.low()

# -------------------- 舵机参数 --------------------
# 初始化两个舵机（水平方向-Pan，垂直方向-Tilt）
pan_servo=Servo(1)   # 水平舵机（连接到P7）
tilt_servo=Servo(2)  # 垂直舵机（连接到P8）

# 设置舵机角度限制（防止过度旋转）
PAN_MIN, PAN_MAX = 45, 150
TILT_MIN, TILT_MAX = 45, 135

pan_servo.calibration(500,2500,500)
tilt_servo.calibration(500,2500,500)


#pan_pid = PID(p=0.07, i=0,d=0.001 , imax=90) #脱机运行或者禁用图像传输，使用这个PID
#tilt_pid = PID(p=0.05, i=0, d=0.001 ,imax=90) #脱机运行或者禁用图像传输，使用这个PID
pan_pid = PID(p=0.04, i=0.009,d=0.0005 ,imax=90)#在线调试使用这个PID
tilt_pid= PID(p=0.04, i=0.009,d=0.0005, imax=90)#在线调试使用这个PID

# -------------------- ROI 设置 --------------------
# 全局检测区域
IMG_WIDTH = int(sensor.width()*(3/4))  # ROI 区宽度（像素）
IMG_HEIGHT = int(sensor.height()*(3/4))  # ROI 区高度（像素）
x_offset = int((sensor.width()-IMG_WIDTH)/2)
y_offset = int((sensor.height()-IMG_HEIGHT)/2)

pig_roi = (x_offset,y_offset, IMG_WIDTH,IMG_HEIGHT)  # 检测区域(起始x,起始y,宽度,高度)

# 中心区域（用于判断目标是否对准喷头），这里以图像中心为参考
IMG_WIDTH = int(sensor.width()*(1/11))  # ROI 区宽度（像素）
IMG_HEIGHT = int(sensor.height()*(1/11))  # ROI 区高度（像素）
x_offset = int((sensor.width()-IMG_WIDTH)/2)
y_offset = int((sensor.height()-IMG_HEIGHT)/2)

center_roi = (x_offset,y_offset, IMG_WIDTH,IMG_HEIGHT)  # 检测区域(起始x,起始y,宽度,高度)

# -------------------- 颜色阈值配置（LAB色彩空间） --------------------
red_threshold  =(16, 55, 3, 127, -22, 17)
shift_threshold  = (18, 79, -38, 57, 35, 102)
shift_threshold=red_threshold

# -------------------- 摄像头初始化 --------------------
sensor.reset() # Initialize the camera sensor.
sensor.set_pixformat(sensor.RGB565) # use RGB565.
sensor.set_framesize(sensor.QVGA) # use QQVGA for speed.
sensor.skip_frames(time=2000) # Let new settings take affect.
sensor.set_auto_whitebal(False) # turn this off.
clock = time.clock() # Tracks FPS.

# -------------------- 辅助函数 --------------------
def find_max(blobs):
    max_blob = None
    max_size=0
    for blob in blobs:
        if blob[2]*blob[3] > max_size:
            max_blob=blob
            max_size = blob[2]*blob[3]
    return max_blob

def set_servo_angle(servo, angle, min_angle, max_angle):
    # 限制角度在安全范围内
    angle = max(min(angle, max_angle), min_angle)
    calibrated_angle = angle
    servo.angle(calibrated_angle)
    time.sleep_ms(50)  # 等待舵机转动

def pump_on():
    #判断舵机是否到达指定位置
    # 判断最大色块的中心是否在 center_roi 内
    # center_roi = (x_offset, y_offset, width, height)
    (cx, cy) = (max_blob.cx(), max_blob.cy())
    if (center_roi[0] <= cx <= center_roi[0] + center_roi[2]) and \
       (center_roi[1] <= cy <= center_roi[1] + center_roi[3]):
        # 最大色块中心在 center_roi 内，说明舵机已经到位
        pump_pin_red.high()  # 设置 pin4 为高电平
        pump_pin_black.low()

def pump_off():
    pump_pin_red.low()
    pump_pin_black.low()

# -------------------- 初始化 PID 控制器 --------------------

#舵机位置初始化
pan_servo.angle(90)
tilt_servo.angle(45)

# 设置长时间未检测到目标的超时时间（毫秒）
no_detection_timeout = 50000
last_detection_time = time.ticks_ms()

# -------------------- 主循环 --------------------
while(True):
    clock.tick() # Track elapsed milliseconds between snapshots().
    img = sensor.snapshot().lens_corr(1.8)  # 镜头畸变校正

    blobs = img.find_blobs([shift_threshold], roi=pig_roi, merge=True)
    img.draw_rectangle(pig_roi, color=(0,255,0))  # 绘制检测区域
    img.draw_rectangle(center_roi, color=(255,0,0))  # 绘制检测区域

    if blobs:
        max_blob = find_max(blobs)
        pan_error = max_blob.cx()-img.width()/2
        tilt_error = max_blob.cy()-img.height()/2

        img.draw_rectangle(max_blob.rect()) # rect
        img.draw_cross(max_blob.cx(), max_blob.cy()) # cx, cy

        pan_output=pan_pid.get_pid(pan_error,1)/2
        tilt_output=tilt_pid.get_pid(tilt_error,1)
        pan_servo.angle(pan_servo.angle()+pan_output)
        tilt_servo.angle(tilt_servo.angle()-tilt_output)

        print(pan_servo.angle(),tilt_servo.angle())
        print(f"舵机的水平旋转角度:{pan_servo.angle()},垂直角度{tilt_servo.angle()} ")  # 打印水泵状态


        print("有粪便")
        pump_on()  # 设置 pin4 为高电平
        print(f"“水泵状态:{pump_pin_red.value()},{pump_pin_black.value()} ")  # 打印水泵状态

    else:
        print("没有粪便")
        pump_pin_red.low()  # 设置 pin4 为低电平
        print(f"“水泵状态:{pump_pin_red.value()},{pump_pin_black.value()} ")  # 打印水泵状态


