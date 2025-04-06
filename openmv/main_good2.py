import sensor, image, time
from pid import PID
from pyb import Servo, Pin

# -------------------- 水泵控制 --------------------
pump_pin_red = Pin('P0', Pin.OUT)
pump_pin_black = Pin('P1', Pin.OUT)
pump_pin_red.low()       # 初始低电平
pump_pin_black.low()

# -------------------- 舵机参数 --------------------
pan_servo = Servo(1)  # 水平舵机（连接到P7）
tilt_servo = Servo(2)  # 垂直舵机（连接到P8）

PAN_MIN, PAN_MAX = 0, 180
TILT_MIN, TILT_MAX = 20, 90

pan_servo.calibration(500, 2500, 500)
tilt_servo.calibration(500, 2500, 500)

# PID 控制器参数
pan_pid = PID(p=0.04, i=0.009,d=0.0005 ,imax=90)#在线调试使用这个PID
tilt_pid= PID(p=0.04, i=0.009,d=0.0005, imax=90)#在线调试使用这个PID
# -------------------- 摄像头初始化 --------------------
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_whitebal(False)
clock = time.clock()

# -------------------- ROI 设置 --------------------
# 全局检测区域
IMG_WIDTH = int(sensor.width() * (3 / 4))
IMG_HEIGHT = int(sensor.height() * (3 / 4))
x_offset = int((sensor.width() - IMG_WIDTH) / 2)
y_offset = int((sensor.height() - IMG_HEIGHT) / 2)
pig_roi = (x_offset, y_offset, IMG_WIDTH, IMG_HEIGHT)

# 中心区域（用于判断目标是否对准喷头），这里以图像中心为参考
roi_size = int(sensor.width() * (1 / 11))
center_x = int((sensor.width() - roi_size) / 2)
center_y = int((sensor.height() - roi_size) / 2)
center_roi = (center_x, center_y, roi_size, roi_size)

# -------------------- 颜色阈值设置 --------------------
red_threshold  =(16, 55, 3, 127, -22, 17)
shift_threshold  = (18, 79, -38, 57, 35, 102)
shift_threshold=red_threshold

# -------------------- 辅助函数 --------------------
def find_max(blobs):
    max_blob = None
    max_size = 0
    for blob in blobs:
        area = blob[2] * blob[3]
        if area > max_size:
            max_blob = blob
            max_size = area
    return max_blob

def set_servo_angle(servo, angle, min_angle, max_angle):
    angle = max(min(angle, max_angle), min_angle)
    servo.angle(angle)
    time.sleep_ms(50)

def is_target_centered(blob):
    # 直接判断目标色块中心是否落在 center_roi 内
    cx = blob.cx()
    cy = blob.cy()
    return (center_roi[0] <= cx <= center_roi[0] + center_roi[2]) and \
           (center_roi[1] <= cy <= center_roi[1] + center_roi[3])

def pump_on():
    pump_pin_red.high()

def pump_off():
    pump_pin_red.low()

# -------------------- 初始化舵机 --------------------
#舵机位置初始化
pan_servo.angle(90)
tilt_servo.angle(45)

# 设置长时间未检测到目标的超时时间（毫秒）
no_detection_timeout = 10000
last_detection_time = time.ticks_ms()

# -------------------- 主循环 --------------------
while True:
    clock.tick()
    img = sensor.snapshot().lens_corr(1.8)
    blobs = img.find_blobs([shift_threshold], roi=pig_roi, merge=True)
    img.draw_rectangle(pig_roi, color=(0, 255, 0))
    img.draw_rectangle(center_roi, color=(255, 0, 0))

    if blobs:
        # 更新最后检测到目标的时间
        last_detection_time = time.ticks_ms()

        max_blob = find_max(blobs)
        img.draw_rectangle(max_blob.rect())
        img.draw_cross(max_blob.cx(), max_blob.cy())

        pan_error = max_blob.cx() - (img.width() / 2)
        tilt_error = max_blob.cy() - (img.height() / 2)
        pan_output = pan_pid.get_pid(pan_error, 1) / 2
        tilt_output = tilt_pid.get_pid(tilt_error, 1)

        new_pan = pan_servo.angle() + pan_output
        new_tilt = tilt_servo.angle() - tilt_output
        set_servo_angle(pan_servo, new_pan, PAN_MIN, PAN_MAX)
        set_servo_angle(tilt_servo, new_tilt, TILT_MIN, TILT_MAX)

        print("舵机位置 -> 水平: {}° 垂直: {}°".format(pan_servo.angle(), tilt_servo.angle()))
        if is_target_centered(max_blob):
            pump_on()
            print("目标对准，水泵启动")
        else:
            pump_off()
            print("目标未对准，水泵关闭")
        print("水泵状态: {}, {}".format(pump_pin_red.value(), pump_pin_black.value()))
    else:
        pump_off()
        print("没有检测到目标")
        # 如果长时间未检测到目标，则重新初始化舵机
        if time.ticks_diff(time.ticks_ms(), last_detection_time) > no_detection_timeout:
            print("长时间未检测到目标，舵机重新初始化")
            pan_servo.angle(90)
            tilt_servo.angle(45)
            last_detection_time = time.ticks_ms()
