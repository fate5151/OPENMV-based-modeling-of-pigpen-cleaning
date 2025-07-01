import sensor, image, time, ml, math, uos, gc
from pid import PID
from pyb import Servo, Pin
# -------------------- 神经网络 --------------------
net = None
labels = None
min_confidence = 0.5

# -------------------- 水泵与报警控制 --------------------
pump_pin_red = Pin('P0', Pin.OUT)
pump_pin_black = Pin('P1', Pin.OUT)
alarm_pin = Pin('P2', Pin.OUT, Pin.PULL_DOWN)  # 报警指示引脚：触发报警时置高
pump_pin_red.low()       # 初始低电平
pump_pin_black.low()
alarm_pin.low()          # 报警初始关闭

# -------------------- 舵机参数 --------------------
pan_servo = Servo(1)  # 水平舵机（连接到P7）
tilt_servo = Servo(2)  # 垂直舵机（连接到P8）

PAN_MIN, PAN_MAX = 0, 180
TILT_MIN, TILT_MAX = 20, 90

pan_servo.calibration(500, 2500, 500)
tilt_servo.calibration(500, 2500, 500)

# -------------------- PID 控制器参数 --------------------
pan_pid = PID(p=0.09, i=0.01, d=0.0009, imax=90)
tilt_pid = PID(p=0.09, i=0.01, d=0.0009, imax=90)

# -------------------- 摄像头初始化 --------------------
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)  # 使用 QVGA 分辨率
sensor.skip_frames(time=2000)
sensor.set_auto_whitebal(False)
clock = time.clock()

# -------------------- ROI 设置函数 --------------------
def update_roi():
    global pig_roi, center_roi, IMG_WIDTH, IMG_HEIGHT, x_offset, y_offset, center_x, center_y, roi_size
    IMG_WIDTH = int(sensor.width() * (3 / 4))
    IMG_HEIGHT = int(sensor.height() * (3 / 4))
    x_offset = int((sensor.width() - IMG_WIDTH) / 2)
    y_offset = int((sensor.height() - IMG_HEIGHT) / 2)
    pig_roi = (x_offset, y_offset, IMG_WIDTH, IMG_HEIGHT)

    roi_size = int(sensor.width() * (1 / 11))
    center_x = int((sensor.width() - roi_size) / 2)
    center_y = int((sensor.height() - roi_size) / 2)
    center_roi = (center_x, center_y, roi_size, roi_size)

update_roi()  # 初始化 ROI

# -------------------- 颜色阈值设置 --------------------
shift_threshold = (28, 77, -13, 59, 31, 127)
red_threshold  = (15, 87, 5, 127, -10, 51)
# 根据需要切换不同的颜色阈值
color_threshold = shift_threshold

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
    cx = blob.cx()
    cy = blob.cy()
    return (center_roi[0] <= cx <= center_roi[0] + center_roi[2]) and \
           (center_roi[1] <= cy <= center_roi[1] + center_roi[3])

def pump_on():
    pump_pin_red.high()
    pump_pin_black.low()

def pump_off():
    pump_pin_red.low()
    pump_pin_black.low()

# -------------------- 节能模式函数 --------------------
def enter_energy_saving_mode():
    global energy_saving
    energy_saving = True
    print("进入节能模式：降低分辨率，舵机归位")
    sensor.set_framesize(sensor.QQVGA)  # 降低分辨率至 QQVGA
    sensor.skip_frames(time=2000)
    update_roi()  # 更新 ROI 参数

def exit_energy_saving_mode():
    global energy_saving
    if energy_saving:
        energy_saving = False
        print("退出节能模式：恢复正常分辨率")
        sensor.set_framesize(sensor.QVGA)  # 恢复 QVGA 分辨率
        sensor.skip_frames(time=2000)
        update_roi()

# -------------------- 舵机初始化归位 --------------------
pan_servo.angle(90)
tilt_servo.angle(45)

# -------------------- 超时及报警参数 --------------------
energy_saving_timeout = 30000    # 30秒未检测到目标，进入节能模式
pump_work_alarm_time = 40000     # 水泵连续工作40秒触发报警

last_detection_time = time.ticks_ms()
pump_work_start_time = None  # 水泵工作起始时间
energy_saving = False

# -------------------- 主循环 --------------------
while True:
    try:
        clock.tick()
        # 获取镜头图像并进行透镜畸变校正
        img = sensor.snapshot().lens_corr(1.8)
        # 查找颜色阈值下的 blob
        blobs = img.find_blobs([color_threshold], roi=pig_roi, merge=True)
        img.draw_rectangle(pig_roi, color=(0, 255, 0))
        img.draw_rectangle(center_roi, color=(255, 0, 0))

        current_time = time.ticks_ms()
        if blobs:
            # 检测到目标，退出节能模式
            exit_energy_saving_mode()
            last_detection_time = current_time

            max_blob = find_max(blobs)
            if max_blob:
                # 对最大色块进行形态学滤波（腐蚀）处理
                blob_roi = max_blob.rect()
                blob_img = img.copy(roi=blob_roi)
                blob_img.erode(1)  # 腐蚀操作，减少噪声干扰
                # 绘制腐蚀后区域，仅供显示参考
                img.draw_rectangle(blob_roi, color=(0, 0, 255))
                img.draw_cross(max_blob.cx(), max_blob.cy())

                # PID 控制计算（以图像中心为参考）
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
                    if pump_work_start_time is None:
                        pump_work_start_time = current_time
                else:
                    pump_off()
                    print("目标未对准，水泵关闭")
                    pump_work_start_time = None

                print("水泵状态: {}".format("开启" if pump_pin_red.value() else "关闭"))

                # 检查水泵是否连续工作超过设定时间
                if pump_work_start_time is not None and time.ticks_diff(current_time, pump_work_start_time) > pump_work_alarm_time:
                    print("报警：水泵连续工作超过40秒！")
                    alarm_pin.high()  # 触发报警
                    time.sleep_ms(5000)
                    alarm_pin.low()
                    pump_work_start_time = current_time  # 重置计时

        else:
            pump_off()
            pump_work_start_time = None
            print("没有检测到目标")
            # 若长时间未检测到目标，进入节能模式并归位舵机
            if time.ticks_diff(current_time, last_detection_time) > energy_saving_timeout:
                pan_servo.angle(90)
                tilt_servo.angle(45)
                enter_energy_saving_mode()
                last_detection_time = current_time

    except Exception as e:
        print("异常报警:", e)
        alarm_pin.high()  # 异常时触发报警
        time.sleep_ms(5000)
        alarm_pin.low()
        last_detection_time = time.ticks_ms()
