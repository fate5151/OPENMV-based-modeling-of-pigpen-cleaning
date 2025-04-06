import time
import sensor
from pyb import Servo, Pin
# -------------------- 系统参数 --------------------

# 初始化两个舵机（水平方向-Pan，垂直方向-Tilt）
# 根据实际接线修改引脚号（例如P7/P8）
pan_servo = Servo(1)  # 舵机1接P7（水平方向）白线
tilt_servo = Servo(2) # 舵机2接P8（垂直方向）黑线

# 设置舵机角度限制（防止过度旋转）
PAN_MIN = 45
PAN_MAX = 150
TILT_MIN = 45
TILT_MAX = 135

# 水泵控制
pump_pin_red = Pin('P5', Pin.OUT)
pump_pin_black = Pin('P6', Pin.OUT)
pump_pin_red.low()
pump_pin_black.low()

# 基础角度控制模式（0°~180°）
pan_servo.calibration(500,2500,500)
tilt_servo.calibration(500,2500,500)

# 初始位置（中间位置）
pan_angle = 90
tilt_angle = 90


def set_servo_angle(servo, angle, min_angle, max_angle):
    # 限制角度在安全范围内
    angle = max(min(angle, max_angle), min_angle)
    # MG90S需要校准角度到脉宽，OpenMV的Servo库角度范围是-90到+90
    # 因此需要将0-180度映射到-90到+90
    calibrated_angle = angle
    servo.angle(calibrated_angle)
    time.sleep_ms(50)  # 等待舵机转动


def pan():
    for pan_angle in range(PAN_MIN, PAN_MAX, 2):
        set_servo_angle(pan_servo, pan_angle, PAN_MIN, PAN_MAX)
        print(pan_servo.angle(), tilt_servo.angle())  # 打印当前舵机角度
        print(pump_pin_red.value(), pump_pin_black.value())  # 打印水泵状态


    for pan_angle in range(PAN_MAX, PAN_MIN, -2):
        set_servo_angle(pan_servo, pan_angle, PAN_MIN, PAN_MAX)
        print(pan_servo.angle(), tilt_servo.angle())  # 打印当前舵机角度
        print(pump_pin_red.value(), pump_pin_black.value())  # 打印水泵状态



def tilt():
        # 示例：垂直扫描运动
    for tilt_angle in range(TILT_MIN, TILT_MAX, 1):
        set_servo_angle(tilt_servo, tilt_angle, TILT_MIN, TILT_MAX)
        print(pan_servo.angle(), tilt_servo.angle())  # 打印当前舵机角度
        print(pump_pin_red.value(), pump_pin_black.value())  # 打印水泵状态



    for tilt_angle in range(TILT_MAX, TILT_MIN, -1):
        set_servo_angle(tilt_servo, tilt_angle, TILT_MIN, TILT_MAX)
        print(pan_servo.angle(), tilt_servo.angle())  # 打印当前舵机角度
        print(pump_pin_red.value(), pump_pin_black.value())  # 打印水泵状态
    # 水泵控制示例


# 初始化云台位置
set_servo_angle(pan_servo, pan_angle, PAN_MIN, PAN_MAX)
set_servo_angle(tilt_servo, tilt_angle, TILT_MIN, TILT_MAX)


sensor.reset() # Initialize the camera sensor.
sensor.set_pixformat(sensor.RGB565) # use RGB565.
sensor.set_framesize(sensor.QVGA) # use QQVGA for speed.
sensor.skip_frames(time=2000) # Let new settings take affect.
sensor.set_auto_whitebal(False) # turn this off.
clock = time.clock() # Tracks FPS.

pan_servo.angle(90)
tilt_servo.angle(90)

while True:

    img = sensor.snapshot()

    print(pan_servo.angle(), tilt_servo.angle())  # 打印当前舵机角度

    pan()
    #pan_servo.angle(50)
    #tilt_servo.angle(50)
    time.sleep(2)
    tilt()
    time.sleep(2)

    #tilt()
