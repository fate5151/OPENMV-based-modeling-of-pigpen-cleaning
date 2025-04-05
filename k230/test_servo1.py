import time, os, sys

from machine import PWM, Pin, FPIOA
from media.sensor import * #导入sensor模块，使用摄像头相关接口
from media.display import * #导入display模块，使用display相关接口
from media.media import * #导入media模块，使用meida相关接口

# -------------------- 系统参数 --------------------

# 初始化 FPIOA（引脚复用）
fpioa = FPIOA()

# 舵机 PWM 引脚配置（请根据实际连接修改引脚号与对应 PWM 通道）
PAN_PWM_PIN = 42       # 水平舵机控制引脚（示例：GPIO42映射到PWM0）白线
TILT_PWM_PIN = 43      # 垂直舵机控制引脚（示例：GPIO43映射到PWM1）黑线

fpioa.set_function(PAN_PWM_PIN, FPIOA.PWM0)
fpioa.set_function(TILT_PWM_PIN, FPIOA.PWM1)

# 构建 PWM 对象，频率50Hz（标准舵机频率），初始占空比设置为0
pan_pwm = PWM(0, 50, 0, enable=True)
tilt_pwm = PWM(1, 50, 0, enable=True)

# 水泵控制引脚（根据实际接线调整引脚号）


# 舵机角度限制（防止过度旋转）
PAN_MIN, PAN_MAX = 45, 150
TILT_MIN, TILT_MAX = 45, 135

# 初始舵机角度（中间位置）
pan_angle = 90
tilt_angle = 90

def print1():
    print("Pan（水平）:", pan_angle, "Tilt（垂直）:", tilt_angle)
    #print("水泵状态:", pump_pin_red.value(), pump_pin_black.value())

def angle_to_duty(angle):
    """
    将角度 (0~180) 映射为 PWM 占空比（MG90S: 2.5% ~ 12.5%）
    duty = (angle/180)*10 + 2.5
    """
    return (angle / 180) * 10 + 2.5

def set_servo_angle(servo_pwm, angle, min_angle, max_angle):
    """
    限制角度在安全范围内，并设置对应 PWM 占空比
    :param servo_pwm: PWM 对象
    :param angle: 目标角度
    :param min_angle: 最小允许角度
    :param max_angle: 最大允许角度
    :return: 实际设置的角度（已限幅）
    """
    angle = max(min(angle, max_angle), min_angle)
    duty = angle_to_duty(angle)
    servo_pwm.duty(duty)
    time.sleep(0.05)  # 等待舵机响应
    return angle

def pan():
    """
    水平方向扫描：
    从 PAN_MIN 增加到 PAN_MAX，再从 PAN_MAX 回到 PAN_MIN
    """
    global pan_angle
    for angle in range(PAN_MIN, PAN_MAX, 2):
        pan_angle = set_servo_angle(pan_pwm, angle, PAN_MIN, PAN_MAX)
        print1()

    for angle in range(PAN_MAX, PAN_MIN, -2):
        pan_angle = set_servo_angle(pan_pwm, angle, PAN_MIN, PAN_MAX)
        print1()

def tilt():
    """
    垂直方向扫描：
    从 TILT_MIN 增加到 TILT_MAX，再从 TILT_MAX 回到 TILT_MIN
    """
    global tilt_angle
    for angle in range(TILT_MIN, TILT_MAX, 1):
        tilt_angle = set_servo_angle(tilt_pwm, angle, TILT_MIN, TILT_MAX)
        print1()

    for angle in range(TILT_MAX, TILT_MIN, -1):
        tilt_angle = set_servo_angle(tilt_pwm, angle, TILT_MIN, TILT_MAX)
        print1()

# 初始化云台位置（舵机置于中间）
pan_angle = set_servo_angle(pan_pwm, pan_angle, PAN_MIN, PAN_MAX)
tilt_angle = set_servo_angle(tilt_pwm, tilt_angle, TILT_MIN, TILT_MAX)

# 设置初始位置
pan_angle = set_servo_angle(pan_pwm, 90, PAN_MIN, PAN_MAX)
tilt_angle = set_servo_angle(tilt_pwm, 90, TILT_MIN, TILT_MAX)

while True:
    print("当前角度 - Pan:", pan_angle, "Tilt:", tilt_angle)

    # 水平扫描
    pan()
    time.sleep(2)

    # 垂直扫描
    #tilt()
    #time.sleep(5)


