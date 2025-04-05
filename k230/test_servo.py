from machine import Pin, PWM
from machine import FPIOA
import time

# -------------------- 系统参数 --------------------



#配置引脚42为PWM0功能
#通道0：GPIO42,通道1：GPIO43,通道2：GPIO46,通道3：GPIO47
fpioa = FPIOA()
fpioa.set_function(42,FPIOA.PWM0)
fpioa.set_function(43,FPIOA.PWM1)

# PWM 端口（根据实际接线修改）
#水平：白线，pwm0（IO42）
#垂直：黑线：pwm1（IO43）
#构建PWM0对象，通道0，频率为50Hz，占空比为0，默认使能输出
PAN_PWM_PIN = PWM(0, 50, 0, enable=True) # 在同一语句下创建和配置PWM
tilt_pwm  = PWM(1, 50, 0, enable=True)

PAN_PWM_PIN = 1   # 水平方向（Pan）舵机PWM端口
TILT_PWM_PIN = 2  # 垂直方向（Tilt）舵机PWM端口

# 初始化 PWM 频率（50Hz 适用于 MG90S 舵机）
pwm.init(PAN_PWM_PIN, freq=50)
pwm.init(TILT_PWM_PIN, freq=50)

# 舵机角度范围设置
PAN_MIN = 45
PAN_MAX = 150
TILT_MIN = 45
TILT_MAX = 135

# 初始角度（中间位置）
pan_angle = 90
tilt_angle = 90

def set_servo_angle(pin, angle):
    """
    限制角度并设置 PWM 占空比。
    MG90S 舵机控制脉宽范围大约在 0.5ms - 2.5ms，
    对应 PWM 占空比大约为 2.5% ~ 12.5%。
    """
    # 限制角度范围在 0° 到 180° 内
    angle = max(min(angle, 180), 0)
    # 将角度映射到 PWM 占空比
    duty = (angle / 180) * 10 + 2.5
    pwm.set_duty(pin, duty)
    time.sleep(0.02)  # 等待舵机响应

def pan():
    """ 水平方向（Pan）扫描运动 """
    # 从 PAN_MIN 运动到 PAN_MAX
    for angle in range(PAN_MIN, PAN_MAX, 2):
        set_servo_angle(PAN_PWM_PIN, angle)
        print("Pan Angle:", angle)
        time.sleep(0.02)
    # 再从 PAN_MAX 运动回 PAN_MIN
    for angle in range(PAN_MAX, PAN_MIN, -2):
        set_servo_angle(PAN_PWM_PIN, angle)
        print("Pan Angle:", angle)
        time.sleep(0.02)

def tilt():
    """ 垂直方向（Tilt）扫描运动 """
    # 从 TILT_MIN 运动到 TILT_MAX
    for angle in range(TILT_MIN, TILT_MAX, 1):
        set_servo_angle(TILT_PWM_PIN, angle)
        print("Tilt Angle:", angle)
        time.sleep(0.02)
    # 再从 TILT_MAX 运动回 TILT_MIN
    for angle in range(TILT_MAX, TILT_MIN, -1):
        set_servo_angle(TILT_PWM_PIN, angle)
        print("Tilt Angle:", angle)
        time.sleep(0.02)

# 初始化舵机至中间位置
set_servo_angle(PAN_PWM_PIN, pan_angle)
set_servo_angle(TILT_PWM_PIN, tilt_angle)

# 初始化摄像头
sensor.reset()                    # 初始化摄像头
sensor.set_pixformat(sensor.RGB565)  # 设置图像格式为 RGB565
sensor.set_framesize(sensor.QVGA)      # 设置分辨率为 QVGA
sensor.skip_frames(time=2000)      # 等待摄像头稳定
sensor.set_auto_whitebal(False)    # 关闭自动白平衡
clock = time.clock()               # 用于计算帧率

while True:
    img = sensor.snapshot()
    print(f"当前初始角度 - Pan: {pan_angle}, Tilt: {tilt_angle}")

    # 执行水平方向扫描
    pan()
    time.sleep(1)

    # 执行垂直方向扫描
    tilt()
    time.sleep(1)
