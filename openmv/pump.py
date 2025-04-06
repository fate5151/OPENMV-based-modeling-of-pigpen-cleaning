import pyb
import time

# 定义控制 L9110S 电机驱动模块的两个 GPIO 引脚
motor_in1 = pyb.Pin("P1", pyb.Pin.OUT_PP)  # 控制电机正转
motor_in2 = pyb.Pin("P0", pyb.Pin.OUT_PP)  # 控制电机反转

def motor_forward():
    # 电机正转：in1 高电平，in2 低电平
    motor_in1.high()
    motor_in2.low()

def motor_reverse():
    # 电机反转：in1 低电平，in2 高电平
    motor_in1.low()
    motor_in2.high()

def motor_stop():
    # 停止电机：两个引脚均为低电平
    motor_in1.low()
    motor_in2.low()

# 主循环：依次正转、停止、反转、停止
while True:
    # 电机正转 2 秒
    motor_forward()
    print(motor_in1.value(), motor_in2.value())
    time.sleep(2)

    # 停止 1 秒
    motor_stop()
    print(motor_in1.value(), motor_in2.value())
    time.sleep(1)

    # 电机反转 2 秒
    motor_reverse()
    print(motor_in1.value(), motor_in2.value())
    time.sleep(2)

    # 停止 1 秒
    motor_stop()
    print(motor_in1.value(), motor_in2.value())
    time.sleep(1)
