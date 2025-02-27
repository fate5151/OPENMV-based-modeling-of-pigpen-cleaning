import sensor, image, time
from pid import PID
from pyb import Servo

pan_servo = Servo(1)  # 水平 P7
tilt_servo = Servo(2)  # 垂直 P8

pan_servo.calibration(500, 2500, 500)
tilt_servo.calibration(500, 2500, 500)

# 颜色阈值，需根据实际目标调整
shift_threshold = (23, 52, -6, 22, 31, 53)

# PID参数调整
pan_pid = PID(p=0.05, i=0.001, d=0.02, imax=50)  # 降低P，添加微分项D
tilt_pid = PID(p=0.05, i=0.001, d=0.02, imax=50)

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)  # 使用QVGA提高帧率
sensor.skip_frames(2000)
sensor.set_auto_whitebal(False)
clock = time.clock()

def clamp_angle(angle, min_angle=0, max_angle=180):
    return max(min_angle, min(angle, max_angle))

def find_max(blobs):
    max_size = 0
    max_blob = None
    for blob in blobs:
        if blob.area() > max_size:
            max_blob = blob
            max_size = blob.area()
    return max_blob

while True:
    clock.tick()
    img = sensor.snapshot()

    blobs = img.find_blobs([shift_threshold], merge=True)
    if blobs:
        max_blob = find_max(blobs)
        if max_blob:
            img.draw_rectangle(max_blob.rect())
            img.draw_cross(max_blob.cx(), max_blob.cy())

            # 计算误差
            pan_error = max_blob.cx() - img.width() // 2
            tilt_error = max_blob.cy() - img.height() // 2

            # 动态死区
            if abs(pan_error) < 5: pan_error = 0
            if abs(tilt_error) < 5: tilt_error = 0

            # 计算PID输出
            pan_output = pan_pid.get_pid(pan_error, 1)
            tilt_output = tilt_pid.get_pid(tilt_error, 1)

            # 限制输出速度
            pan_output = max(-3.0, min(pan_output, 3.0))
            tilt_output = max(-3.0, min(tilt_output, 3.0))

            # 注意这里的方向修正！关键负号
            new_pan = clamp_angle(pan_servo.angle() - pan_output)
            new_tilt = clamp_angle(tilt_servo.angle() - tilt_output)

            pan_servo.angle(new_pan)
            tilt_servo.angle(new_tilt)
    else:
        # 目标丢失时缓慢回中
        pan_output = pan_pid.get_pid(-pan_servo.angle(), 1)
        tilt_output = tilt_pid.get_pid(-tilt_servo.angle(), 1)
        pan_servo.angle(clamp_angle(pan_servo.angle() + pan_output))
        tilt_servo.angle(clamp_angle(tilt_servo.angle() + tilt_output))
