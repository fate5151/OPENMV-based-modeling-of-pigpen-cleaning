'''
智能喷淋系统（带水平偏移补偿）
硬件配置：
- OpenMV摄像头与云台水平间距10.5cm
- 摄像头安装高度33cm（垂直方向）
- 水枪喷头固定在云台上
'''

import sensor, image, time, math
from pid import PID
from pyb import Servo, Pin

# ==================== 系统常量 ====================
CEILING_HEIGHT = 0.33        # 垂直高度（米）
HORIZONTAL_OFFSET = 0.105    # 水平方向偏移量（云台在摄像头右侧+）
WATER_SPEED = 1           # 水速（m/s）需实测校准
GRAVITY = 9.81               # 重力加速度

# ==================== 硬件接口 ====================
pump_pin = Pin('P4', Pin.OUT_PP)
pan_servo = Servo(1)  # 水平舵机（P7）
tilt_servo = Servo(2) # 垂直舵机（P8）

# ==================== 视觉参数 ====================
IMG_WIDTH = 640
IMG_HEIGHT = 480
DETECT_ROI = (0, 10, 480, 450)  # 缩小检测区域提升速度
COLOR_THRESHOLD = (41, 100, 10, 109, 9, 96) # LAB阈值

LENS_FOCAL_MM  = 2.8 # 镜头焦距（mm）
SENSOR_WIDTH_MM =   3.6 # 摄像头传感器宽度（mm）
SENSOR_HEIGHT_MM = 2.7 # 摄像头传感器高度（mm）
# ==================== 控制参数 ====================
PAN_PID = PID(p=0.12, i=0.006, d=0.25, imax=50)
TILT_PID = PID(p=0.18, i=0.01, d=0.3, imax=50)

# 基础角度控制模式（0°~180°）
pan_servo.calibration(600, 2400,800)  # 设置 min=600μs, max=2400μs, 中心=1500μs
tilt_servo.calibration(600, 2400, 800)

# ==================== 摄像头标定参数 ====================
FX = (LENS_FOCAL_MM / SENSOR_WIDTH_MM) * IMG_WIDTH  # 摄像头焦距（像素）需标定
FY = (LENS_FOCAL_MM / SENSOR_HEIGHT_MM) * IMG_HEIGHT

CX = IMG_WIDTH // 2 + 20  # 光学中心偏移补偿
CY = IMG_HEIGHT // 2 - 15

# ==================== 系统初始化 ====================
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.VGA)
#sensor.set_vflip(True)
#sensor.set_hmirror(True)
sensor.skip_frames(1500)
clock = time.clock()

def pixel_to_physical(cx, cy):
    '''
    图像坐标转物理坐标（考虑水平偏移）
    返回：(target_x, target_y) 单位：米
    '''
    # 计算相对于光心的坐标
    dx = (cx - CX) / FX * CEILING_HEIGHT
    dy = (cy - CY) / FY * CEILING_HEIGHT

    # 坐标系转换（摄像头坐标系 -> 云台坐标系）
    target_x = dx
    target_y = dy  - HORIZONTAL_OFFSET
    return (target_x, target_y)

def calculate_jet_angles(target_x, target_y):
    '''
    计算命中目标所需角度（带偏移补偿）
    返回：(pan_angle, tilt_angle)
    '''
    # 水平角度计算（反正切函数）
    pan_angle = math.degrees(math.atan(target_x / CEILING_HEIGHT))

    # 抛物线运动方程
    L = math.sqrt(target_x**2 + target_y**2)
    h = CEILING_HEIGHT

    # 运动学方程解算
    discriminant = WATER_SPEED**4 - GRAVITY*(GRAVITY*L**2 + 2*h*WATER_SPEED**2)
    if discriminant < 0:
        return (pan_angle, 45)  # 无解时使用安全角度

    tan_theta = (WATER_SPEED**2 - math.sqrt(discriminant)) / (GRAVITY * L)
    tilt_angle = math.degrees(math.atan(tan_theta))

    return (pan_angle, tilt_angle)

def servo_safe_set(servo, angle, limit=60):
    '''舵机安全控制'''
    angle = max(-limit, min(limit, angle))
    servo.angle(angle)
    return angle

# ==================== 主控制循环 ====================
while True:
    clock.tick()
    img = sensor.snapshot().lens_corr(1.7)  # 根据实际镜头调整

    # 目标检测
    blobs = img.find_blobs([COLOR_THRESHOLD],
                          roi=DETECT_ROI,
                          merge=True,
                          margin=20)
    img.draw_rectangle(DETECT_ROI, color=(0,255,0))

    if blobs:
        target = max(blobs, key=lambda b: b.pixels())
        img.draw_rectangle(target.rect(), color=(255,0,0))
        img.draw_cross(target.cx(), target.cy(), color=(0,255,255))

        # 坐标转换（带偏移补偿）
        physical_x, physical_y = pixel_to_physical(target.cx(), target.cy())

        # 运动学解算
        pan_target, tilt_target = calculate_jet_angles(physical_x, physical_y)

        # PID控制
        pan_error = pan_target - pan_servo.angle()
        tilt_error = tilt_target - tilt_servo.angle()

        pan_output = PAN_PID.get_pid(pan_error, 1/clock.fps())
        tilt_output = TILT_PID.get_pid(tilt_error, 1/clock.fps())

        # 更新舵机
        #new_pan = servo_safe_set(pan_servo, pan_servo.angle() + pan_output)
       # new_tilt = servo_safe_set(tilt_servo, tilt_servo.angle() + tilt_output)

        pan_servo.angle(pan_servo.angle()+pan_output)
        tilt_servo.angle(tilt_servo.angle()-tilt_output)

        # 控制水泵
        pump_pin.high()

        print(f"物理坐标 X:{physical_x:.3f}m Y:{physical_y:.3f}m")
        #print(f"设定角度 Pan:{new_pan:.1f}° Tilt:{new_tilt:.1f}°")
    else:
        pump_pin.low()
        print("目标丢失...")

    print(f"帧率:{clock.fps():.1f} 水速:{WATER_SPEED}m/s")

# ==================== 系统标定指南 ====================
'''
标定步骤：
1. 焦距标定：
   - 放置标定板（格子间距已知）
   - 测量实际物理尺寸与像素尺寸的比例
   - 计算 FX = (像素宽度 × 实际高度) / 实际宽度

2. 水平偏移补偿验证：
   a. 在地面标记参考点（正对摄像头下方）
   b. 控制云台中心对准该点
   c. 测量实际落点与标记点的水平偏差
   d. 调整HORIZONTAL_OFFSET参数直至重合

3. 水流速度校准：
   - 设置垂直喷射角度为45°
   - 测量水流落地距离d
   - 计算 WATER_SPEED = d / sqrt(2*CEILING_HEIGHT/GRAVITY)

注意事项：
1. 确保OpenMV与云台的刚性连接
2. 定期检查机械结构的偏移量是否变化
3. 不同水压时需要重新校准WATER_SPEED
'''
