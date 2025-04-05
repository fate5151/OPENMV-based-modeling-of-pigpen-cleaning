'''
智能喷淋系统（简化版）
修改说明：
1. 移除舵机角度安全限制
2. 注释弹道学运动计算
'''

import sensor, image, time, math
from pid import PID
from pyb import Servo, Pin

# ==================== 系统常量 ====================
CEILING_HEIGHT = 0.33        # 天花板到地面高度（米）
HORIZONTAL_OFFSET = 0.105    # 云台水平偏移量（正下方为负值）

# ==================== 硬件接口 ====================
pump_pin = Pin('P4', Pin.OUT_PP)
pan_servo = Servo(1)  # 水平舵机（P7）
tilt_servo = Servo(2) # 垂直舵机（P8）

# ==================== 视觉参数 ====================
IMG_WIDTH = 640
IMG_HEIGHT = 480
DETECT_ROI = (0, 10, 480, 450)  # 缩小检测区域提升速度
COLOR_THRESHOLD = (12, 32, 21, 106, -36, 42) # LAB阈值

# ==================== 焦距计算 ====================
SENSOR_WIDTH_MM = 3.6
SENSOR_HEIGHT_MM = 2.7
LENS_FOCAL_MM = 2.8

FX = (LENS_FOCAL_MM / SENSOR_WIDTH_MM) * IMG_WIDTH
FY = (LENS_FOCAL_MM / SENSOR_HEIGHT_MM) * IMG_HEIGHT
CX, CY = IMG_WIDTH//2, IMG_HEIGHT//2

# ==================== 控制系统 ====================
PAN_PID = PID(p=0.12, i=0.006, d=0.25, imax=50)
TILT_PID = PID(p=0.18, i=0.01, d=0.3, imax=50)

# ==================== 初始化摄像头 ====================
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.VGA)
sensor.set_auto_whitebal(False)
sensor.skip_frames(1500)
clock = time.clock()

def pixel_to_ground(cx, cy):
    '''坐标转换保留'''
    dx = (cx - CX) * CEILING_HEIGHT / FX
    dy = (cy - CY) * CEILING_HEIGHT / FY
    return (dx , dy - HORIZONTAL_OFFSET)

def calculate_angles(target_x, target_y):
    '''简化角度计算'''
    # 原运动学计算已被注释
    """
    pan_rad = math.atan(target_x / CEILING_HEIGHT)
    pan_angle = math.degrees(pan_rad)

    L = math.sqrt(target_x**2 + target_y**2)
    h = CEILING_HEIGHT

    sqrt_term = math.sqrt(WATER_SPEED**4 - GRAVITY*(GRAVITY*L**2 + 2*h*WATER_SPEED**2))
    if sqrt_term < 0:
        return (pan_angle, 45)

    tan_theta = (WATER_SPEED**2 - sqrt_term) / (GRAVITY * L)
    tilt_angle = math.degrees(math.atan(tan_theta))
    return (pan_angle, tilt_angle)
    """
    # 简化版本：直接比例控制
    pan_angle = target_x * 150  # 比例系数需实际调整
    tilt_angle = target_y * 125
    return (pan_angle, tilt_angle)

# ==================== 主循环 ====================
while True:
    clock.tick()
    img = sensor.snapshot().lens_corr(1.8)  # 根据实际镜头调整

    blobs = img.find_blobs([COLOR_THRESHOLD], roi=DETECT_ROI, merge=True)
    img.draw_rectangle(DETECT_ROI, color=(0,255,0))

    if blobs:
        target = max(blobs, key=lambda b: b.pixels())
        img.draw_rectangle(target.rect(), color=(255,0,0))
        img.draw_cross(target.cx(), target.cy(), color=(0,255,255))

        physical_x, physical_y = pixel_to_ground(target.cx(), target.cy())
        pan_target, tilt_target = calculate_angles(physical_x, physical_y)

        # 直接更新舵机角度（无安全限制）
        pan_servo.angle(pan_target)
        tilt_servo.angle(tilt_target)

        pump_pin.high()
        print(f"target.cx():{target.cx()},target.cy():{target.cy()}")
        print(f"physical_x:{physical_x},physical_y:{physical_y}")
        print(f"Set Angle - Pan:{pan_target:.1f} Tilt:{tilt_target:.1f}")
    else:
        pump_pin.low()
        print("Target Lost")

    #print(f"FPS:{clock.fps():.1f}")

'''
系统变更说明：
1. 移除的安全控制包括：
   - update_servo()函数
   - 舵机角度限制参数
   - PID的imax限制

2. 运动学修改部分：
   - 注释原抛物线运动计算
   - 添加简易比例控制
   - 移除WATER_SPEED和GRAVITY参数

使用注意事项：
1. 需手动确保舵机不超出物理限位
2. 比例系数需要根据实际测试调整
3. 喷射精度可能下降
'''
