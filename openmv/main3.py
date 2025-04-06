'''
智能喷淋控制系统
功能：通过视觉识别目标位置，控制水枪喷头精准喷射
硬件配置：
- OpenMV摄像头固定在天花板（高度33cm）
- 双轴舵机云台控制喷头方向
- GPIO输出喷射状态
物理模型考虑：
- 抛物线运动补偿
- 摄像头与喷头同轴安装
- 流体力学简化计算
'''

import sensor, image, time, math
from pid import PID
from pyb import Servo, Pin
from image  import Image

# ==================== 系统常量 ====================
CEILING_HEIGHT = 0.33  # 单位：米
WATER_SPEED = 2.5      # 水速估计值 m/s（需实测校准）
GRAVITY = 9.81         # 重力加速度

# ==================== 硬件配置 ====================
pump_pin = Pin('P4', Pin.OUT_PP)  # 水泵控制引脚
pan_servo = Servo(1)              # 水平舵机（P7）
tilt_servo = Servo(2)             # 垂直舵机（P8）

# ==================== 视觉参数 ====================
IMG_WIDTH = 640
IMG_HEIGHT = 480
DETECT_ROI = (80, 60, 480, 360)  # 有效检测区域
COLOR_THRESHOLD =  (41, 100, 10, 109, 9, 96)  # LAB阈值

# ==================== 控制参数 ====================
PAN_PID = PID(p=0.1, i=0.005, d=0.2, imax=45)
TILT_PID = PID(p=0.15, i=0.008, d=0.25, imax=45)

# ==================== 物理模型参数 ====================
FX = (2.8 / 3.6) * IMG_WIDTH # 摄像头焦距（像素）需标定
FY = (2.8 / 2.7) * IMG_HEIGHT

CX = IMG_WIDTH // 2
CY = IMG_HEIGHT // 2

# ==================== 系统初始化 ====================
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.VGA)
#sensor.set_vflip(True)    # 天花板安装需垂直翻转
sensor.skip_frames(1500)
sensor.set_auto_whitebal(False)
clock = time.clock()

def pixel_to_world(cx, cy):
    '''
    图像坐标转物理世界坐标（地面坐标系）
    返回：(x, y) 单位：米
    '''
    dx = (cx - CX) / FX * CEILING_HEIGHT
    dy = (cy - CY) / FY * CEILING_HEIGHT
    return (dx, dy)

def calculate_angles(target_x, target_y):
    '''
    计算命中目标所需的喷射角度
    返回：(水平角度, 垂直角度)
    '''
    # 水平角度计算
    pan_angle = math.degrees(math.atan(target_x / CEILING_HEIGHT))

    # 抛物线方程解算
    distance = math.sqrt(target_x**2 + target_y**2)
    discriminant = WATER_SPEED**4 - GRAVITY*(GRAVITY*distance**2 + 2*CEILING_HEIGHT*WATER_SPEED**2)
    if discriminant < 0:
        return (pan_angle, 45)  # 无法命中时使用默认角度

    tan_theta = (WATER_SPEED**2 - math.sqrt(discriminant)) / (GRAVITY * distance)
    tilt_angle = math.degrees(math.atan(tan_theta))

    return (pan_angle, tilt_angle)

def set_servo_safe(servo, angle):
    '''舵机安全控制'''
    angle = max(-60, min(60, angle))  # 机械限位保护
    servo.angle(angle)

# ==================== 主控制循环 ====================
while True:
    clock.tick()
    img = sensor.snapshot().lens_corr(1.8)  # 镜头畸变校正

    # 目标检测
    blobs = img.find_blobs([COLOR_THRESHOLD],
                          roi=DETECT_ROI,
                          merge=True,)
    img.draw_rectangle(DETECT_ROI, color=(0,255,0))

    if blobs:
        target = max(blobs, key=lambda b: b.pixels(),roi=DETECT_ROI)
        img.draw_rectangle(target.rect(), color=(255,0,0))
        img.draw_cross(target.cx(), target.cy(), color=(0,255,255))

        # 坐标系转换
        world_x, world_y = pixel_to_world(target.cx(), target.cy())

        # 运动学计算
        pan_target, tilt_target = calculate_angles(world_x, world_y)

        # PID控制
        pan_error = pan_target - pan_servo.angle()
        tilt_error = tilt_target - tilt_servo.angle()

        pan_output = PAN_PID.get_pid(pan_error, 1/clock.fps())
        tilt_output = TILT_PID.get_pid(tilt_error, 1/clock.fps())

        # 更新舵机位置
        pan_servo.angle(pan_servo.angle() + pan_output)
        tilt_servo.angle(tilt_servo.angle() - tilt_output)

        # 启动水泵
        pump_pin.high()

        print(f"目标坐标 X:{world_x:.2f}m Y:{world_y:.2f}m")
        print(f"当前角度 水平:{pan_servo.angle():.1f}° 垂直:{tilt_servo.angle():.1f}°")
    else:
        pump_pin.low()
        print("等待目标...")

    print(f"帧率:{clock.fps():.1f} 水压:{WATER_SPEED}m/s")

# ==================== 标定说明 ====================
'''
系统标定流程：
1. 摄像头焦距标定：
   - 放置已知尺寸的标定板（如A4纸）
   - 测量实际物理尺寸与像素尺寸的比例
   - 计算焦距 FX = (像素宽度 * 实际距离) / 实际宽度

2. 水流速度标定：
   - 固定垂直喷射角度为45度
   - 测量水流落点距离
   - 通过公式 v0 = d / sqrt(2h/g) 计算初速度

3. 机械限位测试：
   - 逐步增加舵机角度直至机械限位
   - 记录最大安全角度范围

注意事项：
1. 定期检查喷嘴防止堵塞
2. 保持摄像头镜面清洁
3. 根据水压变化调整WATER_SPEED参数
4. 系统需防水处理
'''
