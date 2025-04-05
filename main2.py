'''
基于OpenMV的粪便检测与追踪系统
功能：通过颜色识别检测特定区域内的目标，使用双轴舵机进行追踪，并通过GPIO输出检测状态
硬件配置：OpenMV摄像头模块、双轴舵机云台、GPIO控制接口
'''

# 导入所需库
import sensor, image, time    # OpenMV图像处理库
from pid import PID           # PID控制算法库
from pyb import Servo, Pin    # 舵机和GPIO控制库

# ========================= 硬件初始化配置 =========================
# 配置GPIO引脚（用于输出检测状态）
pin4 = Pin('P4', Pin.OUT_PP)  # 粪便检测状态引脚
pin5 = Pin('P5', Pin.OUT_PP)  # 备用引脚（当前未使用）
pin4.low()                    # 初始化状态为低电平
pin5.low()

# 初始化舵机系统
pan_servo = Servo(1)   # 水平舵机（P7引脚）
tilt_servo = Servo(2)  # 垂直舵机（P8引脚）
# 舵机校准参数（脉冲宽度范围500-2500μs，误差补偿500μs）
pan_servo.calibration(500, 2500, 500)
tilt_servo.calibration(500, 2500, 500)

# ========================= 视觉参数配置 =========================
pig_roi = (55, 15, 420, 410)  # 检测区域(起始x,起始y,宽度,高度)
# 颜色阈值配置（LAB色彩空间）
red_threshold = (13, 49, 17, 62, 6, 47)       # 基础红色阈值
shift_threshold = (23, 52, -6, 22, 31, 53)    # 动态调整阈值
shift_threshold = red_threshold               # 当前使用阈值

# ========================= 控制系统配置 =========================
# PID控制器参数（比例项、积分项、微分项，积分限幅）

#pan_pid = PID(p=0.07, i=0,d=0.001 , imax=90) #脱机运行或者禁用图像传输，使用这个PID
#tilt_pid = PID(p=0.05, i=0, d=0.001 ,imax=90) #脱机运行或者禁用图像传输，使用这个PID
pan_pid = PID(p=0.05, i=0,d=0.001 ,imax=90)#在线调试使用这个PID
tilt_pid = PID(p=0.05, i=0,d=0.001, imax=90)#在线调试使用这个PID

# ========================= 摄像头初始化 =========================
sensor.reset()                      # 重置摄像头传感器
sensor.set_pixformat(sensor.RGB565) # 设置RGB565像素格式
sensor.set_framesize(sensor.HD)    # 设置VGA分辨率（640x480）
sensor.skip_frames(time=2000)       # 跳过初始不稳定帧（2秒）
sensor.set_auto_whitebal(False)     # 关闭自动白平衡
clock = time.clock()                # 创建时钟对象用于FPS计算

# ========================= 功能函数定义 =========================
def find_max(blobs):
    '''
    寻找最大色块函数
    参数：blobs - 检测到的色块列表
    返回：最大面积的色块对象
    '''
    max_size = 0
    for blob in blobs:
        current_size = blob.w() * blob.h()  # 计算色块面积
        if current_size > max_size:
            max_blob = blob
            max_size = current_size
    return max_blob if max_size > 0 else None

# ========================= 主控制循环 =========================
tilt_servo.angle(0)  # 初始化垂直舵机角度

while(True):
    clock.tick()  # 开始帧计时
    img = sensor.snapshot().lens_corr(1.6)  # 添加镜头校正

    # 在指定区域内进行色块检测
    blobs = img.find_blobs([shift_threshold], roi=pig_roi, merge=True)
    print("当前舵机角度 - 水平:", pan_servo.angle(), "垂直:", tilt_servo.angle())
    img.draw_rectangle(pig_roi, color=(0,255,0))  # 绘制检测区域

    if blobs:
        max_blob = find_max(blobs)
        if max_blob:
            # 计算中心点偏差（相对于图像中心）
            pan_error = max_blob.cx() - img.width()//2
            tilt_error = max_blob.cy() - img.height()//2

            # 在图像上标记目标
            img.draw_rectangle(max_blob.rect(), color=(255,0,0))  # 绘制边界框
            img.draw_cross(max_blob.cx(), max_blob.cy(), color=(0,0,255))  # 绘制中心点

            # PID控制计算
            pan_output = pan_pid.get_pid(pan_error, 1) / 2  # 水平轴输出
            tilt_output = tilt_pid.get_pid(tilt_error, 1)   # 垂直轴输出

            # 更新舵机位置
            pan_servo.angle(pan_servo.angle() + pan_output)
            tilt_servo.angle(tilt_servo.angle() - tilt_output)

            # 输出调试信息
            print("控制输出 - 水平:", pan_output, "垂直:", tilt_output)
            print("状态：检测到目标")
            pin4.high()  # 触发检测信号

    else:
        print("状态：未检测到目标")
        pin4.low()   # 关闭检测信号

    # 显示当前帧率（调试用）
    print("当前FPS:", clock.fps())

# ========================= 系统说明 =========================
'''
系统工作流程：
1. 摄像头采集640x480分辨率图像
2. 在预设的检测区域(pig_roi)内进行颜色阈值检测
3. 筛选出最大色块作为目标
4. 计算目标中心与画面中心的偏差
5. 通过PID算法计算舵机调整量
6. 根据检测结果控制GPIO输出状态

关键参数调整建议：
1. 颜色阈值：需根据实际环境光照和目标颜色调整
2. ROI区域：根据检测目标出现范围优化检测效率
3. PID参数：需根据云台惯性和响应速度调整
4. 舵机校准：需配合实际舵机型号设置脉冲宽度

硬件连接说明：
- 水平舵机：P7引脚
- 垂直舵机：P8引脚
- 状态输出：P4引脚（高电平表示检测到目标）
- 备用输出：P5引脚（当前未启用）

注意事项：
1. 保证充足的环境光照
2. 避免检测区域内有相似颜色的干扰物
3. 定期检查舵机机械结构的稳定性
4. 实际部署前需重新校准颜色阈值
'''
