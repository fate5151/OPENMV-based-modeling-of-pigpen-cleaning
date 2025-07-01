import sensor, image, time, ml, math, uos, gc
from pyb import Pin, LED, Servo
import ml
from ml.utils import NMS

# 用户配置 / User Configuration
# 设置摄像头图像 / Set up the camera
sensor.reset()
sensor.set_pixformat(sensor.RGB565)  # 颜色模式 / Color format
sensor.set_framesize(sensor.QVGA)    # 分辨率 320x240
sensor.set_windowing((240, 240))     # 裁剪为 240x240 （如果需要可调整）/ Crop to 240x240 (adjust as needed)
sensor.skip_frames(time=2000)        # 等待摄像头稳定 / Wait for camera

# 初始化时钟 / Init timer for FPS
clock = time.clock()

# 定义舵机 (水平与垂直) / Define servos (pan and tilt)
servo_pan = Servo(1)   # 舵机1：水平 / Servo1: pan (horizontal)
servo_tilt = Servo(2)  # 舵机2：垂直 / Servo2: tilt (vertical)
# 舵机初始位置 / Set initial position to center
pan_pos = 90
tilt_pos = 45
servo_pan.angle(pan_pos)   # 中心位置 / center
servo_tilt.angle(tilt_pos) # 中心位置 / center

# 定义水泵控制引脚 / Define pump control pin (e.g., P0)
pump = Pin('P0', Pin.OUT_PP)
pump.low()  # 确保水泵初始关闭 / Ensure pump is off at start

# 定义报警LED或蜂鸣器 (这里使用板载LED) / Alarm LED or buzzer (using onboard LED)
alarm_led = LED(3)  # 以板载LED3作为报警指示 / Using on-board LED3 for alarm

# 检测模式 / Detection mode: "color", "fomo" 或 "both"
DETECT_MODE = "both"  # 可切换为 "color" 或 "fomo" / switch to "color" or "fomo"

# 颜色阈值，用于基于颜色的检测（示例红色阈值, 用户可调整） / Color thresholds for blob detection (example: red)
color_thresholds = [(30, 100, 15, 127, 15, 127)]  # 阈值列表 / list of (L_min, L_max, A_min, A_max, B_min, B_max)

# 中心容差 / Center tolerance (pixels) for alignment
center_x = sensor.width() // 2
center_y = sensor.height() // 2
tol_x = 20  # 水平容差 / horizontal tolerance
tol_y = 20  # 垂直容差 / vertical tolerance

# FOMO 模型及阈值初始化 / Initialize FOMO model and threshold
min_confidence = 0.5
fomo_threshold = (math.ceil(min_confidence * 255), 255)
try:
    net = ml.Model("trained.tflite", load_to_fb=uos.stat('trained.tflite')[6] > (gc.mem_free() - (64*1024)))  # 加载TFLite FOMO模型文件 (示例名称, 替换为实际文件名)
    labels = [line.rstrip('\n') for line in open("labels.txt")]
except Exception as e:
    raise RuntimeError("FOMO 模型加载失败: {}".format(e))

# FOMO后处理函数 / FOMO post-processing callback
def fomo_post_process(model, inputs, outputs):
    n, oh, ow, oc = model.output_shape[0]
    x_scale = inputs[0].roi[2] / ow
    y_scale = inputs[0].roi[3] / oh
    scale = min(x_scale, y_scale)
    x_offset = int((inputs[0].roi[2] - (ow * scale)) / 2 + inputs[0].roi[0])
    y_offset = int((inputs[0].roi[3] - (oh * scale)) / 2 + inputs[0].roi[1])
    nms = NMS(ow, oh, inputs[0].roi)  # 非极大值抑制 / Non-maximum suppression
    for i in range(oc):
        heatmap = image.Image(outputs[0][0, :, :, i] * 255)
        blobs = heatmap.find_blobs([fomo_threshold],
                                   x_stride=1, y_stride=1, area_threshold=1, pixels_threshold=1)
        for b in blobs:
            x, y, w, h = b.rect()
            score = heatmap.get_statistics(roi=b.rect()).l_mean() / 255.0
            x = int(x * scale + x_offset)
            y = int(y * scale + y_offset)
            w = int(w * scale)
            h = int(h * scale)
            nms.add_bounding_box(x, y, x + w, y + h, score, i)
    return nms.get_bounding_boxes()

# 初始化定时器 / Timers for detection timeout and pump runtime
last_detect_time = time.ticks_ms()
pump_start_time = None
power_save = False  # 初始不在节能模式 / Start not in power-saving mode

# 主循环 / Main loop
while True:
    clock.tick()
    img = sensor.snapshot()  # 捕获图像 / Capture image

    target_found = False
    target_x = 0
    target_y = 0
    target_label = None
    target_score = 0.0
    target_area = 0

    # 基于颜色检测 / Color-based detection
    if DETECT_MODE in ("color", "both"):
        blobs = img.find_blobs(color_thresholds, merge=True)
        if blobs:
            max_blob = max(blobs, key=lambda b: b.area())
            bx, by = max_blob.cx(), max_blob.cy()
            area = max_blob.w() * max_blob.h()
            if area > target_area:
                target_found = True
                target_x = bx
                target_y = by
                target_label = "ColorBlob"
                target_score = 1.0  # 色块检测无置信度，此处设为1.0 / no confidence metric, use 1.0
                target_area = area

    # 基于FOMO检测 / FOMO-based detection
    if DETECT_MODE in ("fomo", "both"):
        for i, detection_list in enumerate(net.predict([img], callback=fomo_post_process)):
            if i == 0:
                continue  # 跳过背景类 / skip background class
            for (x, y, w, h, score) in detection_list:
                area = w * h
                if area > target_area:
                    target_found = True
                    target_x = x + w // 2
                    target_y = y + h // 2
                    target_label = labels[i] if labels else ("Class_%d" % i)
                    target_score = score
                    target_area = area

                    # 绘制检测框 / Draw detection box
                    img.draw_rectangle(x, y, w, h, color=(0, 255, 0), thickness=2)

    

    # 更新检测超时时间 / Update detection timestamp
    current_time = time.ticks_ms()
    if target_found:
        last_detect_time = current_time
        if power_save:
            power_save = False  # 退出节能模式 / Exit power-saving mode
        # 打印检测结果 / Print detection result
        print("检测到目标:", target_label, "坐标(%d,%d) 置信度%.2f" % (target_x, target_y, target_score))
        # 舵机PID控制跟踪 / Servo PID control for tracking
        error_x = target_x - center_x
        error_y = target_y - center_y
        # PID参数 (仅P控制) / PID parameters (P-only)
        Kp = 0.1  # 比例增益 (可调) / proportional gain (adjust as needed)
        pan_pos += int(Kp * error_x)
        tilt_pos += int(Kp * error_y)
        # 限制舵机角度范围 / Limit servo angles to [-90, 90]
        pan_pos = max(min(pan_pos, 90), -90)
        tilt_pos = max(min(tilt_pos, 90), -90)
        servo_pan.angle(pan_pos)
        servo_tilt.angle(tilt_pos)
        # 检查是否对准中心 / Check if aligned within tolerance
        if abs(error_x) <= tol_x and abs(error_y) <= tol_y:
            pump.high()  # 对准中心 -> 启动水泵 / Aligned -> turn on pump
            if pump_start_time is None:
                pump_start_time = current_time
        else:
            pump.low()
            pump_start_time = None
    else:
        # 无目标检测 / No target detected
        pump.low()
        pump_start_time = None
        # 超时30秒进入节能模式 / Enter power-saving after 30s timeout
        if time.ticks_diff(current_time, last_detect_time) > 30000:
            if not power_save:
                power_save = True
                # 舵机归位到中心 / Reset servos to center
                pan_pos = 0
                tilt_pos = 0
                servo_pan.angle(pan_pos)
                servo_tilt.angle(tilt_pos)

    # 水泵连续运行超时报警 / Pump continuous runtime alarm
    if pump_start_time is not None:
        if time.ticks_diff(current_time, pump_start_time) > 40000:
            print("警告: 水泵已连续运行40秒!")  # Warning message
            alarm_led.on()
            # 可选: 关闭水泵或其他处理 / (Optionally stop pump or take other action)
            pump_start_time = current_time  # 重置以避免持续报警 / Reset to avoid repeated alarm
        else:
            alarm_led.off()
    else:
        alarm_led.off()
