# 导入必要的库
import sensor, image, time, ml, math, uos, gc, pyb, camera_setup, display
#from pyb import UART
from my_uart import send_custom_packet
from utils import set_time, get_time_str, get_unix_timestamp
from gamma_controller import GammaController  # 导入按键控制器

# 常量定义
TARGET_W = 128
TARGET_H = 160
DETECTION_TIMEOUT = 1000  # 检测超时时间（ms）
ALARM_THRESHOLD = 10000   # 声光报警阈值（ms）
FAIL_SEND_LIMIT = 25      # 串口发送失败次数限制
STAT_INTERVAL = 5000      # 统计数据打印间隔（ms）
DAILY_REPORT_HOUR = 13    # 每日报告时间（小时）
MIN_CONFIDENCE = 0.8      # 最小置信度阈值

lcd = display.SPIDisplay()

# 创建按键控制器实例
gamma_ctrl = GammaController()
gamma_ctrl.print_controls()  # 显示控制说明

# 计算缩放参数
scale_x = TARGET_W / 320
scale_y = TARGET_H / 240
scale_ratio = min(scale_x, scale_y)  # 保持宽高比的最小比例

#---------------------参考点--------------------
x_under = 141
y_under = 215

#---------------------时间变量--------------------
rtc = pyb.RTC()

detection_start_time = None       # 第一次检测到目标的时间戳
last_detection_time = 0           # 最近一次检测到目标的时间戳
detection_active = False          # 当前是否正在检测中
detection_duration = 0            # 当前检测持续时长（ms）
last_detection_duration = 0       # 上次检测持续时长（ms）

last_stat_time = pyb.millis()     # 用于控制统计打印间隔
current_day = rtc.datetime()[2]   # 当前日期（天）
daily_report_sent = False         # 防止一天内多次发送

# 设置初始时间（年, 月, 日, 星期, 小时, 分钟, 秒）
set_time(2025, 6, 30, 13, 12, 0)
last_day = rtc.datetime()[2]      # 初始日期

# 每日统计变量初始化
daily_frame_count = 0             # 每日处理的帧数
daily_target_count = 0            # 检测到FOMO目标的总数
daily_red_detect_count = 0        # 成功检测到红色辅助色块次数
daily_uart_send_count = 0         # 成功发送数据帧的次数
daily_fail_count = 0              # 未检测到红色色块导致未发送的次数
daily_blob_count = 0              # 红色区域总数
daily_largest_blob_pixels = 0     # 所有检测中最大色块像素值
daily_total_blob_pixels = 0       # 所有红色色块累计像素值
daily_label_1_detects = 0         # 类别1检测次数
daily_label_2_detects = 0         # 类别2检测次数

error_led = False                 # 声光报警状态

# -------------------- FOMO神经网络模型 --------------------

# 初始化模型相关变量
net = None      # 存储加载的机器学习模型
labels = None   # 存储分类标签

try:
    # 尝试加载TensorFlow Lite模型文件
    # 根据剩余内存决定是否将模型加载到帧缓冲区（提升性能）
    net = ml.Model("trained.tflite",
                  load_to_fb=uos.stat('trained.tflite')[6] > (gc.mem_free() - (64*1024)))
except Exception as e:
    raise Exception('加载"trained.tflite"失败，请确认文件已拷贝到设备存储 (' + str(e) + ')')

try:
    # 读取分类标签文件
    labels = [line.rstrip('\n') for line in open("labels.txt")]
except Exception as e:
    raise Exception('加载"labels.txt"失败，请确认文件已拷贝到设备存储 (' + str(e) + ')')

# 为不同检测类别定义显示颜色（最多支持7个类别）
colors = [
    (255, 0, 0),    # 红色
    (0, 255, 0),    # 绿色
    (255, 255, 0),  # 黄色
    (0, 0, 255),    # 蓝色
    (255, 0, 255),  # 品红
    (0, 255, 255),  # 青色
    (255, 255, 255),# 白色
]

# 将置信度阈值转换为0-255范围（用于后续图像处理）
threshold_list = [(math.ceil(MIN_CONFIDENCE * 255), 255)]

# FOMO模型后处理函数（将模型输出转换为检测框）
def fomo_post_process(model, inputs, outputs):
    # 获取输出张量维度 (batch_size, height, width, channels)
    ob, oh, ow, oc = model.output_shape[0]

    # 计算输入图像到输出特征图的缩放比例
    x_scale = inputs[0].roi[2] / ow  # 宽度缩放比
    y_scale = inputs[0].roi[3] / oh  # 高度缩放比
    scale = min(x_scale, y_scale)     # 取较小比例保持宽高比

    # 计算特征图在原始图像中的偏移量（用于坐标映射）
    x_offset = ((inputs[0].roi[2] - (ow * scale)) / 2) + inputs[0].roi[0]
    y_offset = ((inputs[0].roi[3] - (ow * scale)) / 2) + inputs[0].roi[1]

    # 初始化检测结果列表（每个类别一个子列表）
    l = [[] for _ in range(oc)]

    # 遍历每个输出通道（对应不同类别）
    for i in range(oc):
        # 将模型输出转换为0-255范围的图像
        img = image.Image(outputs[0][0, :, :, i] * 255)
        # 在输出图像中查找高置信度区域（blob检测）
        blobs = img.find_blobs(
            threshold_list,
            x_stride=1,        # 水平检测步长
            y_stride=1,        # 垂直检测步长
            area_threshold=1,  # 最小区域阈值
            pixels_threshold=1 # 最小像素数阈值
        )
        # 处理每个检测到的区域
        for b in blobs:
            rect = b.rect()  # 获取区域边界框
            x, y, w, h = rect
            # 计算区域平均置信度（归一化到0-1）
            score = img.get_statistics(
                thresholds=threshold_list,
                roi=rect
            ).l_mean() / 255.0

            # 将特征图坐标映射回原始图像坐标
            x = int((x * scale) + x_offset)
            y = int((y * scale) + y_offset)
            w = int(w * scale)
            h = int(h * scale)

            # 将检测结果添加到对应类别的列表
            l[i].append((x, y, w, h, score))
    return l  # 返回所有类别的检测结果

# 初始化帧率计时器
clock = time.clock()

img = sensor.snapshot().lens_corr(1.8)  # 捕获一帧图像

# 将基本参数发送到串口（stm32）
send_custom_packet(0, [(img.width() >> 8) & 0xff, img.width() & 0xff,
                       (img.height() >> 8) & 0xff, img.height() & 0xff,
                       (x_under >> 8) & 0xff, x_under & 0xff,
                       (y_under >> 8) & 0xff, y_under & 0xff])

daily_uart_send_count += 1  # 串口发送 +1

# 循环外定义（初始化一次）
fail_send_count = 0

# 主循环
while True:
    # 检查按键状态
    gamma_ctrl.check_buttons()

    current_time = rtc.datetime()  # 格式：(年,月,日,星期,时,分,秒,ms)
    hour = current_time[4]
    minute = current_time[5]
    current_day = current_time[2]

    clock.tick()  # 开始帧计时

    # 捕获图像并应用gamma校正
    img = sensor.snapshot().lens_corr(1.8)
    img = gamma_ctrl.apply_gamma_correction(img)  # 应用gamma校正
    img = img.histeq(adaptive=True, clip_limit=3)  # 直方图均衡化

    daily_frame_count += 1  # 帧数 +1
    pig_detected_count = 0  # 初始化猪的检测计数
    max_pixels = 0          # 记录最大像素数
    target_blob = None      # 存储目标色块对象
    blobs = []              # 初始化blobs列表
    shit_detected_count = 0

    found_class_2 = False   # 标记是否找到类别2（红色色块）

    keypoints = [(x_under, y_under, 270)]  # 参考点坐标
    img.draw_keypoints(keypoints, size=10, color=(0, 255, 0))  # 绘制关键点

    # 使用模型进行预测，并传入后处理回调函数
    for i, detection_list in enumerate(net.predict([img], callback=fomo_post_process)):
        if i == 0: continue   # 跳过背景类别（索引0）
        if not detection_list: continue  # 无检测时跳过

        # 确保颜色索引不越界
        color_index = min(i, len(colors) - 1)

        if i == 1:
            pig_detected_count = len(detection_list)
            for x, y, w, h, score in detection_list:  # FOMO检测的目标物体参数
                img.draw_rectangle((x, y, w, h), color=colors[color_index])

        # 处理类别2（粪便检测）
        if i == 2:
            found_class_2 = True  # 标记已找到类别2
            fail_send_count = 0   # 如果检测到目标类2，重置失败发送次数

            daily_target_count += 1
            daily_label_1_detects += len(detection_list)
            shit_detected_count = len(detection_list)

            for x, y, w, h, score in detection_list:  # FOMO检测的目标物体参数
                img.draw_rectangle((x, y, w, h), color=colors[color_index])

                # 辅助色块检测
                color_x = max(0, math.ceil(x - w))
                color_y = max(0, math.ceil(y - h))
                color_w = min(img.width() - color_x, 3*w)
                color_h = min(img.height() - color_y, 3*h)
                color_roi = (color_x, color_y, color_w, color_h)
                img.draw_rectangle(color_roi, color=(0, 0, 255))  # 蓝色检测区域

                # 颜色阈值（红色）
                shift_threshold = (30, 82, -19, 69, 34, 66)
                blobs = img.find_blobs([shift_threshold], roi=color_roi, merge=True)

                if blobs:
                    for blob in blobs:
                        daily_blob_count += 1
                        daily_total_blob_pixels += blob.pixels()

                        # 比较像素数量，保留最大值
                        if blob.pixels() > max_pixels:
                            max_pixels = blob.pixels()
                            target_blob = blob
                            # 在目标色块上绘制矩形红框
                            img.draw_rectangle(target_blob.rect(), color=(255, 0, 0))  # 红色

                            # 检测时间逻辑
                            now = pyb.millis()
                            if not detection_active:
                                detection_start_time = now
                                detection_active = True
                                print("[开始检测] 时间戳(ms):", detection_start_time)
                            last_detection_time = now

                if max_pixels > daily_largest_blob_pixels:
                    daily_largest_blob_pixels = max_pixels

            daily_red_detect_count += 1  # 红色辅助色块识别 +1

            # 在图像上绘制检测中心十字和圆圈
            if target_blob:
                center_x = target_blob.cx()
                center_y = target_blob.cy()
                img.draw_circle((center_x, center_y, 12), color=colors[color_index])
                img.draw_cross((center_x, center_y), color=colors[color_index])

                print("第二类最大检测框中心点：", "cx:", center_x, "cy:", center_y)

                # 将中心点坐标发送到串口（stm32）
                if not error_led:  # 如果没有触发声光报警
                    send_custom_packet(1, [1, (center_x >> 8) & 0xFF, center_x & 0xFF,
                                          (center_y >> 8) & 0xFF, center_y & 0xFF])
                    daily_uart_send_count += 1
                print("日期:", get_time_str())

    print(f"是否识别到粪便：{found_class_2}，失败次数：{fail_send_count}，限制：{FAIL_SEND_LIMIT}")

    # 处理未检测到目标的情况
    if not found_class_2 and fail_send_count < FAIL_SEND_LIMIT:
        fail_send_count += 1
        send_custom_packet(1, [0])  # 发送0表示未检测到目标色块
        daily_uart_send_count += 1
        daily_fail_count += 1
        print(f"未检测到红色色块，发送0 (第{fail_send_count}次)")
    elif not found_class_2 and fail_send_count >= FAIL_SEND_LIMIT:
        print(f"未检测到红色色块，但已达到发送次数上限，不再发送 fail_send_count:{fail_send_count}")

    # 检测时间逻辑处理
    current_duration = 0
    if detection_active:
        now = pyb.millis()
        current_duration = now - detection_start_time

        if now - last_detection_time > DETECTION_TIMEOUT:
            # 超过超时时间未检测到目标，结束一次检测
            detection_active = False
            last_detection_duration = last_detection_time - detection_start_time
            print("[检测结束] 持续时间(ms):", last_detection_duration)
        else:
            # 正在检测中，输出当前持续时间
            print("当前检测持续(ms):", current_duration)

    # 声光报警逻辑
    if detection_active and current_duration >= ALARM_THRESHOLD:
        error_led = True
        print("检测持续时间超过10秒，触发声光报警！")
    elif not detection_active:
        error_led = False

    # 每隔指定时间打印一次实时统计数据
    if pyb.millis() - last_stat_time >= STAT_INTERVAL:
        print("===== 实时监控数据 =======================================")
        print("日期:", rtc.datetime())
        print("串口发送次数:", daily_uart_send_count)
        print("粪便数目:", shit_detected_count)
        print("猪的数目：", pig_detected_count)
        print(f"声光报警:{error_led}")
        print("=========================================================")

        # 将实时监控数据发送到串口（stm32）
        send_custom_packet(2, [(shit_detected_count >> 8) & 0xff, shit_detected_count & 0xff,
                               (pig_detected_count >> 8) & 0xff, pig_detected_count & 0xff,
                               1 if error_led else 0])
        daily_uart_send_count += 1
        last_stat_time = pyb.millis()

        # 调试输出
        print("色块数量:", len(blobs))

    # 每日统计数据报告
    if hour == DAILY_REPORT_HOUR and not daily_report_sent:
        print("=== [每日13:00统计报告] =====================================")
        print("帧数:", daily_frame_count)
        print("FOMO目标次数:", daily_target_count)
        print("红色检测成功数:", daily_red_detect_count)
        print("红色检测失败数:", daily_fail_count)
        print("串口发送次数:", daily_uart_send_count)
        print("色块总数:", daily_blob_count)
        print("最大红色块像素数:", daily_largest_blob_pixels)
        print("累计红色像素数:", daily_total_blob_pixels)
        print("=========================================================")
        daily_report_sent = True

    # 重置每日报告标记
    if hour == 0:
        daily_report_sent = False
        # 发送每日统计数据
        send_custom_packet(3, [(daily_frame_count >> 8) & 0xff, daily_frame_count & 0xff,
                               (daily_blob_count >> 8) & 0xff, daily_blob_count & 0xff,
                               (daily_total_blob_pixels >> 8) & 0xff, daily_total_blob_pixels & 0xff,
                               (daily_largest_blob_pixels >> 8) & 0xff, daily_largest_blob_pixels & 0xff])
        daily_uart_send_count += 1

    # 每日数据清零
    if current_day != last_day:
        daily_frame_count = 0
        daily_target_count = 0
        daily_red_detect_count = 0
        daily_uart_send_count = 0
        daily_fail_count = 0
        daily_blob_count = 0
        daily_largest_blob_pixels = 0
        daily_total_blob_pixels = 0
        daily_label_1_detects = 0
        daily_label_2_detects = 0
        last_day = current_day
        print("== 新的一天，统计数据已清零 ==")

    # 在图像上绘制信息
    img.draw_string(0, 0, f"Shit: {shit_detected_count}", color=(0, 255, 0), thickness=2)
    img.draw_string(0, 20, f"Pig: {pig_detected_count}", color=(0, 255, 0), thickness=2)

    # 显示gamma控制状态
    status_text = gamma_ctrl.get_status_text()
    img.draw_string(10, 60, status_text, color=(255, 255, 255), scale=1)

    # 打印当前帧率
    print(f"FPS: {clock.fps():.2f}", end="\n\n")
