# 导入必要的库
import sensor, image, time, ml, math, uos, gc, pyb,camera_setup,display
#from pyb import UART
from my_uart import send_custom_packet
from utils import set_time, get_time_str, get_unix_timestamp


lcd = display.SPIDisplay()

# 目标分辨率
TARGET_W = 128
TARGET_H = 160

# 计算缩放参数
scale_x = TARGET_W / 320
scale_y = TARGET_H / 240
scale_ratio = min(scale_x, scale_y)  # 保持宽高比的最小比例

#---------------------参考点--------------------
x_under =139
y_under =212

#---------------------时间变量--------------------
rtc = pyb.RTC()

detection_start_time = None       # 第一次检测到目标的时间戳
last_detection_time = 0           # 最近一次检测到目标的时间戳
detection_active = False          # 当前是否正在检测中
detection_duration = 0            # 当前检测持续时长（ms）
detection_timeout = 1000          # 超过这个毫秒未检测到，认为结束（比如1秒）

last_stat_time = pyb.millis()  # 用于控制10秒打印一次统计
current_day = rtc.datetime()[2]  # 当前日期（天）
daily_report_sent = False  # 防止一天内多次发送
# 设置初始时间（年, 月, 日, 星期, 小时, 分钟, 秒）
set_time (2025, 6, 30,  13, 12, 0)
last_day = rtc.datetime()[2]  # 初始日期

# 每日统计变量初始化
daily_frame_count = 0           #每日处理的帧数：	总体工作量
daily_target_count = 0          #检测到FOMO目标的总数（有目标就加1）：反映目标出现频率
daily_red_detect_count = 0      #成功检测到红色辅助色块次数:	色块检测可靠性
daily_uart_send_count = 0       #成功发送数据帧的次数:	通信稳定性判断
daily_fail_count = 0            #未检测到红色色块/无目标导致未发送的次数:	检测失败评估
daily_blob_count = 0            #img.find_blobs()返回的红色区域总数:	用于分析误检情况
daily_largest_blob_pixels = 0   #所有检测中最大色块像素值（可能是当天最大）:	检测质量指标
daily_total_blob_pixels = 0     #所有红色色块累计像素值:	色块整体面积分析
daily_label_1_detects = 0       #类别1检测次数（检测框数量）:	类别使用频率
daily_label_2_detects = 0       #类别2检测次数（如有）:	同上，其他类别可类推



# -------------------- FOMO神经网络模型 --------------------

# 初始化模型相关变量
net = None      # 存储加载的机器学习模型
labels = None   # 存储分类标签
min_confidence = 0.8  # 最小置信度阈值（仅显示置信度高于80%的检测结果）

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
threshold_list = [(math.ceil(min_confidence * 255), 255)]

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


# 将基本参数发送到串口（stm32）,
send_custom_packet(0 ,[  (img.width() >> 8) & 0xff, img.width() &  0xff,\
                        (img.height() >> 8) & 0xff, img.height() &  0xff,\
                        (x_under >> 8) & 0xff, x_under &  0xff,\
                        (y_under >> 8) & 0xff, y_under &  0xff, ]  )

daily_uart_send_count += 1  # 串口发送 +1

# 循环外定义（初始化一次）
fail_send_limit = 25  # 串口发送失败次数限制（超过此次数不再发送）
fail_send_count = 0
found_class_2 = False  # 标记是否找到类别2（红色色块）

# 主循环
while True:
    current_time = rtc.datetime()  # 格式：(年,月,日,星期,时,分,秒,ms)
    hour = current_time[4]
    minute = current_time[5]
    clock.tick()  # 开始帧计时
    img = sensor.snapshot().lens_corr(1.8)  # 捕获一帧图像
    daily_frame_count += 1  # 帧数 +1
    pig_detected_count=0  # 初始化猪的检测计数
    max_pixels = 0     # 记录最大像素数
    target_blob = None # 存储目标色块对象
    blobs =[] # 初始化blobs列表
    shit_detected_count=0
    shidu=0
    wendu=0
    error_led = False

    keypoints=[(x_under, y_under,270)]  # 参考点坐标//三元组
    img.draw_keypoints(keypoints, size=10, color=(0, 255, 0))  # 绘制关键点（可选）

    # 使用模型进行预测，并传入后处理回调函数
    for i, detection_list in enumerate(net.predict([img], callback=fomo_post_process)):
        if i == 0: continue   # 跳过背景类别（索引0）
        if not detection_list: continue  # 无检测时跳过
        if i == 1 :
            pig_detected_count = len(detection_list)

            for x, y, w, h, score in detection_list:  #FOMO检测的目标物体参数
                img.draw_rectangle((x, y, w, h), color=colors[i])


        # ---- 在这里添加：仅对第一类（i==2）提取最大框中心点 ----
        if i == 2 :
            found_class_2 = True  # 标记已找到类别2
            fail_send_count = 0  # 如果检测到目标类2，重置失败发送次数
            daily_target_count += 1
            daily_label_1_detects += len(detection_list)
            shit_detected_count = len(detection_list)

            for x, y, w, h, score in detection_list:  #FOMO检测的目标物体参数
                img.draw_rectangle((x, y, w, h), color=colors[i])
               #-------------辅助色块检测--------------------------------------
                color_x = math.ceil(x - w )
                color_y = math.ceil(y - h)
                color_w = 3*w
                color_h = 3*h
                color_roi = (color_x, color_y, color_w, color_h)
                img.draw_rectangle(color_roi, color=(0, 0, 255))#蓝
                # 在检测蓝框内查找颜色
                # 颜色阈值（红框）
                shift_threshold = (30, 82, -19, 69, 34, 66)
                blobs = img.find_blobs([shift_threshold], roi=color_roi, merge=True)
                total_pixels = 0
                if blobs:
                    #img.draw_string(0, 0, f"number of blobs: {len(blobs)}", color=(0, 255, 0), thickness=2)  # 如果支持 f-string
                    for blob in blobs:
                        daily_blob_count += 1
                        daily_total_blob_pixels += blob.pixels()
                        # 比较像素数量，保留最大值，即找到最大的目标色块
                        if blob.pixels() > max_pixels:
                            max_pixels = blob.pixels()
                            target_blob = blob             # 输出结果
                            #在目标色块上绘制矩形红框
                            img.draw_rectangle(target_blob.rect(), color=(255, 0, 0))#红
                            #----------------------检测时间--------------------
                            now = pyb.millis()
                            if not detection_active:
                                detection_start_time = now      # 第一次检测
                                detection_active = True
                                print("[开始检测] 时间戳(ms):", detection_start_time)
                            last_detection_time = now          # 每次都更新最后一次检测时间

                if max_pixels > daily_largest_blob_pixels:
                    daily_largest_blob_pixels = max_pixels

            daily_red_detect_count += 1  # 红色辅助色块识别 +1

            # 在图像上绘制检测中心十字和圆圈（绿框）
            if target_blob :
                center_x = target_blob.cx()
                center_y = target_blob.cy()
                img.draw_circle((center_x, center_y, 12), color=colors[i])
                img.draw_cross((center_x, center_y), color=colors[i])
                # 在图像上绘制检测框
                # 打印最大检测框的中心点坐标
                print("第二类最大检测框中心点：", "cx:",center_x, "cy:", center_y)
                #print(detection_list)
                #print("x:",x,"y:",y,"w:",w,"h:",h)

                # 将中心点坐标发送到串口（stm32）
                send_custom_packet(1,[1 ,   (center_x >> 8) & 0xFF, center_x & 0xFF , \
                                            (center_y >> 8) & 0xFF, center_y & 0xFF,])
                daily_uart_send_count += 1  # 串口发送 +1
                print("日期:", get_time_str())
        elif detection_duration > 5000 and detection_duration < 10000:  # 如果不是类别2，且检测持续时间大于5秒
            found_class_2 = False
            daily_fail_count += 1
            # 如果只需要这个点，可以在这里跳出循环或做其他处理
            # break
    if  found_class_2 == False and fail_send_count < fail_send_limit:
        fail_send_count += 1
        send_custom_packet(1, [0])  # 发送0表示未检测到目标色块
        #data1 = bytearray([0xf1, 0, 0xe1])
        #uart.write(data1)
        daily_uart_send_count += 1
        daily_fail_count += 1
        print(f"未检测到红色色块，发送0 , (第{fail_send_count}次)")
    elif found_class_2 == False and fail_send_count >= fail_send_limit:
        print(f"未检测到红色色块，但已达到发送次数上限，不再发送,fail_send_count:{fail_send_count}")

        # 原有：打印并画框
        print("********** %s **********" % labels[i])
        for x, y, w, h, score in detection_list:
            cx = math.floor(x + w/2)
            cy = math.floor(y + h/2)
            #print(f"x {cx}\ty {cy}\tscore {score:.2f}")
            #print(detection_list)
            img.draw_rectangle((x, y, w, h), color=colors[i])
            #img.draw_string(x, y, "%d%%" % (score * 100), color=colors[i], scale=2)

    ## 打印当前帧率（每秒帧数）
    #print(clock.fps(), "fps", end="\n\n")
    # 如果目标检测中
    if detection_active:
        now = pyb.millis()
        if now - last_detection_time > detection_timeout:
            # 超过1秒未检测到目标，结束一次检测
            detection_active = False
            detection_duration = last_detection_time - detection_start_time
            print("[检测结束] 持续时间(ms):", detection_duration)
            # 可选：记录累计时长 / 保存 / 上报串口等
        else:
            # 正在检测中，可实时输出持续时间（可选）
            current_duration = now - detection_start_time
            print("当前检测持续(ms):", current_duration)

            #===========声光报警逻辑============
        if current_duration >=10000:
            error_led = True
            print("检测持续时间超过10秒，触发声光报警！")
        else :
            error_led = False
            #print("检测持续时间未超过10秒，无报警")

# 每隔5秒打印一次实时统计数据
    if pyb.millis() - last_stat_time >= 5000:  # 5000毫秒 = 5秒
        print("===== 实时监控数据 =======================================")
        print("日期:", rtc.datetime())
        print("串口发送次数:", daily_uart_send_count)
        #print(" 粪便数目:", len(blobs))
        print(" 粪便数目:", shit_detected_count)
        print("猪的数目：", pig_detected_count)


        if not detection_active:
            now = pyb.millis()
            detection_start_time = now
            current_duration = now - detection_start_time
            print("最近检测持续(ms):", current_duration)


        print("=========================================================")
        pig_detected_count=3
        # 将实时监控数据发送到串口（stm32） 粪便数目，猪的数目,声光报警的标志位：
        send_custom_packet(2, [ shit_detected_count // 0xff,   shit_detected_count % 0xff,\
                                pig_detected_count // 0xff,   pig_detected_count % 0xff,\
                                error_led \
                                ])
                                #rtc.datetime()[4] // 0xff,   rtc.datetime()[4] % 0xff, \
                                #rtc.datetime()[5] // 0xff,  rtc.datetime()[5] % 0xff,  \
                                #rtc.datetime()[6] // 0xff, rtc.datetime()[6] % 0xff,   \
                                #current_duration // 0xff, current_duration % 0xff,
        daily_uart_send_count += 1
        last_stat_time = pyb.millis()
        print(len(blobs))


#------------------------每日统计数据-------------
    if hour == 13 and not daily_report_sent:
        print("帧数:", daily_frame_count)
        print("=== [每日13:00统计报告] =====================================")
        print("FOMO目标次数:", daily_target_count)
        print("红色检测成功数:", daily_red_detect_count)
        print("红色检测失败数:", daily_fail_count)
        print("串口发送次数:", daily_uart_send_count)
        print("色块总数:", daily_blob_count)
        print("最大红色块像素数:", daily_largest_blob_pixels)
        print("累计红色像素数:", daily_total_blob_pixels)
        print("=========================================================")
        daily_report_sent = True

    if hour == 0:
        daily_report_sent = False

        # 将每日统计数据发送到串口（stm32）
        '''
        data3 = bytearray([
            0xf3,  # 帧头1
            daily_frame_count // 0xff, # 每日处理的帧数：	总体工作量
            daily_frame_count % 0xff,  #
            daily_blob_count // 0xff, #img.find_blobs()返回的红色区域总数:	用于分析误检情况
            daily_blob_count % 0xff,  #
            daily_total_blob_pixels // 0xff, #所有红色色块累计像素值:	色块整体面积分析
            daily_total_blob_pixels % 0xff,
            daily_largest_blob_pixels // 0xff,
            daily_largest_blob_pixels % 0xff, #所有检测中最大色块像素值（可能是当天最大）:	检测质量指标
            0xe3   # 帧尾
        ])
        uart.write(data3)
        print(f"每日统计数据:{data3}")
        '''

        send_custom_packet(3, [daily_frame_count // 0xff, daily_frame_count % 0xff, \
                                daily_blob_count // 0xff, daily_blob_count % 0xff, \
                                daily_total_blob_pixels // 0xff, daily_total_blob_pixels % 0xff, \
                                daily_largest_blob_pixels // 0xff, daily_largest_blob_pixels % 0xff])
        daily_uart_send_count += 1
        last_stat_time = pyb.millis()
#------------------------数据清零-------------
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

    # 打印当前帧率（每秒帧数）5
    print(clock.fps(), "fps", end="\n\n")
    wendu =32
    shidu =66
    # 发送给 LCD 显示
    lcd.write(img,  x_scale=scale_x, y_scale=scale_y,  )
      # 拍照并显示图像。
    img.draw_string(0, 0, f"number of shit: {shit_detected_count}", color=(0, 255, 0), thickness=2)  # 如果支持 f-string
    img.draw_string(0, 16, f"温度： {wendu} 度 ,湿度：{shidu } %", color=(0, 255, 0), thickness=2)  # 如果支持 f-string


