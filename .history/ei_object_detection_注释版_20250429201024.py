# Edge Impulse - OpenMV FOMO 目标检测示例
#
# 本代码遵循MIT许可协议
# 版权归OpenMV LLC所有
# 详情请访问：https://github.com/openmv/openmv/blob/master/LICENSE

# 导入必要的库
import sensor, image, time, ml, math, uos, gc

# 初始化摄像头传感器
sensor.reset()                         # 重置并初始化摄像头传感器
sensor.set_pixformat(sensor.RGB565)    # 设置像素格式为RGB565（每个像素16位）
sensor.set_framesize(sensor.QVGA)      # 设置帧尺寸为QVGA（320x240分辨率）
sensor.set_windowing((240, 240))       # 设置240x240的采集窗口（中心裁剪）
sensor.skip_frames(time=2000)          # 等待2秒让相机自动调整（自动曝光/白平衡）

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

# 主循环
while True:
    clock.tick()  # 开始帧计时
    img = sensor.snapshot()  # 捕获一帧图像

    # 使用模型进行预测，并传入后处理回调函数
    for i, detection_list in enumerate(net.predict([img], callback=fomo_post_process)):
        if i == 0: continue   # 跳过背景类别（索引0）
        if not detection_list: continue  # 无检测时跳过

        # ---- 在这里添加：仅对第一类（i==1）提取最大框中心点 ----
        if i == 1:
            # 按面积 (w*h) 找到最大的检测框
            x, y, w, h, score = max(detection_list, key=lambda b: b[2]*b[3])
            center_x = x + w // 2
            center_y = y + h // 2
            img.draw_rectangle((x, y, w, h), color=colors[i])
            img.draw_cross
            print("第二类最大检测框中心点：", center_x, center_y)
            # 如果只需要这个点，可以在这里跳出循环或做其他处理
            # break

        # 原有：打印并画框
        print("********** %s **********" % labels[i])
        for x, y, w, h, score in detection_list:
            cx = math.floor(x + w/2)
            cy = math.floor(y + h/2)
            print(f"x {cx}\ty {cy}\tscore {score:.2f}")
            img.draw_circle((cx, cy, 12), color=colors[i])
            img.draw_string(x, y, "%d%%" % (score * 100), color=colors[i], scale=2)

    # 打印当前帧率（每秒帧数）
    print(clock.fps(), "fps", end="\n\n")
