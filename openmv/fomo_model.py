# fomo_model.py - FOMO神经网络模型模块

import ml
import uos
import gc
import math
import image

class FOMOModel:
    """
    FOMO（Faster Objects, More Objects）神经网络模型封装类
    用于目标检测和分类
    """
    
    def __init__(self, model_path="trained.tflite", labels_path="labels.txt", min_confidence=0.8):
        """
        初始化FOMO模型
        
        Args:
            model_path (str): 模型文件路径
            labels_path (str): 标签文件路径
            min_confidence (float): 最小置信度阈值 (0.0-1.0)
        """
        self.model_path = model_path
        self.labels_path = labels_path
        self.min_confidence = min_confidence
        self.net = None
        self.labels = None
        self.threshold_list = [(math.ceil(min_confidence * 255), 255)]
        
        # 为不同检测类别定义显示颜色
        self.colors = [
            (255, 0, 0),    # 红色
            (0, 255, 0),    # 绿色
            (255, 255, 0),  # 黄色
            (0, 0, 255),    # 蓝色
            (255, 0, 255),  # 品红
            (0, 255, 255),  # 青色
            (255, 255, 255),# 白色
        ]
        
        # 加载模型和标签
        self._load_model()
        self._load_labels()
    
    def _load_model(self):
        """加载TensorFlow Lite模型"""
        try:
            # 根据剩余内存决定是否将模型加载到帧缓冲区
            load_to_fb = uos.stat(self.model_path)[6] > (gc.mem_free() - (64*1024))
            self.net = ml.Model(self.model_path, load_to_fb=load_to_fb)
            print(f"模型加载成功: {self.model_path}")
        except Exception as e:
            raise Exception(f'加载模型"{self.model_path}"失败: {str(e)}')
    
    def _load_labels(self):
        """加载分类标签"""
        try:
            with open(self.labels_path, 'r') as f:
                self.labels = [line.rstrip('\n') for line in f]
            print(f"标签加载成功: {self.labels}")
        except Exception as e:
            raise Exception(f'加载标签文件"{self.labels_path}"失败: {str(e)}')
    
    def get_color(self, class_id):
        """
        获取指定类别的颜色
        
        Args:
            class_id (int): 类别ID
            
        Returns:
            tuple: RGB颜色值
        """
        return self.colors[min(class_id, len(self.colors) - 1)]
    
    def post_process(self, model, inputs, outputs):
        """
        FOMO模型后处理函数，将模型输出转换为检测框
        
        Args:
            model: 模型对象
            inputs: 输入数据
            outputs: 模型输出
            
        Returns:
            list: 每个类别的检测结果列表
        """
        # 获取输出张量维度 (batch_size, height, width, channels)
        ob, oh, ow, oc = model.output_shape[0]

        # 计算输入图像到输出特征图的缩放比例
        x_scale = inputs[0].roi[2] / ow
        y_scale = inputs[0].roi[3] / oh
        scale = min(x_scale, y_scale)

        # 计算特征图在原始图像中的偏移量
        x_offset = ((inputs[0].roi[2] - (ow * scale)) / 2) + inputs[0].roi[0]
        y_offset = ((inputs[0].roi[3] - (oh * scale)) / 2) + inputs[0].roi[1]

        # 初始化检测结果列表
        detection_results = [[] for _ in range(oc)]

        # 遍历每个输出通道（对应不同类别）
        for class_id in range(oc):
            # 将模型输出转换为0-255范围的图像
            confidence_map = image.Image(outputs[0][0, :, :, class_id] * 255)
            
            # 在置信度图中查找高置信度区域
            blobs = confidence_map.find_blobs(
                self.threshold_list,
                x_stride=1,
                y_stride=1,
                area_threshold=1,
                pixels_threshold=1
            )
            
            # 处理每个检测到的区域
            for blob in blobs:
                rect = blob.rect()
                x, y, w, h = rect
                
                # 计算区域平均置信度
                score = confidence_map.get_statistics(
                    thresholds=self.threshold_list,
                    roi=rect
                ).l_mean() / 255.0

                # 将特征图坐标映射回原始图像坐标
                x = int((x * scale) + x_offset)
                y = int((y * scale) + y_offset)
                w = int(w * scale)
                h = int(h * scale)

                # 添加检测结果
                detection_results[class_id].append({
                    'bbox': (x, y, w, h),
                    'score': score,
                    'class_id': class_id,
                    'class_name': self.labels[class_id] if class_id < len(self.labels) else f'class_{class_id}'
                })

        return detection_results
    
    def predict(self, img):
        """
        对输入图像进行预测
        
        Args:
            img: 输入图像
            
        Returns:
            list: 检测结果列表
        """
        if self.net is None:
            raise Exception("模型未加载")
        
        return self.net.predict([img], callback=self.post_process)
    
    def draw_detections(self, img, detections, draw_background=False):
        """
        在图像上绘制检测结果
        
        Args:
            img: 输入图像
            detections: 检测结果
            draw_background (bool): 是否绘制背景类别
            
        Returns:
            dict: 各类别的检测统计
        """
        stats = {}
        
        for class_id, detection_list in enumerate(detections):
            # 跳过背景类别
            if class_id == 0 and not draw_background:
                continue
                
            if not detection_list:
                continue
            
            stats[class_id] = {
                'count': len(detection_list),
                'class_name': self.labels[class_id] if class_id < len(self.labels) else f'class_{class_id}',
                'detections': detection_list
            }
            
            # 绘制检测框
            color = self.get_color(class_id)
            for detection in detection_list:
                x, y, w, h = detection['bbox']
                score = detection['score']
                
                # 绘制边界框
                img.draw_rectangle((x, y, w, h), color=color, thickness=2)
                
                # 绘制类别标签和置信度
                label = f"{detection['class_name']}: {score:.2f}"
                img.draw_string(x, y-15, label, color=color, scale=1)
        
        return stats
    
    def get_best_detection(self, detections, class_id, criteria='score'):
        """
        获取指定类别的最佳检测结果
        
        Args:
            detections: 检测结果
            class_id (int): 类别ID
            criteria (str): 筛选标准 ('score', 'area', 'x', 'y')
            
        Returns:
            dict: 最佳检测结果，如果没有检测到则返回None
        """
        if class_id >= len(detections) or not detections[class_id]:
            return None
        
        detection_list = detections[class_id]
        
        if criteria == 'score':
            return max(detection_list, key=lambda d: d['score'])
        elif criteria == 'area':
            return max(detection_list, key=lambda d: d['bbox'][2] * d['bbox'][3])
        elif criteria == 'x':
            return max(detection_list, key=lambda d: d['bbox'][0])
        elif criteria == 'y':
            return max(detection_list, key=lambda d: d['bbox'][1])
        else:
            return detection_list[0]
    
    def filter_detections(self, detections, min_confidence=None, min_area=None):
        """
        过滤检测结果
        
        Args:
            detections: 检测结果
            min_confidence (float): 最小置信度阈值
            min_area (int): 最小面积阈值
            
        Returns:
            list: 过滤后的检测结果
        """
        filtered_detections = []
        
        for class_detections in detections:
            filtered_class = []
            for detection in class_detections:
                # 置信度过滤
                if min_confidence and detection['score'] < min_confidence:
                    continue
                
                # 面积过滤
                if min_area:
                    x, y, w, h = detection['bbox']
                    area = w * h
                    if area < min_area:
                        continue
                
                filtered_class.append(detection)
            
            filtered_detections.append(filtered_class)
        
        return filtered_detections
    
    def get_detection_center(self, detection):
        """
        获取检测框的中心点坐标
        
        Args:
            detection (dict): 检测结果
            
        Returns:
            tuple: (center_x, center_y)
        """
        x, y, w, h = detection['bbox']
        return (x + w // 2, y + h // 2)
    
    def set_confidence_threshold(self, threshold):
        """
        设置置信度阈值
        
        Args:
            threshold (float): 新的置信度阈值 (0.0-1.0)
        """
        self.min_confidence = threshold
        self.threshold_list = [(math.ceil(threshold * 255), 255)]
    
    def get_model_info(self):
        """
        获取模型信息
        
        Returns:
            dict: 模型信息
        """
        if self.net is None:
            return None
        
        return {
            'model_path': self.model_path,
            'labels_path': self.labels_path,
            'min_confidence': self.min_confidence,
            'num_classes': len(self.labels) if self.labels else 0,
            'labels': self.labels,
            'input_shape': self.net.input_shape,
            'output_shape': self.net.output_shape
        }
    
    def __del__(self):
        """析构函数，释放模型资源"""
        if self.net:
            del self.net
            self.net = None