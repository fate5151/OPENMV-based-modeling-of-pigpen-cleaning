# laser_detection.py - UV激光点检测模块
# 实现蓝紫色激光点的高精度检测和质量评估

import sensor, image, time, math

class LaserDetection:
    """UV激光点检测类 - 实现多级检测策略和质量评估"""
    
    def __init__(self):
        # 蓝紫色激光检测阈值（HSV颜色空间）
        self.laser_thresholds = [
            # 主要蓝紫色阈值
            (220, 255, 30, 100, 30, 100),  # 蓝色范围
            (240, 280, 30, 100, 30, 100),  # 紫色范围
            (200, 240, 20, 80, 20, 80),    # 更宽泛的蓝色
        ]
        
        # RGB备用阈值（当HSV检测失效时使用）
        self.laser_thresholds_rgb = [
            (0, 100, 50, 255, 100, 255),   # 强蓝色
            (50, 150, 0, 100, 150, 255),   # 蓝紫色
            (20, 120, 20, 120, 200, 255),  # 宽泛蓝紫色
        ]
        
        # 激光点形状约束
        self.min_area = 5          # 最小面积（像素）
        self.max_area = 500        # 最大面积（像素）
        self.min_circularity = 0.3  # 最小圆形度
        self.max_aspect_ratio = 3.0  # 最大长宽比
        
        # 检测历史记录
        self.position_history = []
        self.max_history_length = 10
        self.confidence_history = []
        
        # 检测统计
        self.detection_stats = {
            'total_attempts': 0,
            'successful_detections': 0,
            'hsv_detections': 0,
            'rgb_detections': 0,
            'shape_filtered': 0,
            'confidence_filtered': 0
        }
        
        print("激光检测模块初始化完成")
        self._print_thresholds()
    
    def _print_thresholds(self):
        """打印检测阈值信息"""
        print("=== 激光检测阈值 ===")
        print("HSV阈值组:", len(self.laser_thresholds))
        for i, thresh in enumerate(self.laser_thresholds):
            print(f"  阈值{i+1}: {thresh}")
        print("RGB备用阈值组:", len(self.laser_thresholds_rgb))
        print("形状约束: 面积({}-{}), 圆形度>{}, 长宽比<{}".format(
            self.min_area, self.max_area, self.min_circularity, self.max_aspect_ratio))
        print("==================")
    
    def _validate_shape(self, blob):
        """
        验证色块形状是否符合激光点特征
        
        Args:
            blob: 检测到的色块
            
        Returns:
            tuple: (is_valid, confidence, details)
        """
        area = blob.pixels()
        
        # 面积检查
        if area < self.min_area or area > self.max_area:
            return False, 0.0, f"面积不符合({area})"
        
        # 圆形度检查
        try:
            circularity = 4 * math.pi * area / (blob.perimeter() * blob.perimeter())
            if circularity < self.min_circularity:
                return False, 0.1, f"圆形度不足({circularity:.2f})"
        except:
            circularity = 0.0
        
        # 长宽比检查
        aspect_ratio = max(blob.w(), blob.h()) / max(min(blob.w(), blob.h()), 1)
        if aspect_ratio > self.max_aspect_ratio:
            return False, 0.2, f"长宽比过大({aspect_ratio:.2f})"
        
        # 计算综合置信度
        area_score = min(1.0, area / 50.0)  # 面积评分
        circularity_score = min(1.0, circularity / 0.8)  # 圆形度评分
        aspect_score = max(0.0, 1.0 - (aspect_ratio - 1.0) / 2.0)  # 长宽比评分
        
        confidence = (area_score + circularity_score + aspect_score) / 3.0
        
        details = f"面积:{area}, 圆形度:{circularity:.2f}, 长宽比:{aspect_ratio:.2f}"
        return True, confidence, details
    
    def _detect_with_thresholds(self, img, thresholds, color_space="HSV"):
        """
        使用指定阈值组进行检测
        
        Args:
            img: 输入图像
            thresholds: 阈值列表
            color_space: 颜色空间 ("HSV" 或 "RGB")
            
        Returns:
            list: [(blob, confidence, threshold_index), ...]
        """
        all_candidates = []
        
        for i, threshold in enumerate(thresholds):
            try:
                # 根据颜色空间进行检测
                if color_space == "HSV":
                    # 转换为HSV并检测
                    hsv_img = img.to_hsv()
                    blobs = hsv_img.find_blobs([threshold], merge=True, area_threshold=self.min_area)
                else:
                    # RGB检测
                    blobs = img.find_blobs([threshold], merge=True, area_threshold=self.min_area)
                
                for blob in blobs:
                    # 形状验证
                    is_valid, confidence, details = self._validate_shape(blob)
                    if is_valid:
                        all_candidates.append((blob, confidence, i, details))
                        if color_space == "HSV":
                            self.detection_stats['hsv_detections'] += 1
                        else:
                            self.detection_stats['rgb_detections'] += 1
                    else:
                        self.detection_stats['shape_filtered'] += 1
                        
            except Exception as e:
                print(f"{color_space}检测阈值{i}失败: {e}")
                continue
        
        return all_candidates
    
    def _filter_by_history(self, candidates):
        """
        基于历史位置过滤候选点
        
        Args:
            candidates: 候选激光点列表
            
        Returns:
            list: 过滤后的候选点
        """
        if not self.position_history:
            return candidates
        
        # 计算历史位置的加权平均
        weights = [1.0 / (i + 1) for i in range(len(self.position_history))]
        total_weight = sum(weights)
        
        avg_x = sum(pos[0] * w for pos, w in zip(self.position_history, weights)) / total_weight
        avg_y = sum(pos[1] * w for pos, w in zip(self.position_history, weights)) / total_weight
        
        # 计算每个候选点与历史位置的距离
        filtered_candidates = []
        for blob, confidence, threshold_idx, details in candidates:
            distance = math.sqrt((blob.cx() - avg_x)**2 + (blob.cy() - avg_y)**2)
            
            # 距离权重（距离越近权重越高）
            max_distance = 100  # 最大允许距离
            if distance <= max_distance:
                distance_weight = 1.0 - (distance / max_distance)
                adjusted_confidence = confidence * (0.7 + 0.3 * distance_weight)
                filtered_candidates.append((blob, adjusted_confidence, threshold_idx, details, distance))
        
        return filtered_candidates
    
    def get_current_laser_position(self, img, debug=False):
        """
        获取当前激光点位置 - 主要检测函数
        
        Args:
            img: 输入图像
            debug: 是否显示调试信息
            
        Returns:
            dict: {
                'position': (x, y) 或 None,
                'confidence': 0.0-1.0,
                'method': 检测方法,
                'details': 详细信息,
                'fallback_used': 是否使用了回退策略
            }
        """
        self.detection_stats['total_attempts'] += 1
        
        # 第一级：HSV颜色空间检测
        hsv_candidates = self._detect_with_thresholds(img, self.laser_thresholds, "HSV")
        
        # 第二级：RGB颜色空间检测（HSV失效时）
        rgb_candidates = []
        if not hsv_candidates:
            rgb_candidates = self._detect_with_thresholds(img, self.laser_thresholds_rgb, "RGB")
        
        # 合并候选点
        all_candidates = hsv_candidates + rgb_candidates
        
        if not all_candidates:
            return self._handle_detection_failure(debug)
        
        # 基于历史位置过滤
        filtered_candidates = self._filter_by_history(all_candidates)
        
        if not filtered_candidates:
            self.detection_stats['confidence_filtered'] += 1
            return self._handle_detection_failure(debug)
        
        # 选择最佳候选点（置信度最高）
        best_candidate = max(filtered_candidates, key=lambda x: x[1])
        blob, confidence, threshold_idx, details = best_candidate[:4]
        
        position = (blob.cx(), blob.cy())
        
        # 更新历史记录
        self._update_history(position, confidence)
        
        # 绘制检测结果
        if debug:
            self._draw_detection_result(img, blob, confidence, details)
        
        self.detection_stats['successful_detections'] += 1
        
        method = "HSV" if threshold_idx < len(self.laser_thresholds) else "RGB"
        
        return {
            'position': position,
            'confidence': confidence,
            'method': method,
            'details': details,
            'fallback_used': False
        }
    
    def _handle_detection_failure(self, debug=False):
        """
        处理检测失效的情况
        
        Args:
            debug: 是否显示调试信息
            
        Returns:
            dict: 回退策略结果
        """
        if debug:
            print("激光点检测失效，使用回退策略")
        
        # 策略1：使用历史位置的平滑估算
        if len(self.position_history) >= 2:
            # 基于最近两个位置预测下一个位置
            pos1 = self.position_history[-1]
            pos2 = self.position_history[-2]
            
            # 简单线性预测
            predicted_x = pos1[0] + (pos1[0] - pos2[0]) * 0.5
            predicted_y = pos1[1] + (pos1[1] - pos2[1]) * 0.5
            
            # 计算置信度（基于历史一致性）
            consistency = self._calculate_history_consistency()
            
            return {
                'position': (int(predicted_x), int(predicted_y)),
                'confidence': consistency * 0.5,  # 回退策略的置信度较低
                'method': 'prediction',
                'details': '基于历史位置预测',
                'fallback_used': True
            }
        
        # 策略2：使用历史位置平均值
        elif self.position_history:
            avg_x = sum(pos[0] for pos in self.position_history) / len(self.position_history)
            avg_y = sum(pos[1] for pos in self.position_history) / len(self.position_history)
            
            return {
                'position': (int(avg_x), int(avg_y)),
                'confidence': 0.3,
                'method': 'average',
                'details': '历史位置平均值',
                'fallback_used': True
            }
        
        # 策略3：完全失效，返回None
        return {
            'position': None,
            'confidence': 0.0,
            'method': 'none',
            'details': '检测完全失效',
            'fallback_used': True
        }
    
    def _update_history(self, position, confidence):
        """更新位置和置信度历史"""
        self.position_history.append(position)
        self.confidence_history.append(confidence)
        
        # 限制历史长度
        if len(self.position_history) > self.max_history_length:
            self.position_history.pop(0)
            self.confidence_history.pop(0)
    
    def _calculate_history_consistency(self):
        """计算历史位置的一致性得分"""
        if len(self.position_history) < 2:
            return 0.5
        
        # 计算位置变化的标准差
        recent_positions = self.position_history[-5:]  # 最近5个位置
        
        x_values = [pos[0] for pos in recent_positions]
        y_values = [pos[1] for pos in recent_positions]
        
        x_std = self._calculate_std(x_values)
        y_std = self._calculate_std(y_values)
        
        # 标准差越小，一致性越高
        max_std = 50  # 最大允许标准差
        x_consistency = max(0.0, 1.0 - x_std / max_std)
        y_consistency = max(0.0, 1.0 - y_std / max_std)
        
        return (x_consistency + y_consistency) / 2.0
    
    def _calculate_std(self, values):
        """计算标准差"""
        if len(values) < 2:
            return 0.0
        
        mean = sum(values) / len(values)
        variance = sum((x - mean) ** 2 for x in values) / len(values)
        return math.sqrt(variance)
    
    def _draw_detection_result(self, img, blob, confidence, details):
        """绘制检测结果"""
        # 绘制激光点
        color = (0, 255, 0) if confidence > 0.7 else (255, 255, 0) if confidence > 0.4 else (255, 0, 0)
        
        # 绘制圆圈和十字
        img.draw_circle((blob.cx(), blob.cy(), 15), color=color, thickness=2)
        img.draw_cross((blob.cx(), blob.cy()), color=color, size=10, thickness=2)
        
        # 绘制置信度
        img.draw_string(blob.cx() + 20, blob.cy() - 10, f"C:{confidence:.2f}", color=color)
        
        # 绘制历史轨迹
        if len(self.position_history) > 1:
            for i in range(1, len(self.position_history)):
                x1, y1 = self.position_history[i-1]
                x2, y2 = self.position_history[i]
                img.draw_line((x1, y1, x2, y2), color=(100, 100, 100))
    
    def get_detection_stats(self):
        """获取检测统计信息"""
        total = max(1, self.detection_stats['total_attempts'])
        success_rate = self.detection_stats['successful_detections'] / total * 100
        
        return {
            'success_rate': success_rate,
            'total_attempts': self.detection_stats['total_attempts'],
            'successful_detections': self.detection_stats['successful_detections'],
            'hsv_detections': self.detection_stats['hsv_detections'],
            'rgb_detections': self.detection_stats['rgb_detections'],
            'shape_filtered': self.detection_stats['shape_filtered'],
            'confidence_filtered': self.detection_stats['confidence_filtered'],
            'history_length': len(self.position_history)
        }
    
    def print_detection_stats(self):
        """打印检测统计信息"""
        stats = self.get_detection_stats()
        print("=== 激光检测统计 ===")
        print(f"成功率: {stats['success_rate']:.1f}%")
        print(f"总尝试: {stats['total_attempts']}")
        print(f"成功检测: {stats['successful_detections']}")
        print(f"HSV检测: {stats['hsv_detections']}")
        print(f"RGB检测: {stats['rgb_detections']}")
        print(f"形状过滤: {stats['shape_filtered']}")
        print(f"置信度过滤: {stats['confidence_filtered']}")
        print(f"历史长度: {stats['history_length']}")
        print("==================")
    
    def adjust_thresholds(self, threshold_index, param_index, delta):
        """
        动态调整检测阈值
        
        Args:
            threshold_index: 阈值组索引
            param_index: 参数索引 (0-5: H_min, H_max, S_min, S_max, V_min, V_max)
            delta: 调整量
        """
        if 0 <= threshold_index < len(self.laser_thresholds):
            threshold = list(self.laser_thresholds[threshold_index])
            threshold[param_index] = max(0, min(255, threshold[param_index] + delta))
            self.laser_thresholds[threshold_index] = tuple(threshold)
            print(f"阈值{threshold_index}参数{param_index}调整为: {threshold[param_index]}")
    
    def reset_history(self):
        """重置历史记录"""
        self.position_history = []
        self.confidence_history = []
        print("历史记录已重置")