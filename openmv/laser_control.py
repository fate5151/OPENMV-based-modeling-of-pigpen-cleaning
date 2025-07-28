# laser_control.py - UV激光控制模块
# 实现激光位置控制和自适应PID调节

import time, pyb, math
from pid import PID

class LaserControl:
    """UV激光控制类 - 实现自适应PID控制和激光位置管理"""
    
    def __init__(self, servo_pan_pin=1, servo_tilt_pin=2):
        # 初始化舵机
        try:
            self.pan_servo = pyb.Servo(servo_pan_pin)
            self.tilt_servo = pyb.Servo(servo_tilt_pin)
            
            # 舵机校准
            self.pan_servo.calibration(500, 2500, 500)
            self.tilt_servo.calibration(500, 2500, 500)
            
            self.servo_available = True
            print(f"舵机初始化成功: Pan={servo_pan_pin}, Tilt={servo_tilt_pin}")
        except Exception as e:
            print(f"舵机初始化失败: {e}")
            self.servo_available = False
        
        # 初始化PID控制器
        self.init_pid_controllers()
        
        # 控制参数
        self.target_position = (160, 120)  # 目标位置（图像中心）
        self.current_position = (160, 120)  # 当前位置
        self.last_update_time = 0
        
        # 控制限制
        self.max_servo_angle = 90
        self.min_servo_angle = -90
        self.servo_speed_limit = 10  # 每次最大角度变化
        
        # 自适应控制参数
        self.adaptive_control = True
        self.confidence_threshold_high = 0.8
        self.confidence_threshold_low = 0.4
        
        # 控制历史
        self.control_history = []
        self.max_control_history = 20
        
        # 性能统计
        self.control_stats = {
            'control_attempts': 0,
            'successful_controls': 0,
            'adaptive_adjustments': 0,
            'servo_limits_hit': 0,
            'average_error': 0.0,
            'last_error': 0.0
        }
        
        print("激光控制模块初始化完成")
        self._print_control_params()
    
    def init_pid_controllers(self):
        """初始化PID控制器"""
        # 基础PID参数（高置信度时使用）
        self.base_pan_pid = PID(p=0.05, i=0.001, d=0.01, imax=50)
        self.base_tilt_pid = PID(p=0.05, i=0.001, d=0.01, imax=50)
        
        # 保守PID参数（低置信度时使用）
        self.conservative_pan_pid = PID(p=0.02, i=0.0005, d=0.005, imax=30)
        self.conservative_tilt_pid = PID(p=0.02, i=0.0005, d=0.005, imax=30)
        
        # 激进PID参数（高精度需求时使用）
        self.aggressive_pan_pid = PID(p=0.08, i=0.002, d=0.02, imax=80)
        self.aggressive_tilt_pid = PID(p=0.08, i=0.002, d=0.02, imax=80)
        
        # 当前使用的PID控制器
        self.current_pan_pid = self.base_pan_pid
        self.current_tilt_pid = self.base_tilt_pid
        
        print("PID控制器初始化完成")
    
    def _print_control_params(self):
        """打印控制参数"""
        print("=== 激光控制参数 ===")
        print(f"目标位置: {self.target_position}")
        print(f"舵机可用: {self.servo_available}")
        print(f"自适应控制: {self.adaptive_control}")
        print(f"角度限制: [{self.min_servo_angle}, {self.max_servo_angle}]")
        print(f"速度限制: {self.servo_speed_limit}度/次")
        print("==================")
    
    def set_target_position(self, x, y):
        """
        设置目标位置
        
        Args:
            x, y: 目标位置坐标
        """
        self.target_position = (x, y)
        print(f"目标位置更新为: ({x}, {y})")
    
    def get_current_laser_position(self, laser_detection_result):
        """
        获取当前激光位置（从激光检测结果中提取）
        
        Args:
            laser_detection_result: 激光检测模块的返回结果
            
        Returns:
            tuple: (x, y) 位置坐标，如果检测失效返回历史位置估算
        """
        if laser_detection_result['position'] is not None:
            self.current_position = laser_detection_result['position']
            return self.current_position
        
        # 检测失效时的位置估算
        if laser_detection_result['fallback_used']:
            # 如果检测模块提供了回退位置，使用它
            if laser_detection_result['position'] is not None:
                estimated_pos = laser_detection_result['position']
            else:
                # 使用控制历史进行估算
                estimated_pos = self._estimate_position_from_control_history()
            
            print(f"检测失效，使用估算位置: {estimated_pos}")
            return estimated_pos
        
        return self.current_position
    
    def _estimate_position_from_control_history(self):
        """基于控制历史估算当前位置"""
        if not self.control_history:
            return self.current_position
        
        # 基于最近的控制输出预测位置变化
        recent_controls = self.control_history[-5:]  # 最近5次控制
        
        if len(recent_controls) >= 2:
            # 计算平均控制变化
            avg_pan_change = sum(ctrl['pan_output'] for ctrl in recent_controls) / len(recent_controls)
            avg_tilt_change = sum(ctrl['tilt_output'] for ctrl in recent_controls) / len(recent_controls)
            
            # 估算位置变化（简化的模型）
            estimated_x = self.current_position[0] + avg_pan_change * 2
            estimated_y = self.current_position[1] + avg_tilt_change * 2
            
            return (int(estimated_x), int(estimated_y))
        
        return self.current_position
    
    def _select_pid_controller(self, confidence, error_magnitude):
        """
        根据检测置信度和误差大小选择合适的PID控制器
        
        Args:
            confidence: 检测置信度 (0.0-1.0)
            error_magnitude: 误差大小
            
        Returns:
            tuple: (pan_pid, tilt_pid, controller_type)
        """
        if not self.adaptive_control:
            return self.current_pan_pid, self.current_tilt_pid, "base"
        
        # 根据置信度选择控制策略
        if confidence >= self.confidence_threshold_high:
            if error_magnitude > 50:  # 大误差时使用激进参数
                return self.aggressive_pan_pid, self.aggressive_tilt_pid, "aggressive"
            else:  # 小误差时使用基础参数
                return self.base_pan_pid, self.base_tilt_pid, "base"
        elif confidence >= self.confidence_threshold_low:
            return self.base_pan_pid, self.base_tilt_pid, "base"
        else:
            # 低置信度时使用保守参数
            return self.conservative_pan_pid, self.conservative_tilt_pid, "conservative"
    
    def control_laser(self, detection_result, debug=False):
        """
        控制激光位置 - 主要控制函数
        
        Args:
            detection_result: 激光检测结果
            debug: 是否显示调试信息
            
        Returns:
            dict: 控制结果
        """
        self.control_stats['control_attempts'] += 1
        
        # 获取当前激光位置
        current_pos = self.get_current_laser_position(detection_result)
        
        if current_pos is None:
            return self._handle_control_failure(detection_result, debug)
        
        # 计算误差
        target_x, target_y = self.target_position
        current_x, current_y = current_pos
        
        pan_error = current_x - target_x
        tilt_error = current_y - target_y
        error_magnitude = math.sqrt(pan_error**2 + tilt_error**2)
        
        # 更新统计
        self.control_stats['last_error'] = error_magnitude
        self._update_average_error(error_magnitude)
        
        # 选择合适的PID控制器
        confidence = detection_result.get('confidence', 0.5)
        pan_pid, tilt_pid, controller_type = self._select_pid_controller(confidence, error_magnitude)
        
        if controller_type != "base":
            self.control_stats['adaptive_adjustments'] += 1
        
        # 计算PID输出
        dt = self._get_delta_time()
        pan_output = pan_pid.get_pid(pan_error, dt)
        tilt_output = tilt_pid.get_pid(tilt_error, dt)
        
        # 应用输出限制
        pan_output = self._limit_servo_output(pan_output)
        tilt_output = self._limit_servo_output(tilt_output)
        
        # 执行舵机控制
        control_success = self._execute_servo_control(pan_output, tilt_output, debug)
        
        # 记录控制历史
        control_record = {
            'timestamp': time.ticks_ms(),
            'target_pos': self.target_position,
            'current_pos': current_pos,
            'pan_error': pan_error,
            'tilt_error': tilt_error,
            'pan_output': pan_output,
            'tilt_output': tilt_output,
            'confidence': confidence,
            'controller_type': controller_type,
            'error_magnitude': error_magnitude
        }
        
        self._update_control_history(control_record)
        
        if control_success:
            self.control_stats['successful_controls'] += 1
        
        # 绘制控制信息
        if debug:
            self._draw_control_info(current_pos, pan_error, tilt_error, controller_type, confidence)
        
        return {
            'success': control_success,
            'current_position': current_pos,
            'target_position': self.target_position,
            'pan_error': pan_error,
            'tilt_error': tilt_error,
            'pan_output': pan_output,
            'tilt_output': tilt_output,
            'error_magnitude': error_magnitude,
            'confidence': confidence,
            'controller_type': controller_type
        }
    
    def _get_delta_time(self):
        """计算时间间隔"""
        current_time = time.ticks_ms()
        if self.last_update_time == 0:
            dt = 0.05  # 默认50ms
        else:
            dt = (current_time - self.last_update_time) / 1000.0
            dt = max(0.01, min(0.2, dt))  # 限制在10ms-200ms之间
        
        self.last_update_time = current_time
        return dt
    
    def _limit_servo_output(self, output):
        """限制舵机输出"""
        return max(-self.servo_speed_limit, min(self.servo_speed_limit, output))
    
    def _execute_servo_control(self, pan_output, tilt_output, debug=False):
        """执行舵机控制"""
        if not self.servo_available:
            if debug:
                print(f"舵机不可用，模拟控制: Pan={pan_output:.2f}, Tilt={tilt_output:.2f}")
            return True
        
        try:
            # 获取当前角度
            current_pan = self.pan_servo.angle()
            current_tilt = self.tilt_servo.angle()
            
            # 计算新角度
            new_pan = current_pan + pan_output
            new_tilt = current_tilt - tilt_output  # 注意Y轴方向
            
            # 角度限制
            new_pan = max(self.min_servo_angle, min(self.max_servo_angle, new_pan))
            new_tilt = max(self.min_servo_angle, min(self.max_servo_angle, new_tilt))
            
            # 检查是否达到限制
            if new_pan == self.min_servo_angle or new_pan == self.max_servo_angle or \
               new_tilt == self.min_servo_angle or new_tilt == self.max_servo_angle:
                self.control_stats['servo_limits_hit'] += 1
            
            # 设置新角度
            self.pan_servo.angle(new_pan)
            self.tilt_servo.angle(new_tilt)
            
            if debug:
                print(f"舵机控制: Pan {current_pan:.1f}→{new_pan:.1f}, Tilt {current_tilt:.1f}→{new_tilt:.1f}")
            
            return True
            
        except Exception as e:
            print(f"舵机控制失败: {e}")
            return False
    
    def _handle_control_failure(self, detection_result, debug=False):
        """处理控制失效的情况"""
        if debug:
            print("激光控制失效，保持当前位置")
        
        return {
            'success': False,
            'current_position': None,
            'target_position': self.target_position,
            'pan_error': 0,
            'tilt_error': 0,
            'pan_output': 0,
            'tilt_output': 0,
            'error_magnitude': float('inf'),
            'confidence': 0.0,
            'controller_type': 'none'
        }
    
    def _update_control_history(self, control_record):
        """更新控制历史"""
        self.control_history.append(control_record)
        
        if len(self.control_history) > self.max_control_history:
            self.control_history.pop(0)
    
    def _update_average_error(self, error):
        """更新平均误差"""
        alpha = 0.1  # 指数移动平均的权重
        if self.control_stats['average_error'] == 0.0:
            self.control_stats['average_error'] = error
        else:
            self.control_stats['average_error'] = \
                alpha * error + (1 - alpha) * self.control_stats['average_error']
    
    def _draw_control_info(self, current_pos, pan_error, tilt_error, controller_type, confidence):
        """绘制控制信息（这里只是打印，实际绘制在主程序中进行）"""
        print(f"控制信息: 位置{current_pos}, 误差({pan_error:.1f},{tilt_error:.1f}), "
              f"控制器:{controller_type}, 置信度:{confidence:.2f}")
    
    def get_control_stats(self):
        """获取控制统计信息"""
        total = max(1, self.control_stats['control_attempts'])
        success_rate = self.control_stats['successful_controls'] / total * 100
        
        return {
            'success_rate': success_rate,
            'control_attempts': self.control_stats['control_attempts'],
            'successful_controls': self.control_stats['successful_controls'],
            'adaptive_adjustments': self.control_stats['adaptive_adjustments'],
            'servo_limits_hit': self.control_stats['servo_limits_hit'],
            'average_error': self.control_stats['average_error'],
            'last_error': self.control_stats['last_error'],
            'control_history_length': len(self.control_history)
        }
    
    def print_control_stats(self):
        """打印控制统计信息"""
        stats = self.get_control_stats()
        print("=== 激光控制统计 ===")
        print(f"成功率: {stats['success_rate']:.1f}%")
        print(f"控制尝试: {stats['control_attempts']}")
        print(f"成功控制: {stats['successful_controls']}")
        print(f"自适应调整: {stats['adaptive_adjustments']}")
        print(f"舵机限制: {stats['servo_limits_hit']}")
        print(f"平均误差: {stats['average_error']:.1f}")
        print(f"最近误差: {stats['last_error']:.1f}")
        print(f"历史长度: {stats['control_history_length']}")
        print("==================")
    
    def reset_pid_controllers(self):
        """重置PID控制器"""
        self.base_pan_pid.reset_I()
        self.base_tilt_pid.reset_I()
        self.conservative_pan_pid.reset_I()
        self.conservative_tilt_pid.reset_I()
        self.aggressive_pan_pid.reset_I()
        self.aggressive_tilt_pid.reset_I()
        print("PID控制器已重置")
    
    def set_adaptive_control(self, enabled):
        """设置自适应控制开关"""
        self.adaptive_control = enabled
        print(f"自适应控制: {'开启' if enabled else '关闭'}")
    
    def adjust_pid_params(self, controller_type, param_type, delta):
        """
        调整PID参数
        
        Args:
            controller_type: 控制器类型 ("base", "conservative", "aggressive")
            param_type: 参数类型 ("p", "i", "d")
            delta: 调整量
        """
        controllers = {
            "base": (self.base_pan_pid, self.base_tilt_pid),
            "conservative": (self.conservative_pan_pid, self.conservative_tilt_pid),
            "aggressive": (self.aggressive_pan_pid, self.aggressive_tilt_pid)
        }
        
        if controller_type in controllers:
            pan_pid, tilt_pid = controllers[controller_type]
            
            for pid in [pan_pid, tilt_pid]:
                if param_type == "p":
                    pid._kp = max(0.001, pid._kp + delta)
                elif param_type == "i":
                    pid._ki = max(0.0, pid._ki + delta)
                elif param_type == "d":
                    pid._kd = max(0.0, pid._kd + delta)
            
            print(f"{controller_type}控制器{param_type}参数调整: {delta}")
    
    def get_servo_positions(self):
        """获取当前舵机位置"""
        if not self.servo_available:
            return None, None
        
        try:
            pan_angle = self.pan_servo.angle()
            tilt_angle = self.tilt_servo.angle()
            return pan_angle, tilt_angle
        except:
            return None, None
    
    def center_servos(self):
        """将舵机归零到中心位置"""
        if not self.servo_available:
            print("舵机不可用，无法归零")
            return False
        
        try:
            self.pan_servo.angle(0)
            self.tilt_servo.angle(0)
            print("舵机已归零到中心位置")
            return True
        except Exception as e:
            print(f"舵机归零失败: {e}")
            return False