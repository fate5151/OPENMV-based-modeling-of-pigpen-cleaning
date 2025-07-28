# uv_laser_drawing.py - UV激光绘图系统主程序
# 集成激光检测和控制模块，实现高精度UV激光绘图

import sensor, image, time, pyb, math
from laser_detection import LaserDetection
from laser_control import LaserControl

class UVLaserDrawingSystem:
    """UV激光绘图系统主类"""
    
    def __init__(self):
        print("正在初始化UV激光绘图系统...")
        
        # 初始化摄像头
        self._init_camera()
        
        # 初始化激光检测和控制模块
        self.laser_detector = LaserDetection()
        self.laser_controller = LaserControl()
        
        # 系统状态
        self.system_active = False
        self.debug_mode = True
        self.auto_target_mode = False  # 自动目标模式
        
        # 绘图状态
        self.drawing_active = False
        self.drawing_path = []  # 绘图路径
        self.current_path_index = 0
        
        # 性能监控
        self.frame_count = 0
        self.fps_timer = time.clock()
        self.last_stats_time = time.ticks_ms()
        self.stats_interval = 5000  # 5秒打印一次统计
        
        # 按键控制（使用现有的引脚配置）
        self._init_controls()
        
        # 显示参数
        self.image_width = 320
        self.image_height = 240
        
        print("UV激光绘图系统初始化完成！")
        self._print_usage_instructions()
    
    def _init_camera(self):
        """初始化摄像头"""
        try:
            sensor.reset()
            sensor.set_pixformat(sensor.RGB565)
            sensor.set_framesize(sensor.QVGA)  # 320x240
            sensor.skip_frames(time=2000)
            sensor.set_auto_whitebal(False)  # 关闭自动白平衡以获得一致的颜色
            print("摄像头初始化成功")
        except Exception as e:
            print(f"摄像头初始化失败: {e}")
            raise
    
    def _init_controls(self):
        """初始化按键控制"""
        try:
            # 使用现有的按键配置
            self.button_pins = [
                pyb.Pin('P7', pyb.Pin.IN, pyb.Pin.PULL_UP),   # 功能键1
                pyb.Pin('P8', pyb.Pin.IN, pyb.Pin.PULL_UP),   # 功能键2
                pyb.Pin('P9', pyb.Pin.IN, pyb.Pin.PULL_UP),   # 功能键3
                pyb.Pin('P4', pyb.Pin.IN, pyb.Pin.PULL_UP),   # 功能键4
                pyb.Pin('P3', pyb.Pin.IN, pyb.Pin.PULL_UP),   # 功能键5
                pyb.Pin('P5', pyb.Pin.IN, pyb.Pin.PULL_UP),   # 功能键6
            ]
            
            self.last_button_states = [True] * 6
            self.debounce_time = 200  # 防抖时间
            self.last_press_times = [0] * 6
            
            print("按键控制初始化成功")
        except Exception as e:
            print(f"按键控制初始化失败: {e}")
            self.button_pins = []
    
    def _print_usage_instructions(self):
        """打印使用说明"""
        print("\n=== UV激光绘图系统控制说明 ===")
        print("P7: 系统开/关")
        print("P8: 调试模式开/关")
        print("P9: 重置系统")
        print("P4: 设置目标位置到图像中心")
        print("P3: 舵机归零")
        print("P5: 自动目标模式开/关")
        print("P7+P8: 开始/停止绘图")
        print("P9+P4: 打印系统统计")
        print("========================\n")
    
    def _check_buttons(self):
        """检查按键状态"""
        if not self.button_pins:
            return
        
        current_time = time.ticks_ms()
        current_states = [pin.value() for pin in self.button_pins]
        
        # 检查组合按键
        if not current_states[0] and not current_states[1]:  # P7+P8: 开始/停止绘图
            if current_time - self.last_press_times[0] > self.debounce_time:
                self._toggle_drawing()
                self.last_press_times[0] = current_time
                return
        
        if not current_states[2] and not current_states[3]:  # P9+P4: 打印统计
            if current_time - self.last_press_times[2] > self.debounce_time:
                self._print_system_stats()
                self.last_press_times[2] = current_time
                return
        
        # 检查单个按键
        for i in range(6):
            if self.last_button_states[i] and not current_states[i]:  # 按键按下
                if current_time - self.last_press_times[i] > self.debounce_time:
                    self._handle_button_press(i)
                    self.last_press_times[i] = current_time
        
        self.last_button_states = current_states
    
    def _handle_button_press(self, button_index):
        """处理单个按键按下"""
        if button_index == 0:      # P7: 系统开/关
            self._toggle_system()
        elif button_index == 1:    # P8: 调试模式开/关
            self._toggle_debug_mode()
        elif button_index == 2:    # P9: 重置系统
            self._reset_system()
        elif button_index == 3:    # P4: 设置目标位置
            self._set_target_to_center()
        elif button_index == 4:    # P3: 舵机归零
            self._center_servos()
        elif button_index == 5:    # P5: 自动目标模式
            self._toggle_auto_target_mode()
    
    def _toggle_system(self):
        """切换系统开关状态"""
        self.system_active = not self.system_active
        status = "激活" if self.system_active else "停用"
        print(f"系统状态: {status}")
        
        if not self.system_active:
            self.drawing_active = False
    
    def _toggle_debug_mode(self):
        """切换调试模式"""
        self.debug_mode = not self.debug_mode
        status = "开启" if self.debug_mode else "关闭"
        print(f"调试模式: {status}")
    
    def _toggle_drawing(self):
        """切换绘图状态"""
        if not self.system_active:
            print("系统未激活，无法开始绘图")
            return
        
        self.drawing_active = not self.drawing_active
        status = "开始" if self.drawing_active else "停止"
        print(f"绘图状态: {status}")
        
        if self.drawing_active:
            self._init_drawing_path()
    
    def _toggle_auto_target_mode(self):
        """切换自动目标模式"""
        self.auto_target_mode = not self.auto_target_mode
        status = "开启" if self.auto_target_mode else "关闭"
        print(f"自动目标模式: {status}")
    
    def _reset_system(self):
        """重置系统"""
        print("正在重置系统...")
        self.laser_detector.reset_history()
        self.laser_controller.reset_pid_controllers()
        self.drawing_active = False
        self.drawing_path = []
        self.current_path_index = 0
        print("系统重置完成")
    
    def _set_target_to_center(self):
        """设置目标位置到图像中心"""
        center_x = self.image_width // 2
        center_y = self.image_height // 2
        self.laser_controller.set_target_position(center_x, center_y)
        print(f"目标位置设置为图像中心: ({center_x}, {center_y})")
    
    def _center_servos(self):
        """舵机归零"""
        if self.laser_controller.center_servos():
            print("舵机归零完成")
        else:
            print("舵机归零失败")
    
    def _init_drawing_path(self):
        """初始化绘图路径（示例：正方形）"""
        center_x = self.image_width // 2
        center_y = self.image_height // 2
        size = 60
        
        # 创建正方形路径
        self.drawing_path = [
            (center_x - size, center_y - size),  # 左上
            (center_x + size, center_y - size),  # 右上
            (center_x + size, center_y + size),  # 右下
            (center_x - size, center_y + size),  # 左下
            (center_x - size, center_y - size),  # 回到起点
        ]
        self.current_path_index = 0
        print(f"绘图路径初始化完成，共{len(self.drawing_path)}个点")
    
    def _update_drawing_target(self):
        """更新绘图目标位置"""
        if not self.drawing_active or not self.drawing_path:
            return
        
        current_target = self.drawing_path[self.current_path_index]
        current_pos = self.laser_controller.current_position
        
        # 计算到目标的距离
        distance = math.sqrt((current_pos[0] - current_target[0])**2 + 
                           (current_pos[1] - current_target[1])**2)
        
        # 如果接近目标，移动到下一个点
        if distance < 15:  # 15像素的容差
            self.current_path_index = (self.current_path_index + 1) % len(self.drawing_path)
            next_target = self.drawing_path[self.current_path_index]
            self.laser_controller.set_target_position(next_target[0], next_target[1])
            
            if self.debug_mode:
                print(f"移动到路径点 {self.current_path_index}: {next_target}")
    
    def _draw_system_info(self, img):
        """在图像上绘制系统信息"""
        # 绘制系统状态
        status_color = (0, 255, 0) if self.system_active else (255, 0, 0)
        img.draw_string(5, 5, f"系统:{'激活' if self.system_active else '停用'}", 
                       color=status_color, scale=1)
        
        # 绘制调试模式状态
        debug_color = (255, 255, 0) if self.debug_mode else (128, 128, 128)
        img.draw_string(5, 20, f"调试:{'开' if self.debug_mode else '关'}", 
                       color=debug_color, scale=1)
        
        # 绘制绘图状态
        if self.drawing_active:
            img.draw_string(5, 35, f"绘图:{self.current_path_index}/{len(self.drawing_path)}", 
                           color=(0, 255, 255), scale=1)
        
        # 绘制目标位置
        target_x, target_y = self.laser_controller.target_position
        img.draw_circle((target_x, target_y, 10), color=(255, 0, 255), thickness=2)
        img.draw_cross((target_x, target_y), color=(255, 0, 255), size=5, thickness=2)
        
        # 绘制绘图路径
        if self.debug_mode and self.drawing_path:
            for i in range(len(self.drawing_path)):
                x, y = self.drawing_path[i]
                color = (0, 255, 255) if i == self.current_path_index else (100, 100, 100)
                img.draw_circle((x, y, 5), color=color)
                
                # 绘制路径连线
                if i > 0:
                    prev_x, prev_y = self.drawing_path[i-1]
                    img.draw_line((prev_x, prev_y, x, y), color=(100, 100, 100))
        
        # 绘制FPS
        fps = self.fps_timer.fps()
        img.draw_string(self.image_width - 80, 5, f"FPS:{fps:.1f}", 
                       color=(255, 255, 255), scale=1)
    
    def _print_system_stats(self):
        """打印系统统计信息"""
        print("\n=== UV激光绘图系统统计 ===")
        
        # 激光检测统计
        print("激光检测:")
        self.laser_detector.print_detection_stats()
        
        print()
        # 激光控制统计
        print("激光控制:")
        self.laser_controller.print_control_stats()
        
        # 舵机位置
        pan_angle, tilt_angle = self.laser_controller.get_servo_positions()
        if pan_angle is not None:
            print(f"舵机位置: Pan={pan_angle:.1f}°, Tilt={tilt_angle:.1f}°")
        
        print("========================\n")
    
    def run(self):
        """主运行循环"""
        print("UV激光绘图系统开始运行...")
        
        # 设置初始目标位置
        self._set_target_to_center()
        
        while True:
            try:
                # 开始帧计时
                self.fps_timer.tick()
                self.frame_count += 1
                
                # 检查按键
                self._check_buttons()
                
                # 捕获图像
                img = sensor.snapshot()
                
                # 系统激活时进行激光检测和控制
                if self.system_active:
                    # 激光检测
                    detection_result = self.laser_detector.get_current_laser_position(
                        img, debug=self.debug_mode)
                    
                    # 更新绘图目标（如果在绘图模式）
                    if self.drawing_active:
                        self._update_drawing_target()
                    
                    # 激光控制
                    control_result = self.laser_controller.control_laser(
                        detection_result, debug=self.debug_mode)
                    
                    # 调试信息
                    if self.debug_mode:
                        if detection_result['position']:
                            pos = detection_result['position']
                            conf = detection_result['confidence']
                            method = detection_result['method']
                            print(f"检测: 位置{pos}, 置信度{conf:.2f}, 方法{method}")
                        
                        if control_result['success']:
                            error_mag = control_result['error_magnitude']
                            controller = control_result['controller_type']
                            print(f"控制: 误差{error_mag:.1f}, 控制器{controller}")
                
                # 绘制系统信息
                self._draw_system_info(img)
                
                # 定期打印统计信息
                if time.ticks_diff(time.ticks_ms(), self.last_stats_time) > self.stats_interval:
                    if self.system_active:
                        self._print_system_stats()
                    self.last_stats_time = time.ticks_ms()
                
                # 短暂延时
                time.sleep_ms(10)
                
            except KeyboardInterrupt:
                print("用户中断程序")
                break
            except Exception as e:
                print(f"运行时错误: {e}")
                # 继续运行，不退出
                time.sleep_ms(100)
        
        print("UV激光绘图系统停止运行")

# 主程序入口
if __name__ == "__main__":
    try:
        system = UVLaserDrawingSystem()
        system.run()
    except Exception as e:
        print(f"系统启动失败: {e}")
        import traceback
        traceback.print_exc()