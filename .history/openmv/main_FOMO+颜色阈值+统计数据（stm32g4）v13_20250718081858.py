# main.py - 使用模块化FOMO模型的主程序

import sensor, image, time, math, pyb, camera_setup, display
from my_uart import send_custom_packet
from utils import set_time, get_time_str, get_unix_timestamp
from gamma_controller import GammaController
from fomo_model import FOMOModel  # 导入模块化的FOMO模型

# 常量定义
TARGET_W = 128
TARGET_H = 160
DETECTION_TIMEOUT = 1000  # 检测超时时间（ms）
ALARM_THRESHOLD = 10000   # 声光报警阈值（ms）
FAIL_SEND_LIMIT = 25      # 串口发送失败次数限制
STAT_INTERVAL = 5000      # 统计数据打印间隔（ms）
DAILY_REPORT_HOUR = 13    # 每日报告时间（小时）
MIN_CONFIDENCE = 0.8      # 最小置信度阈值

# 类别ID定义
CLASS_BACKGROUND = 0
CLASS_PIG = 1
CLASS_FECES = 2

# 颜色检测阈值（红色）
FECES_COLOR_THRESHOLD = (30, 82, -19, 69, 34, 66)

class AnimalMonitoringSystem:
    """动物监控系统主类"""
    
    def __init__(self):
        # 初始化显示和控制器
        self.lcd = display.SPIDisplay()
        self.gamma_ctrl = GammaController()
        self.gamma_ctrl.print_controls()
        
        # 初始化FOMO模型
        self.fomo_model = FOMOModel(
            model_path="trained.tflite",
            labels_path="labels.txt", 
            min_confidence=MIN_CONFIDENCE
        )
        
        # 计算缩放参数
        self.scale_x = TARGET_W / 320
        self.scale_y = TARGET_H / 240
        self.scale_ratio = min(self.scale_x, self.scale_y)
        
        # 参考点
        self.x_under = 141
        self.y_under = 215
        
        # 初始化时间和RTC
        self.rtc = pyb.RTC()
        set_time(2025, 6, 30, 13, 12, 0)
        
        # 检测状态变量
        self.detection_start_time = None
        self.last_detection_time = 0
        self.detection_active = False
        self.last_detection_duration = 0
        self.error_led = False
        
        # 统计变量
        self.last_stat_time = pyb.millis()
        self.last_day = self.rtc.datetime()[2]
        self.daily_report_sent = False
        self.fail_send_count = 0
        
        # 初始化每日统计
        self.reset_daily_stats()
        
        # 初始化帧率计时器
        self.clock = time.clock()
        
        # 发送初始参数
        self._send_initial_params()
    
    def reset_daily_stats(self):
        """重置每日统计数据"""
        self.daily_frame_count = 0
        self.daily_target_count = 0
        self.daily_red_detect_count = 0
        self.daily_uart_send_count = 0
        self.daily_fail_count = 0
        self.daily_blob_count = 0
        self.daily_largest_blob_pixels = 0
        self.daily_total_blob_pixels = 0
        self.daily_label_1_detects = 0
        self.daily_label_2_detects = 0
    
    def _send_initial_params(self):
        """发送初始参数到串口"""
        img = sensor.snapshot().lens_corr(1.8)
        send_custom_packet(0, [
            (img.width() >> 8) & 0xff, img.width() & 0xff,
            (img.height() >> 8) & 0xff, img.height() & 0xff,
            (self.x_under >> 8) & 0xff, self.x_under & 0xff,
            (self.y_under >> 8) & 0xff, self.y_under & 0xff
        ])
        self.daily_uart_send_count += 1
    
    def process_feces_detection(self, img, detection):
        """
        处理粪便检测的辅助色块检测
        
        Args:
            img: 输入图像
            detection: FOMO检测结果
            
        Returns:
            tuple: (target_blob, max_pixels)
        """
        x, y, w, h = detection['bbox']
        
        # 计算辅助色块检测区域
        color_x = max(0, math.ceil(x - w))
        color_y = max(0, math.ceil(y - h))
        color_w = min(img.width() - color_x, 3 * w)
        color_h = min(img.height() - color_y, 3 * h)
        color_roi = (color_x, color_y, color_w, color_h)
        
        # 绘制检测区域
        img.draw_rectangle(color_roi, color=(0, 0, 255))  # 蓝色
        
        # 在检测区域内查找红色色块
        blobs = img.find_blobs([FECES_COLOR_THRESHOLD], roi=color_roi, merge=True)
        
        target_blob = None
        max_pixels = 0
        
        if blobs:
            for blob in blobs:
                self.daily_blob_count += 1
                self.daily_total_blob_pixels += blob.pixels()
                
                # 找到最大的色块
                if blob.pixels() > max_pixels:
                    max_pixels = blob.pixels()
                    target_blob = blob
                    
                    # 绘制目标色块
                    img.draw_rectangle(target_blob.rect(), color=(255, 0, 0))
                    
                    # 更新检测时间
                    self._update_detection_time()
        
        return target_blob, max_pixels
    
    def _update_detection_time(self):
        """更新检测时间状态"""
        now = pyb.millis()
        if not self.detection_active:
            self.detection_start_time = now
            self.detection_active = True
            print("[开始检测] 时间戳(ms):", self.detection_start_time)
        self.last_detection_time = now
    
    def _check_detection_timeout(self):
        """检查检测超时"""
        if self.detection_active:
            now = pyb.millis()
            if now - self.last_detection_time > DETECTION_TIMEOUT:
                self.detection_active = False
                self.last_detection_duration = self.last_detection_time - self.detection_start_time
                print("[检测结束] 持续时间(ms):", self.last_detection_duration)
                return True
        return False
    
    def _update_alarm_status(self):
        """更新声光报警状态"""
        if self.detection_active:
            current_duration = pyb.millis() - self.detection_start_time
            if current_duration >= ALARM_THRESHOLD:
                self.error_led = True
                print("检测持续时间超过10秒，触发声光报警！")
                return current_duration
        elif not self.detection_active:
            self.error_led = False
        return 0
    
    def _send_detection_data(self, target_blob):
        """发送检测数据到串口"""
        if target_blob and not self.error_led:
            center_x, center_y = self.fomo_model.get_detection_center({
                'bbox': (target_blob.cx() - target_blob.w()//2, 
                        target_blob.cy() - target_blob.h()//2,
                        target_blob.w(), target_blob.h())
            })
            
            # 实际上直接使用blob的中心点
            center_x, center_y = target_blob.cx(), target_blob.cy()
            
            send_custom_packet(1, [
                1, 
                (center_x >> 8) & 0xFF, center_x & 0xFF,
                (center_y >> 8) & 0xFF, center_y & 0xFF
            ])
            self.daily_uart_send_count += 1
            print(f"发送检测中心点: cx={center_x}, cy={center_y}")
    
    def _handle_no_detection(self):
        """处理未检测到目标的情况"""
        if self.fail_send_count < FAIL_SEND_LIMIT:
            self.fail_send_count += 1
            send_custom_packet(1, [0])
            self.daily_uart_send_count += 1
            self.daily_fail_count += 1
            print(f"未检测到目标，发送0 (第{self.fail_send_count}次)")
        else:
            print(f"未检测到目标，已达到发送上限 ({self.fail_send_count})")
    
    def _send_realtime_stats(self, pig_count, feces_count):
        """发送实时统计数据"""
        if pyb.millis() - self.last_stat_time >= STAT_INTERVAL:
            print("===== 实时监控数据 =======================================")
            print("日期:", self.rtc.datetime())
            print("串口发送次数:", self.daily_uart_send_count)
            print("粪便数目:", feces_count)
            print("猪的数目:", pig_count)
            print(f"声光报警: {self.error_led}")
            print("=========================================================")
            
            # 发送到串口
            send_custom_packet(2, [
                (feces_count >> 8) & 0xff, feces_count & 0xff,
                (pig_count >> 8) & 0xff, pig_count & 0xff,
                1 if self.error_led else 0
            ])
            self.daily_uart_send_count += 1
            self.last_stat_time = pyb.millis()
    
    def _check_daily_report(self):
        """检查并发送每日报告"""
        current_time = self.rtc.datetime()
        hour = current_time[4]
        current_day = current_time[2]
        
        # 每日13点报告
        if hour == DAILY_REPORT_HOUR and not self.daily_report_sent:
            print("=== [每日13:00统计报告] =====================================")
            print("帧数:", self.daily_frame_count)
            print("FOMO目标次数:", self.daily_target_count)
            print("红色检测成功数:", self.daily_red_detect_count)
            print("红色检测失败数:", self.daily_fail_count)
            print("串口发送次数:", self.daily_uart_send_count)
            print("色块总数:", self.daily_blob_count)
            print("最大红色块像素数:", self.daily_largest_blob_pixels)
            print("累计红色像素数:", self.daily_total_blob_pixels)
            print("=========================================================")
            self.daily_report_sent = True
        
        # 0点重置报告状态并发送每日统计
        if hour == 0:
            self.daily_report_sent = False
            send_custom_packet(3, [
                (self.daily_frame_count >> 8) & 0xff, self.daily_frame_count & 0xff,
                (self.daily_blob_count >> 8) & 0xff, self.daily_blob_count & 0xff,
                (self.daily_total_blob_pixels >> 8) & 0xff, self.daily_total_blob_pixels & 0xff,
                (self.daily_largest_blob_pixels >> 8) & 0xff, self.daily_largest_blob_pixels & 0xff
            ])
            self.daily_uart_send_count += 1
        
        # 新的一天，重置统计数据
        if current_day != self.last_day:
            self.reset_daily_stats()
            self.last_day = current_day
            print("== 新的一天，统计数据已清零 ==")
    
    def _draw_image_info(self, img, pig_count, feces_count):
        """在图像上绘制信息"""
        # 绘制参考点
        keypoints = [(self.x_under, self.y_under, 270)]
        img.draw_keypoints(keypoints, size=10, color=(0, 255, 0))
        
        # 绘制统计信息
        img.draw_string(0, 0, f"Shit: {feces_count}", color=(0, 255, 0), thickness=2)
        img.draw_string(0, 20, f"Pig: {pig_count}", color=(0, 255, 0), thickness=2)
        
        # 显示gamma控制状态
        status_text = self.gamma_ctrl.get_status_text()
        img.draw_string(10, 60, status_text, color=(255, , 255), scale=1)
    
    def run(self):
        """主运行循环"""
        print("动物监控系统启动...")
        print("模型信息:", self.fomo_model.get_model_info())
        
        while True:
            # 检查按键状态
            self.gamma_ctrl.check_buttons()
            
            # 开始帧计时
            self.clock.tick()
            
            # 捕获和预处理图像
            img = sensor.snapshot().lens_corr(1.8)
            img = self.gamma_ctrl.apply_gamma_correction(img)
            img = img.histeq(adaptive=True, clip_limit=3)
            
            # 更新帧统计
            self.daily_frame_count += 1
            
            # 使用FOMO模型进行检测
            detections = self.fomo_model.predict(img)
            
            # 绘制检测结果
            detection_stats = self.fomo_model.draw_detections(img, detections)
            
            # 处理检测结果
            pig_count = 0
            feces_count = 0
            found_feces = False
            target_blob = None
            max_pixels = 0
            
            # 处理猪的检测
            if CLASS_PIG in detection_stats:
                pig_count = detection_stats[CLASS_PIG]['count']
            
            # 处理粪便检测
            if CLASS_FECES in detection_stats:
                found_feces = True
                feces_count = detection_stats[CLASS_FECES]['count']
                self.fail_send_count = 0  # 重置失败计数
                
                # 更新统计
                self.daily_target_count += 1
                self.daily_label_1_detects += feces_count
                self.daily_red_detect_count += 1
                
                # 处理每个粪便检测
                for detection in detection_stats[CLASS_FECES]['detections']:
                    blob, pixels = self.process_feces_detection(img, detection)
                    if pixels > max_pixels:
                        max_pixels = pixels
                        target_blob = blob
                
                # 更新最大像素统计
                if max_pixels > self.daily_largest_blob_pixels:
                    self.daily_largest_blob_pixels = max_pixels
                
                # 绘制目标中心点
                if target_blob:
                    center_x, center_y = target_blob.cx(), target_blob.cy()
                    img.draw_circle((center_x, center_y, 12), color=(0, 255, 0))
                    img.draw_cross((center_x, center_y), color=(0, 255, 0))
                    print(f"粪便检测中心点: cx={center_x}, cy={center_y}")
            
            # 发送检测数据或处理未检测情况
            if found_feces and target_blob:
                self._send_detection_data(target_blob)
                print("日期:", get_time_str())
            else:
                self._handle_no_detection()

            # 检查检测超时
            self._check_detection_timeout()

            # 更新声光报警状态
            current_duration = self._update_alarm_status()
            
            # 发送实时统计数据
            self._send_realtime_stats(pig_count, feces_count)

            # 检查每日报告
            self._check_daily_report()

            # 在图像上绘制信息
            self._draw_image_info(img, pig_count, feces_count)

            # 打印帧率和调试信息
            print(f"FPS: {self.clock.fps():.2f}")
            print(f"检测状态: 粪便={found_feces}, 猪={pig_count}, 报警={self.error_led}")

            # 修复：只有在检测激活时才显示持续时间
            if self.detection_active:
                print(f"当前检测持续时间: {current_duration}ms")
            elif self.last_detection_duration > 0:
                print(f"上次检测持续时间: {self.last_detection_duration}ms")

            print()

# 主程序入口
if __name__ == "__main__":
    try:
        system = AnimalMonitoringSystem()
        system.run()
    except KeyboardInterrupt:
        print("程序被用户中断")
    except Exception as e:
        print(f"程序异常: {e}")
        import traceback
        traceback.print_exc()