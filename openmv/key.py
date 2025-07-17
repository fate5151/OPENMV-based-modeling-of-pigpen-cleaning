import sensor, image, time, pyb
import math

# 初始化摄像头
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)

# 定义按键引脚
button_pins = [
    pyb.Pin('P7', pyb.Pin.IN, pyb.Pin.PULL_UP),   # Gamma +
    pyb.Pin('P8', pyb.Pin.IN, pyb.Pin.PULL_UP),   # Gamma -
    pyb.Pin('P9', pyb.Pin.IN, pyb.Pin.PULL_UP),   # Contrast +
    pyb.Pin('P0', pyb.Pin.IN, pyb.Pin.PULL_UP),   # Contrast -
    pyb.Pin('P3', pyb.Pin.IN, pyb.Pin.PULL_UP),   # Brightness +
    pyb.Pin('P2', pyb.Pin.IN, pyb.Pin.PULL_UP),   # Brightness -
]

# 状态LED（可选）
led_red = pyb.Pin('LED_RED', pyb.Pin.OUT_PP)
led_green = pyb.Pin('LED_GREEN', pyb.Pin.OUT_PP)
led_blue = pyb.Pin('LED_BLUE', pyb.Pin.OUT_PP)

class GammaController:
    def __init__(self):
        # 初始参数值
        self.gamma = 1.0        # 范围: 0.1 - 3.0
        self.contrast = 1.0     # 范围: 0.1 - 3.0
        self.brightness = 0.0   # 范围: -1.0 - 1.0

        # 调整步长
        self.gamma_step = 0.1
        self.contrast_step = 0.1
        self.brightness_step = 0.1

        # 参数范围限制
        self.gamma_range = (0.1, 3.0)
        self.contrast_range = (0.1, 3.0)
        self.brightness_range = (-1.0, 1.0)

        # 按键状态
        self.last_button_states = [True] * 6
        self.last_press_time = [0] * 6
        self.debounce_time = 100  # 防抖时间

        # 显示模式
        self.display_mode = 0  # 0: 参数显示, 1: 预设模式
        self.preset_index = 0

        # 预设参数组合
        self.presets = [
            {"name": "默认", "gamma": 1.0, "contrast": 1.0, "brightness": 0.0},
            {"name": "提亮", "gamma": 0.7, "contrast": 1.2, "brightness": 0.2},
            {"name": "增强", "gamma": 0.8, "contrast": 1.5, "brightness": 0.1},
            {"name": "柔和", "gamma": 1.2, "contrast": 0.8, "brightness": 0.1},
            {"name": "高对比", "gamma": 0.9, "contrast": 2.0, "brightness": 0.0},
            {"name": "低光", "gamma": 0.6, "contrast": 1.3, "brightness": 0.3}
        ]

        print("Gamma校正控制系统启动...")
        self.print_controls()
        self.print_current_params()

    def print_controls(self):
        """打印控制说明"""
        print("\n=== 按键控制说明 ===")
        print("P7: Gamma +     P8: Gamma -")
        print("P9: Contrast +  P0: Contrast -")
        print("P1: Brightness+ P2: Brightness-")
        print("同时按P7+P8: 重置参数")
        print("同时按P9+P0: 切换预设")
        print("同时按P1+P2: 保存当前参数")
        print("==================\n")

    def print_current_params(self):
        """打印当前参数"""
        print(f"当前参数: Gamma={self.gamma:.1f}, Contrast={self.contrast:.1f}, Brightness={self.brightness:.1f}")

    def clamp_value(self, value, min_val, max_val):
        """限制数值范围"""
        return max(min_val, min(max_val, value))

    def update_gamma(self, delta):
        """更新Gamma值"""
        self.gamma += delta
        self.gamma = self.clamp_value(self.gamma, *self.gamma_range)
        print(f"Gamma: {self.gamma:.1f}")
        self.blink_led(led_red)

    def update_contrast(self, delta):
        """更新对比度值"""
        self.contrast += delta
        self.contrast = self.clamp_value(self.contrast, *self.contrast_range)
        print(f"Contrast: {self.contrast:.1f}")
        self.blink_led(led_green)

    def update_brightness(self, delta):
        """更新亮度值"""
        self.brightness += delta
        self.brightness = self.clamp_value(self.brightness, *self.brightness_range)
        print(f"Brightness: {self.brightness:.1f}")
        self.blink_led(led_blue)

    def blink_led(self, led):
        """LED闪烁提示"""
        led.on()
        pyb.delay(50)
        led.off()

    def reset_params(self):
        """重置参数到默认值"""
        self.gamma = 1.0
        self.contrast = 1.0
        self.brightness = 0.0
        print("参数已重置到默认值")
        self.print_current_params()

        # 全部LED闪烁
        for led in [led_red, led_green, led_blue]:
            led.on()
        pyb.delay(200)
        for led in [led_red, led_green, led_blue]:
            led.off()

    def load_preset(self):
        """加载预设参数"""
        preset = self.presets[self.preset_index]
        self.gamma = preset["gamma"]
        self.contrast = preset["contrast"]
        self.brightness = preset["brightness"]
        print(f"加载预设: {preset['name']}")
        self.print_current_params()

        # 循环到下一个预设
        self.preset_index = (self.preset_index + 1) % len(self.presets)

    def save_current_params(self):
        """保存当前参数（示例功能）"""
        params_str = f"gamma={self.gamma:.1f}, contrast={self.contrast:.1f}, brightness={self.brightness:.1f}"
        print(f"保存参数: {params_str}")

        # 可以在这里添加保存到文件的代码
        try:
            with open("gamma_params.txt", "w") as f:
                f.write(f"{self.gamma},{self.contrast},{self.brightness}")
            print("参数已保存到文件")
        except:
            print("保存文件失败")

    def load_saved_params(self):
        """加载保存的参数"""
        try:
            with open("gamma_params.txt", "r") as f:
                line = f.read().strip()
                params = line.split(',')
                if len(params) == 3:
                    self.gamma = float(params[0])
                    self.contrast = float(params[1])
                    self.brightness = float(params[2])
                    print("已加载保存的参数")
                    self.print_current_params()
        except:
            print("无法加载保存的参数，使用默认值")

    def check_buttons(self):
        """检查按键状态"""
        current_time = pyb.millis()
        current_states = [pin.value() for pin in button_pins]

        # 检查组合按键
        if not current_states[0] and not current_states[1]:  # P7+P8: 重置
            if current_time - self.last_press_time[0] > self.debounce_time:
                self.reset_params()
                self.last_press_time[0] = current_time
                return

        if not current_states[2] and not current_states[3]:  # P9+P0: 切换预设
            if current_time - self.last_press_time[2] > self.debounce_time:
                self.load_preset()
                self.last_press_time[2] = current_time
                return

        if not current_states[4] and not current_states[5]:  # P1+P2: 保存参数
            if current_time - self.last_press_time[4] > self.debounce_time:
                self.save_current_params()
                self.last_press_time[4] = current_time
                return

        # 检查单个按键
        for i in range(6):
            if self.last_button_states[i] and not current_states[i]:  # 按键按下
                if current_time - self.last_press_time[i] > self.debounce_time:
                    self.handle_button_press(i)
                    self.last_press_time[i] = current_time

        self.last_button_states = current_states

    def handle_button_press(self, button_index):
        """处理单个按键按下"""
        if button_index == 0:      # P7: Gamma +
            self.update_gamma(self.gamma_step)
        elif button_index == 1:    # P8: Gamma -
            self.update_gamma(-self.gamma_step)
        elif button_index == 2:    # P9: Contrast +
            self.update_contrast(self.contrast_step)
        elif button_index == 3:    # P0: Contrast -
            self.update_contrast(-self.contrast_step)
        elif button_index == 4:    # P1: Brightness +
            self.update_brightness(self.brightness_step)
        elif button_index == 5:    # P2: Brightness -
            self.update_brightness(-self.brightness_step)

    def apply_gamma_correction(self, img):
        """应用Gamma校正"""
        try:
            # 应用gamma校正
            corrected_img = img.gamma_corr(
                gamma=self.gamma,
                contrast=self.contrast,
                brightness=self.brightness
            )
            return corrected_img
        except Exception as e:
            print(f"Gamma校正错误: {e}")
            return img

    def get_status_text(self):
        """获取状态文本"""
        return f"G:{self.gamma:.1f} C:{self.contrast:.1f} B:{self.brightness:.1f}"

# 创建控制器实例
controller = GammaController()

# 尝试加载保存的参数
controller.load_saved_params()

# 主循环
clock = time.clock()
frame_count = 0

print("开始图像处理循环...")
print("实时调整Gamma校正参数")

while True:
    clock.tick()

    # 检查按键输入
    controller.check_buttons()

    # 捕获图像
    img = sensor.snapshot()

    # 应用Gamma校正
    processed_img = controller.apply_gamma_correction(img)

    # 在图像上显示参数信息
    status_text = controller.get_status_text()
    processed_img.draw_string(10, 10, status_text, color=(255, 255, 255), scale=1)

    # 显示预设信息
    if controller.display_mode == 1:
        preset_name = controller.presets[controller.preset_index]["name"]
        processed_img.draw_string(10, 25, f"预设: {preset_name}", color=(255, 255, 0), scale=1)

    # 每秒打印一次FPS和参数
    frame_count += 1
    if frame_count % 30 == 0:
        fps = clock.fps()
        print(f"FPS: {fps:.1f}, {status_text}")

    # 短暂延时
    time.sleep_ms(10)

    count=0
    for pin_value in button_pins:
        print(f"Button state{count}: {pin_value.value()}")
        count += 1
    print("=============================")
