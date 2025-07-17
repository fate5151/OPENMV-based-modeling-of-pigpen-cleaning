import sensor, image, time, pyb

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

# 状态LED
led_red = pyb.Pin('LED_RED', pyb.Pin.OUT_PP)
led_green = pyb.Pin('LED_GREEN', pyb.Pin.OUT_PP)
led_blue = pyb.Pin('LED_BLUE', pyb.Pin.OUT_PP)

class GammaController:
    def __init__(self):
        # 核心参数
        self.gamma = 1.0        # 范围: 0.1 - 3.0
        self.contrast = 1.0     # 范围: 0.1 - 3.0
        self.brightness = 0.0   # 范围: -1.0 - 1.0

        # 调整步长
        self.gamma_step = 0.1
        self.contrast_step = 0.1
        self.brightness_step = 0.1

        # 参数范围
        self.gamma_range = (0.1, 3.0)
        self.contrast_range = (0.1, 3.0)
        self.brightness_range = (-1.0, 1.0)

        # 按键防抖
        self.last_button_states = [True] * 6
        self.last_press_time = [0] * 6
        self.debounce_time = 100

        # 预设参数
        self.presets = [
            {"name": "默认", "gamma": 1.0, "contrast": 1.0, "brightness": 0.0},
            {"name": "提亮", "gamma": 0.7, "contrast": 1.2, "brightness": 0.2},
            {"name": "增强", "gamma": 0.8, "contrast": 1.5, "brightness": 0.1},
            {"name": "柔和", "gamma": 1.2, "contrast": 0.8, "brightness": 0.1},
        ]
        self.preset_index = 0

        print("Gamma校正控制系统启动")
        self.print_controls()
        self.print_current_params()

    def print_controls(self):
        print("\n=== 按键控制 ===")
        print("P7: Gamma +     P8: Gamma -")
        print("P9: Contrast +  P0: Contrast -")
        print("P3: Brightness+ P2: Brightness-")
        print("P7+P8: 重置     P9+P0: 预设")
        print("================\n")

    def print_current_params(self):
        print(f"参数: G={self.gamma:.1f}, C={self.contrast:.1f}, B={self.brightness:.1f}")

    def clamp_value(self, value, min_val, max_val):
        return max(min_val, min(max_val, value))

    def update_gamma(self, delta):
        self.gamma += delta
        self.gamma = self.clamp_value(self.gamma, *self.gamma_range)
        print(f"Gamma: {self.gamma:.1f}")
        self.blink_led(led_red)

    def update_contrast(self, delta):
        self.contrast += delta
        self.contrast = self.clamp_value(self.contrast, *self.contrast_range)
        print(f"Contrast: {self.contrast:.1f}")
        self.blink_led(led_green)

    def update_brightness(self, delta):
        self.brightness += delta
        self.brightness = self.clamp_value(self.brightness, *self.brightness_range)
        print(f"Brightness: {self.brightness:.1f}")
        self.blink_led(led_blue)

    def blink_led(self, led):
        led.on()
        pyb.delay(50)
        led.off()

    def reset_params(self):
        self.gamma = 1.0
        self.contrast = 1.0
        self.brightness = 0.0
        print("参数已重置")
        self.print_current_params()

        # 全部LED闪烁
        for led in [led_red, led_green, led_blue]:
            led.on()
        pyb.delay(200)
        for led in [led_red, led_green, led_blue]:
            led.off()

    def load_preset(self):
        preset = self.presets[self.preset_index]
        self.gamma = preset["gamma"]
        self.contrast = preset["contrast"]
        self.brightness = preset["brightness"]
        print(f"预设: {preset['name']}")
        self.print_current_params()

        # 循环到下一个预设
        self.preset_index = (self.preset_index + 1) % len(self.presets)

    def check_buttons(self):
        current_time = pyb.millis()
        current_states = [pin.value() for pin in button_pins]

        # 组合按键检查
        if not current_states[0] and not current_states[1]:  # P7+P8: 重置
            if current_time - self.last_press_time[0] > self.debounce_time:
                self.reset_params()
                self.last_press_time[0] = current_time
                return

        if not current_states[2] and not current_states[3]:  # P9+P0: 预设
            if current_time - self.last_press_time[2] > self.debounce_time:
                self.load_preset()
                self.last_press_time[2] = current_time
                return

        # 单个按键检查
        for i in range(6):
            if self.last_button_states[i] and not current_states[i]:
                if current_time - self.last_press_time[i] > self.debounce_time:
                    self.handle_button_press(i)
                    self.last_press_time[i] = current_time

        self.last_button_states = current_states

    def handle_button_press(self, button_index):
        if button_index == 0:      # P7: Gamma +
            self.update_gamma(self.gamma_step)
        elif button_index == 1:    # P8: Gamma -
            self.update_gamma(-self.gamma_step)
        elif button_index == 2:    # P9: Contrast +
            self.update_contrast(self.contrast_step)
        elif button_index == 3:    # P0: Contrast -
            self.update_contrast(-self.contrast_step)
        elif button_index == 4:    # P3: Brightness +
            self.update_brightness(self.brightness_step)
        elif button_index == 5:    # P2: Brightness -
            self.update_brightness(-self.brightness_step)

    def apply_gamma_correction(self, img):
        try:
            return img.gamma_corr(
                gamma=self.gamma,
                contrast=self.contrast,
                brightness=self.brightness
            )
        except Exception as e:
            print(f"Gamma校正错误: {e}")
            return img

    def get_status_text(self):
        return f"G:{self.gamma:.1f} C:{self.contrast:.1f} B:{self.brightness:.1f}"

# 创建控制器
controller = GammaController()

# 主循环
clock = time.clock()
frame_count = 0

print("开始图像处理...")

while True:
    clock.tick()

    # 检查按键
    controller.check_buttons()

    # 捕获并处理图像
    img = sensor.snapshot()
    processed_img = controller.apply_gamma_correction(img)

    # 显示参数信息
    status_text = controller.get_status_text()
    processed_img.draw_string(10, 10, status_text, color=(255, 255, 255), scale=1)

    # 显示当前预设
    preset_name = controller.presets[controller.preset_index]["name"]
    processed_img.draw_string(10, 25, f"预设: {preset_name}", color=(255, 255, 0), scale=1)

    # 每30帧打印一次状态
    frame_count += 1
    if frame_count % 30 == 0:
        fps = clock.fps()
        print(f"FPS: {fps:.1f}, {status_text}")

    time.sleep_ms(10)
