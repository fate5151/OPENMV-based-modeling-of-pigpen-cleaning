# 本作品采用MIT许可证授权。
# 版权所有 (c) 2013-2023 OpenMV LLC。保留所有权利。
# https://github.com/openmv/openmv/blob/master/LICENSE
#
# 伽马校正
#
# 这个示例展示了伽马校正以使图像更亮。伽马
# 校正方法还可以同时修正对比度和亮度。

import sensor
import time
from pyb import Pin

gamma=1.0
contrast=1.0
brightness=0.0

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
clock = time.clock()

pin_gamma_add =     Pin('P7', Pin.IN, Pin.PULL_UP)
pin_gamma_reduce = Pin('P8', Pin.IN, Pin.PULL_UP)
pin_contrast_add = Pin('P9', Pin.IN, Pin.PULL_UP)
pin_contrast_reduce = Pin('P0', Pin.IN, Pin.PULL_UP)
pin_brightness_add = Pin('P3', Pin.IN, Pin.PULL_UP)
pin_brightness_reduce = Pin('P2', Pin.IN, Pin.PULL_UP)

print(f"pin_gamma_add:{pin_gamma_add.value()}")
print(f"pin_gamma_reduce:{pin_gamma_reduce.value()}")
print(f"pin_contrast_add:{pin_contrast_add.value()}")
print(f"pin_contrast_reduce:{pin_contrast_reduce.value()}")
print(f"pin_brightness_add:{pin_brightness_add.value()}")
print(f"pin_brightness_reduce:{pin_brightness_reduce.value()}")

while True:
    clock.tick()

    # 伽马校正、对比度及亮度调整会分别应用于每个颜色通道。
    # 数值会根据图像类型及每个颜色通道的范围进行缩放...
    img = sensor.snapshot().gamma_corr(gamma=gamma, contrast=contrast, brightness=brightness)

    if pin_gamma_add.value() == 0:
        while pin_gamma_add.value() == False:
            gamma +=0.1
            img = img.gamma_corr(gamma=gamma)
        print(f"Gamma增加,pin_gamma_add:{pin_gamma_add.value()}")

    status_text = f"Gamma:{gamma:.1f}, Contrast:{contrast:.1f}, Brightness:{brightness:.1f}"
    img.draw_string(10, 10, status_text, color=(255, 255, 255), scale=1)
    #print(f"Gamma:{gamma:.1f}, Contrast:{contrast:.1f}, Brightness:{brightness:.1f}")
    # print(f"pin_gamma_add:{pin_gamma_add.value()}")
    # print(f"pin_gamma_reduce:{pin_gamma_reduce.value()}")
    # print(f"pin_contrast_add:{pin_contrast_add.value()}")
    # print(f"pin_contrast_reduce:{pin_contrast_reduce.value()}")
    # print(f"pin_brightness_add:{pin_brightness_add.value()}")
    # print(f"pin_brightness_reduce:{pin_brightness_reduce.value()}")
    # print("=============================================")

    #print(clock.fps())
