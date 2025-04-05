import sensor, image, time

from pid import PID
from pyb import Servo,Pin



# 设置 pin4 和 pin5
pin4 = Pin('P4', Pin.OUT_PP)
pin5 = Pin('P5', Pin.OUT_PP)
pin4.low()  # 设置 pin4 为低电平
pin5.low()   # 设置 pin5 为低电平


pan_servo=Servo(1)   #水平 P7
tilt_servo=Servo(2)  #垂直 P8

# 基础角度控制模式（0°~180°）
pan_servo.calibration(600, 2400,500)  # 设置 min=600μs, max=2400μs, 中心=1500μs
tilt_servo.calibration(600, 2400, 500)

# 若需高级控制（如速度模式）
# pan_servo.calibration(600, 2400, 1500, 1500, 2000)

pig_roi=(55,15,420,410)
red_threshold  =(13, 49, 17, 62, 6, 47)
shift_threshold  = (23, 52, -6, 22, 31, 53)
shift_threshold=red_threshold

#pan_pid = PID(p=0.07, i=0,d=0.001 , imax=90) #脱机运行或者禁用图像传输，使用这个PID
#tilt_pid = PID(p=0.05, i=0, d=0.001 ,imax=90) #脱机运行或者禁用图像传输，使用这个PID
pan_pid = PID(p=0.05, i=0,d=0.001 ,imax=90)#在线调试使用这个PID
tilt_pid = PID(p=0.05, i=0,d=0.001, imax=90)#在线调试使用这个PID

sensor.reset() # Initialize the camera sensor.
sensor.set_pixformat(sensor.RGB565) # use RGB565.
sensor.set_framesize(sensor.VGA) # use QQVGA for speed.
sensor.skip_frames(time=2000) # Let new settings take affect.
sensor.set_auto_whitebal(False) # turn this off.
clock = time.clock() # Tracks FPS.

def find_max(blobs):
    max_size=0
    for blob in blobs:
        if blob[2]*blob[3] > max_size:
            max_blob=blob
            max_size = blob[2]*blob[3]
    return max_blob


tilt_servo.angle(0)
pan_servo.angle(0)


while(True):
    clock.tick() # Track elapsed milliseconds between snapshots().
    img = sensor.snapshot() # Take a picture and return the image.

    print(pan_servo.angle(),tilt_servo.angle())
    #pin4.high()  # 设置 pin4 为高电平

    for i in range(0,90,1):
        tilt_servo.angle(i,100)
     #   pan_servo.angle(i,100)
        print(pan_servo.angle(),tilt_servo.angle())



