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

pan_servo.calibration(500,2500,500)
tilt_servo.calibration(500,2500,500)


red_threshold  =(13, 49, 17, 62, 6, 47)
shift_threshold  = (23, 52, -6, 22, 31, 53)
#shift_threshold=red_threshold

#pan_pid = PID(p=0.07, i=0,d=0.001 , imax=90) #脱机运行或者禁用图像传输，使用这个PID
#tilt_pid = PID(p=0.05, i=0, d=0.001 ,imax=90) #脱机运行或者禁用图像传输，使用这个PID
pan_pid = PID(p=0.05, i=0,d=0.001 ,imax=90)#在线调试使用这个PID
tilt_pid = PID(p=0.05, i=0,d=0.001, imax=90)#在线调试使用这个PID

sensor.reset() # Initialize the camera sensor.
sensor.set_pixformat(sensor.RGB565) # use RGB565.
sensor.set_framesize(sensor.QVGA) # use QQVGA for speed.
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



while(True):
    clock.tick() # Track elapsed milliseconds between snapshots().
    img = sensor.snapshot() # Take a picture and return the image.

    blobs = img.find_blobs([shift_threshold])
    print(pan_servo.angle(),tilt_servo.angle())


    if blobs:
        max_blob = find_max(blobs)
        pan_error = max_blob.cx()-img.width()/2
        tilt_error = max_blob.cy()-img.height()/2

        #print("pan_error: ", pan_error)

        img.draw_rectangle(max_blob.rect()) # rect
        img.draw_cross(max_blob.cx(), max_blob.cy()) # cx, cy

        pan_output=pan_pid.get_pid(pan_error,1)/2
        tilt_output=tilt_pid.get_pid(tilt_error,1)
        #print("pan_output",pan_output)
        pan_servo.angle(pan_servo.angle()+pan_output)
        tilt_servo.angle(tilt_servo.angle()-tilt_output)

        print(pan_output,tilt_output)

        print("有粪便")
        pin4.high()  # 设置 pin4 为高电平

    else:
        print("没有粪便")
        pin4.low()  # 设置 pin4 为低电平

