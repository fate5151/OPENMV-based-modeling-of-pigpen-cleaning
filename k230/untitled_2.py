import time
from machine import Pin
from machine import FPIOA


#将GPIO52配置为普通GPIO
fpioa = FPIOA()
#fpioa.set_function(19,FPIOA.GPIO19)
#fpioa.set_function(60,FPIOA.GPIO60)


# 定义控制水泵的引脚（根据实际连接修改引脚号）
motorA = Pin(19, Pin.OUT)
motorB = Pin(60, Pin.OUT)

def pump_forward():
    """ 水泵正转：水泵开始工作 """
    motorA.value(1)
    motorB.value(0)
    print("水泵正转")

def pump_reverse():
    """ 水泵反转：用于特殊需要反转水泵（如果支持） """
    motorA.value(0)
    motorB.value(1)
    print("水泵反转")

def pump_stop():
    """ 停止水泵 """
    motorA.value(0)
    motorB.value(0)
    print("水泵停止")

# 示例：先正转 5 秒，再停止
while(True):
    pump_forward()
    time.sleep(5)
    pump_stop()
    time.sleep(2)
