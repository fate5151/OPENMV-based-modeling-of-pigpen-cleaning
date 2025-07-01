# This work is licensed under the MIT license.
# Copyright (c) 2013-2023 OpenMV LLC. All rights reserved.
# https://github.com/openmv/openmv/blob/master/LICENSE
#
# UART Control
#
# This example shows how to use the serial port on your OpenMV Cam. Attach pin
# P4 to the serial input of a serial LCD screen to see "Hello World!" printed
# on the serial LCD display.

import time,pyb
from pyb import UART


# Always pass UART 3 for the UART number for your OpenMV Cam.
# The second argument is the UART baud rate. For a more advanced UART control
# example see the BLE-Shield driver.

rtc = pyb.RTC()
rtc.datetime((2014, 5, 1, 4, 13, 0, 0, 0))


uart = UART(1, 115200 , timeout_char=200)

data1 = bytearray([0xFF],[0x01,0x02],[0xfe])

data2 =bytearray([0xff,0x01,0x02,0x01,0x02,0xfe])

data3 =bytearray([1,2,3,4,5,15,99,255,256,500])

#data3 =bytearray(0xff,0x01,0x02,0x01,0x02,0xfe)

def send_packet(data):
    # 数据包格式：0xff + 长度(1字节) + 数据 + 校验和
    start_byte = b'\xFF'
    end_byte = b'\xFE'
    #length = bytes([len(data)])
    #checksum = sum(data) & 0xFF  # 简单求和校验
    #packet = start_byte + length + bytes(data) + bytes([checksum])
    packet = start_byte + bytes(data) + end_byte
    uart.write(packet)

while True:
    print(rtc.datetime())

    #uart.write("\x11")
    #time.sleep_ms(1000)
    #uart.write("\x22")
    #time.sleep_ms(1000)
    #uart.write("@LED_ON")
    #time.sleep_ms(1000)
    #uart.write("@LED_OFF")
    #time.sleep_ms(1000)

    #send_packet(0x01)

    #uart.write(data1)
    #print(f"data1:{data1}")
    #time.sleep_ms(1000)

    #uart.write(data2)
    #print(f"data2:{data2}")
    #time.sleep_ms(1000)

    uart.write(data3)
    print(f"data3:{data3}")
    time.sleep_ms(1000)

    #uart.write("hello string!")
    #time.sleep_ms(1000)
    #read_data=uart.read()
    #print(read_data)

    #uart.write("@LED_ON\r\n")
    #time.sleep_ms(2000)
    #uart.write("@LED_OFF\r\n")
    #time.sleep_ms(2000)
    #uart.write("1")
