import pyb
import time

def set_time(year, month, day, hour, minute, second):
    rtc = pyb.RTC()
    # 设置时间（年, 月, 日, 星期, 时, 分, 秒, 毫秒）
    rtc.datetime((year, month, day, 0, hour, minute, second, 0))

def get_time_str():
    rtc = pyb.RTC()
    dt = rtc.datetime()
    return "%04d-%02d-%02d %02d:%02d:%02d" % (dt[0], dt[1], dt[2], dt[4], dt[5], dt[6])

def get_unix_timestamp():
    dt = pyb.RTC().datetime()
    return time.mktime((dt[0], dt[1], dt[2], dt[4], dt[5], dt[6], 0, 0))

if __name__ == "__main__":
    # 测试时间设置和获取
    set_time(2023, 10, 1, 12, 0, 0)
    print("当前时间:", get_time_str())
    print("Unix时间戳:", get_unix_timestamp())

    # 等待5秒后再次获取时间
    time.sleep(5)
    print("5秒后当前时间:", get_time_str())
    print("5秒后Unix时间戳:", get_unix_timestamp())
