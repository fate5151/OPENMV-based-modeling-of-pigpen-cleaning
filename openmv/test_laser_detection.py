# test_laser_detection.py - 激光检测功能测试程序
# 用于验证激光检测算法的准确性和可靠性

import sensor, image, time
from laser_detection import LaserDetection

def test_laser_detection():
    """测试激光检测功能"""
    print("开始激光检测测试...")
    
    # 初始化摄像头
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)  # 320x240
    sensor.skip_frames(time=2000)
    sensor.set_auto_whitebal(False)
    
    # 创建激光检测器
    detector = LaserDetection()
    
    # 测试参数
    test_duration = 30000  # 30秒测试
    start_time = time.ticks_ms()
    frame_count = 0
    detection_count = 0
    
    clock = time.clock()
    
    print("测试开始，持续30秒...")
    print("请确保有蓝紫色激光点在视野中")
    
    while time.ticks_diff(time.ticks_ms(), start_time) < test_duration:
        clock.tick()
        frame_count += 1
        
        # 捕获图像
        img = sensor.snapshot()
        
        # 激光检测
        result = detector.get_current_laser_position(img, debug=True)
        
        if result['position'] is not None:
            detection_count += 1
            x, y = result['position']
            confidence = result['confidence']
            method = result['method']
            
            # 绘制检测结果
            color = (0, 255, 0) if confidence > 0.7 else (255, 255, 0) if confidence > 0.4 else (255, 0, 0)
            img.draw_circle((x, y, 20), color=color, thickness=3)
            img.draw_cross((x, y), color=color, size=15, thickness=3)
            
            # 显示信息
            img.draw_string(10, 10, f"位置: ({x}, {y})", color=(255, 255, 255), scale=2)
            img.draw_string(10, 30, f"置信度: {confidence:.2f}", color=(255, 255, 255), scale=2)
            img.draw_string(10, 50, f"方法: {method}", color=(255, 255, 255), scale=2)
            
            if result['fallback_used']:
                img.draw_string(10, 70, "回退策略", color=(255, 0, 0), scale=2)
        else:
            # 未检测到
            img.draw_string(10, 10, "未检测到激光", color=(255, 0, 0), scale=2)
        
        # 显示统计信息
        fps = clock.fps()
        detection_rate = (detection_count / frame_count) * 100 if frame_count > 0 else 0
        
        img.draw_string(10, 200, f"FPS: {fps:.1f}", color=(0, 255, 255), scale=1)
        img.draw_string(10, 215, f"检测率: {detection_rate:.1f}%", color=(0, 255, 255), scale=1)
        
        # 每100帧打印一次统计
        if frame_count % 100 == 0:
            print(f"帧数: {frame_count}, 检测数: {detection_count}, 检测率: {detection_rate:.1f}%")
            detector.print_detection_stats()
    
    # 测试结束，打印最终统计
    print("\n=== 测试完成 ===")
    print(f"总帧数: {frame_count}")
    print(f"检测次数: {detection_count}")
    print(f"总检测率: {(detection_count / frame_count) * 100:.1f}%")
    
    detector.print_detection_stats()
    print("================")

def test_threshold_adjustment():
    """测试阈值调整功能"""
    print("开始阈值调整测试...")
    
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.skip_frames(time=2000)
    sensor.set_auto_whitebal(False)
    
    detector = LaserDetection()
    
    # 测试不同的阈值调整
    adjustments = [
        (0, 0, 10),   # 阈值0，H_min参数，+10
        (0, 1, -10),  # 阈值0，H_max参数，-10
        (1, 2, 5),    # 阈值1，S_min参数，+5
        (1, 3, -5),   # 阈值1，S_max参数，-5
    ]
    
    for threshold_idx, param_idx, delta in adjustments:
        print(f"调整阈值{threshold_idx}参数{param_idx}: {delta}")
        detector.adjust_thresholds(threshold_idx, param_idx, delta)
        
        # 测试调整后的效果
        for i in range(10):
            img = sensor.snapshot()
            result = detector.get_current_laser_position(img, debug=False)
            if result['position']:
                print(f"  测试{i+1}: 检测成功，置信度{result['confidence']:.2f}")
            else:
                print(f"  测试{i+1}: 检测失败")
        
        time.sleep_ms(500)
    
    print("阈值调整测试完成")

def interactive_test():
    """交互式测试模式"""
    print("交互式激光检测测试")
    print("按P7退出，按P8重置检测器")
    
    # 初始化按键
    try:
        button_exit = pyb.Pin('P7', pyb.Pin.IN, pyb.Pin.PULL_UP)
        button_reset = pyb.Pin('P8', pyb.Pin.IN, pyb.Pin.PULL_UP)
        buttons_available = True
    except:
        print("按键不可用，将运行固定时间")
        buttons_available = False
    
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.skip_frames(time=2000)
    
    detector = LaserDetection()
    clock = time.clock()
    
    start_time = time.ticks_ms()
    
    while True:
        clock.tick()
        
        # 检查退出条件
        if buttons_available:
            if not button_exit.value():  # P7按下
                print("用户退出")
                break
            if not button_reset.value():  # P8按下
                print("重置检测器")
                detector.reset_history()
                time.sleep_ms(500)  # 防抖
        else:
            # 无按键时运行60秒后退出
            if time.ticks_diff(time.ticks_ms(), start_time) > 60000:
                print("测试时间结束")
                break
        
        # 图像处理
        img = sensor.snapshot()
        result = detector.get_current_laser_position(img, debug=True)
        
        # 显示基本信息
        fps = clock.fps()
        img.draw_string(250, 10, f"FPS:{fps:.1f}", color=(255, 255, 255), scale=1)
        
        if result['position']:
            x, y = result['position']
            img.draw_string(250, 25, f"X:{x}", color=(0, 255, 0), scale=1)
            img.draw_string(250, 40, f"Y:{y}", color=(0, 255, 0), scale=1)
            img.draw_string(250, 55, f"C:{result['confidence']:.2f}", color=(0, 255, 0), scale=1)
        else:
            img.draw_string(250, 25, "NO LASER", color=(255, 0, 0), scale=1)
        
        time.sleep_ms(20)
    
    # 打印最终统计
    detector.print_detection_stats()

if __name__ == "__main__":
    try:
        print("选择测试模式:")
        print("1. 基础检测测试")
        print("2. 阈值调整测试") 
        print("3. 交互式测试")
        
        # 默认运行交互式测试
        interactive_test()
        
    except Exception as e:
        print(f"测试失败: {e}")
        import traceback
        traceback.print_exc()