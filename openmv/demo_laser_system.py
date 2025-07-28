# demo_laser_system.py - UV激光绘图系统演示程序
# 用于演示系统的主要功能和接口

def demo_laser_detection():
    """演示激光检测功能"""
    print("=== UV激光检测功能演示 ===")
    
    try:
        from laser_detection import LaserDetection
        
        # 创建检测器
        detector = LaserDetection()
        
        # 显示检测参数
        print("检测器初始化成功!")
        print(f"HSV阈值数量: {len(detector.laser_thresholds)}")
        print(f"RGB备用阈值数量: {len(detector.laser_thresholds_rgb)}")
        print(f"历史记录长度: {detector.max_history_length}")
        
        # 模拟检测结果
        print("\n模拟检测过程...")
        for i in range(5):
            # 这里应该传入真实图像，演示中使用None
            result = {
                'position': (160 + i*10, 120 + i*5),
                'confidence': 0.8 - i*0.1,
                'method': 'HSV' if i < 3 else 'RGB',
                'details': f'模拟检测{i+1}',
                'fallback_used': False
            }
            print(f"检测{i+1}: 位置{result['position']}, 置信度{result['confidence']:.2f}")
        
        # 显示统计信息
        stats = detector.get_detection_stats()
        print(f"\n检测统计:")
        print(f"- 总尝试: {stats['total_attempts']}")
        print(f"- 成功检测: {stats['successful_detections']}")
        print(f"- 成功率: {stats['success_rate']:.1f}%")
        
    except Exception as e:
        print(f"激光检测演示失败: {e}")
        import traceback
        traceback.print_exc()

def demo_laser_control():
    """演示激光控制功能"""
    print("\n=== UV激光控制功能演示 ===")
    
    try:
        from laser_control import LaserControl
        
        # 创建控制器
        controller = LaserControl()
        
        print("控制器初始化成功!")
        print(f"舵机可用: {controller.servo_available}")
        print(f"自适应控制: {controller.adaptive_control}")
        print(f"目标位置: {controller.target_position}")
        
        # 模拟控制过程
        print("\n模拟控制过程...")
        
        # 模拟不同置信度的检测结果
        test_cases = [
            {'position': (150, 110), 'confidence': 0.9, 'method': 'HSV'},
            {'position': (155, 115), 'confidence': 0.6, 'method': 'RGB'},
            {'position': (145, 125), 'confidence': 0.3, 'method': 'prediction'},
            {'position': None, 'confidence': 0.0, 'method': 'none'}
        ]
        
        for i, detection_result in enumerate(test_cases):
            detection_result.update({
                'details': f'测试用例{i+1}',
                'fallback_used': detection_result['confidence'] < 0.5
            })
            
            result = controller.control_laser(detection_result, debug=True)
            
            print(f"控制{i+1}: 成功={result['success']}, "
                  f"误差={result['error_magnitude']:.1f}, "
                  f"控制器={result['controller_type']}")
        
        # 显示控制统计
        stats = controller.get_control_stats()
        print(f"\n控制统计:")
        print(f"- 控制尝试: {stats['control_attempts']}")
        print(f"- 成功控制: {stats['successful_controls']}")
        print(f"- 成功率: {stats['success_rate']:.1f}%")
        print(f"- 平均误差: {stats['average_error']:.1f}")
        
    except Exception as e:
        print(f"激光控制演示失败: {e}")
        import traceback
        traceback.print_exc()

def demo_system_integration():
    """演示系统集成功能"""
    print("\n=== 系统集成功能演示 ===")
    
    try:
        # 这里通常会导入完整系统，但为了演示我们只显示接口
        print("UV激光绘图系统主要组件:")
        print("1. 激光检测模块 (laser_detection.py)")
        print("   - 多级检测策略")
        print("   - 智能形状验证")
        print("   - 检测质量评估")
        print("   - 智能回退机制")
        
        print("\n2. 激光控制模块 (laser_control.py)")
        print("   - 自适应PID控制")
        print("   - 三级控制策略")
        print("   - 舵机保护机制")
        print("   - 控制历史记录")
        
        print("\n3. 主系统程序 (uv_laser_drawing.py)")
        print("   - 用户界面控制")
        print("   - 实时状态显示")
        print("   - 绘图路径规划")
        print("   - 性能监控")
        
        print("\n4. 测试程序 (test_laser_detection.py)")
        print("   - 检测功能验证")
        print("   - 性能基准测试")
        print("   - 交互式调试")
        
        print("\n系统使用方法:")
        print("1. 导入主系统: from uv_laser_drawing import UVLaserDrawingSystem")
        print("2. 创建实例: system = UVLaserDrawingSystem()")
        print("3. 启动系统: system.run()")
        
        print("\n按键控制:")
        print("- P7: 系统开/关")
        print("- P8: 调试模式")
        print("- P7+P8: 开始/停止绘图")
        print("- P9+P4: 系统统计")
        
    except Exception as e:
        print(f"系统集成演示失败: {e}")

def main():
    """主演示程序"""
    print("UV激光绘图系统 - 功能演示")
    print("=" * 50)
    
    # 演示各个模块
    demo_laser_detection()
    demo_laser_control()
    demo_system_integration()
    
    print("\n" + "=" * 50)
    print("演示完成！")
    print("\n使用说明:")
    print("1. 将所有文件复制到OpenMV设备")
    print("2. 运行 uv_laser_drawing.py 启动完整系统")
    print("3. 运行 test_laser_detection.py 进行测试")
    print("4. 查看 README_UV_LASER.md 获取详细文档")

if __name__ == "__main__":
    main()