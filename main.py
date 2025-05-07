# main.py
# -*- coding: utf-8 -*-

import traceback
from main_controller import DualArmController
import config # 导入 config 以便访问 CONFIG_FILE

if __name__ == "__main__":
    print("启动双臂控制器...")
    # 使用 config.py 中定义的配置文件路径
    controller_app = DualArmController(config_path=config.CONFIG_FILE)

    try:
        # 执行设置（加载配置、初始化 Pygame 和机器人）
        if controller_app.setup():
            # 如果设置成功，运行主循环
            controller_app.run_main_loop()
        else:
            print("控制器设置失败，无法启动主循环。")

    except KeyboardInterrupt:
        print("\n检测到手动中断 (Ctrl+C)... 正在清理...")
    except Exception as e:
        print("\n主程序运行时发生未捕获的异常:")
        traceback.print_exc() # 打印完整的错误信息
    finally:
        # 无论如何，执行清理操作
        controller_app.cleanup()

    print("应用程序已结束。")