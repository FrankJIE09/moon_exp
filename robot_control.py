# robot_control.py
# -*- coding: utf-8 -*-

import time
import traceback
from CPS import CPSClient # 确保 CPSClient 在 CPS.py 中

# 导入 config 中的常量或直接在这里定义
from config import (TARGET_RESET_RPY_LEFT, TARGET_RESET_RPY_RIGHT,
                    MODE_XYZ, MODE_RPY, MODE_VISION, MODE_RESET)

# 假设 desire_left_pose 和 desire_right_pose 在 CPS.py 中

from CPS import desire_left_pose, desire_right_pose



def initialize_robot(controller, arm_name):
    """初始化单个机器人（上电、清报警、同步、使能）"""
    print(f"\n--- 初始化 {arm_name} ---")
    try:
        print(f"[{arm_name}] 1/4: 上电...")
        if not controller.power_on():
            raise RuntimeError(f"{arm_name} 上电失败")
        print(f"[{arm_name}] 上电成功.")
        time.sleep(0.5)

        print(f"[{arm_name}] 2/4: 清除报警...")
        controller.clearAlarm()
        print(f"[{arm_name}] 清除报警完成.")
        time.sleep(0.5)

        print(f"[{arm_name}] 3/4: 同步电机...")
        if not controller.setMotorStatus():
            print(f"警告: {arm_name} 同步电机失败.")
        else:
            print(f"[{arm_name}] 同步电机成功.")
            time.sleep(0.5)

        print(f"[{arm_name}] 4/4: 伺服使能...")
        if not controller.set_servo(1):
            raise RuntimeError(f"{arm_name} 伺服使能失败")
        print(f"[{arm_name}] 伺服使能成功.")
        print(f"--- {arm_name} 初始化完成 ---")
        return True
    except Exception as e:
        print(f"错误: 初始化 {arm_name} 时: {e}")
        traceback.print_exc()
        return False

def connect_arm_gripper(controller, arm_name):
    """尝试连接并激活单个夹爪"""
    print(f"--- 尝试连接 {arm_name} 夹爪 ---")
    try:
        if controller.connect_gripper():
            print(f"{arm_name} 夹爪连接并激活成功.")
            return True
        else:
            print(f"错误: {arm_name} 夹爪连接或激活失败.")
            return False
    except AttributeError:
        print(f"错误: 控制器对象无 'connect_gripper' 方法.")
        return False
    except Exception as e:
        print(f"连接 {arm_name} 夹爪时异常: {e}")
        return False

def format_speed(speed_array):
    """格式化速度向量以便打印"""
    if not hasattr(speed_array, '__len__') or len(speed_array) < 6:
        return "[无效速度数据]"
    try:
        return (f"[{float(speed_array[0]):>6.1f}, {float(speed_array[1]):>6.1f}, {float(speed_array[2]):>6.1f}, "
                f"{float(speed_array[3]):>6.1f}, {float(speed_array[4]):>6.1f}, {float(speed_array[5]):>6.1f}]")
    except (ValueError, IndexError):
        return "[格式化错误]"

def run_vision_mode():
    """视觉模式占位符"""
    # print("== 进入视觉模式 (占位符) ==")
    # 在这里实现视觉模式的具体逻辑
    pass

def attempt_reset_arm(controller, arm_name, target_rpy, reset_speed, desire_pose_func, move_func_name, sound_player=None, success_sound=None, fail_sound=None):
    """尝试将指定机械臂回正到目标姿态 (通用函数)"""
    print(f"尝试将 {arm_name} 回正到垂直姿态...")
    try:
        current_pose = controller.getTCPPose()
        if current_pose is None:
            print(f"错误：无法获取 {arm_name} 当前 TCP 位姿。")
            if sound_player and fail_sound: sound_player(fail_sound)
            return False

        try:
            rpy_angles = desire_pose_func(rpy_array=target_rpy)
        except NameError:
             print(f"错误: 函数 {desire_pose_func.__name__} 未定义!")
             if sound_player and fail_sound: sound_player(fail_sound)
             return False
        except Exception as e_desire:
             print(f"错误: 调用 {desire_pose_func.__name__} 时出错: {e_desire}")
             if sound_player and fail_sound: sound_player(fail_sound)
             return False

        target_pose = list(current_pose) # 创建副本
        target_pose[3:6] = rpy_angles    # 仅修改姿态部分
        print(f"  {arm_name} 目标位姿 (仅姿态): {target_pose}")

        # 获取移动方法
        move_method = getattr(controller, move_func_name, None)
        if move_method is None:
             print(f"错误: 控制器对象缺少方法 '{move_func_name}'")
             if sound_player and fail_sound: sound_player(fail_sound)
             return False

        # 调用移动方法
        success = move_method(target_pose, speed=reset_speed, block=True)

        if success:
            print(f"{arm_name} 回正成功。")
            if sound_player and success_sound: sound_player(success_sound)
            return True
        else:
            print(f"{arm_name} 回正失败。 ({move_func_name} 返回 False)")
            if sound_player and fail_sound: sound_player(fail_sound)
            return False

    except AttributeError as ae:
         print(f"{arm_name} 回正错误: 控制器对象缺少方法 (可能是 getTCPPose 或 {move_func_name}): {ae}")
         traceback.print_exc()
         if sound_player and fail_sound: sound_player(fail_sound)
         return False
    except Exception as e:
        print(f"{arm_name} 回正过程中发生异常: {e}")
        traceback.print_exc()
        if sound_player and fail_sound: sound_player(fail_sound)
        return False

def map_speed_to_jog(speed, min_speed, max_speed):
    """将速度值映射到 Jog 指令所需的百分比 (示例)"""
    # 这个映射关系需要根据你的机器人控制器 API 定义来调整
    jog_speed = abs(speed)
    # 假设 Jog 速度是 0-100 的百分比，需要将物理速度映射过去
    # 这里的映射逻辑是简化的，你需要根据实际情况修改
    # 例如，如果最大速度 100.0 对应 jog 100%
    mapped_percentage = (jog_speed / max_speed) * 100 if max_speed > 0 else 0
    # 确保在合理范围内，例如 Jog 可能不允许 0%
    mapped_percentage = max(0.1, min(100.0, mapped_percentage)) # 假设最小 0.1%
    # 或者直接使用速度值，如果 Jog API 接受物理速度
    # mapped_speed = max(min_speed, min(max_speed, jog_speed))
    # return mapped_speed
    return mapped_percentage # 返回百分比示例

def send_jog_command(controller, speed_vector, min_speed, max_speed):
    """根据速度向量发送 Jog 指令 (适用于 RPY 模式)"""
    try:
        # 线性轴 X (index 0: +, index 1: -)
        if speed_vector[0] > 0.05: controller.jog(index=0, speed=map_speed_to_jog(speed_vector[0], min_speed, max_speed))
        elif speed_vector[0] < -0.05: controller.jog(index=1, speed=map_speed_to_jog(-speed_vector[0], min_speed, max_speed))
        # 线性轴 Y (index 2: +, index 3: -)
        if speed_vector[1] > 0.05: controller.jog(index=2, speed=map_speed_to_jog(speed_vector[1], min_speed, max_speed))
        elif speed_vector[1] < -0.05: controller.jog(index=3, speed=map_speed_to_jog(-speed_vector[1], min_speed, max_speed))
        # 线性轴 Z (index 4: +, index 5: -)
        if speed_vector[2] > 0.05: controller.jog(index=4, speed=map_speed_to_jog(speed_vector[2], min_speed, max_speed))
        elif speed_vector[2] < -0.05: controller.jog(index=5, speed=map_speed_to_jog(-speed_vector[2], min_speed, max_speed))
        # 旋转轴 Rx (index 6: +, index 7: -)
        if speed_vector[3] > 0.05: controller.jog(index=6, speed=map_speed_to_jog(speed_vector[3], min_speed, max_speed))
        elif speed_vector[3] < -0.05: controller.jog(index=7, speed=map_speed_to_jog(-speed_vector[3], min_speed, max_speed))
        # 旋转轴 Ry (index 8: +, index 9: -)
        if speed_vector[4] > 0.05: controller.jog(index=8, speed=map_speed_to_jog(speed_vector[4], min_speed, max_speed))
        elif speed_vector[4] < -0.05: controller.jog(index=9, speed=map_speed_to_jog(-speed_vector[4], min_speed, max_speed))
        # 旋转轴 Rz (index 10: +, index 11: -)
        if speed_vector[5] > 0.05: controller.jog(index=10, speed=map_speed_to_jog(speed_vector[5], min_speed, max_speed))
        elif speed_vector[5] < -0.05: controller.jog(index=11, speed=map_speed_to_jog(-speed_vector[5], min_speed, max_speed))
        # 如果所有速度分量都接近零，可能需要发送停止指令
        # controller.stop_jog() # 假设有这个方法
    except Exception as e:
        print(f"发送 Jog 指令时失败: {e}")