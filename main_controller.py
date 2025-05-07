# main_controller.py
# -*- coding: utf-8 -*-
import pygame
import time
import numpy as np
from scipy.spatial.transform import Rotation as R
import traceback
import threading
import vision_interaction
import os
# 导入其他模块
import config
from robot_control import (initialize_robot, connect_arm_gripper, format_speed,
                           run_vision_mode, attempt_reset_arm, send_jog_command)
from ui import UIManager
from CPS import CPSClient # 假设 CPSClient 在 CPS.py

# 导入常量
from config import (MODE_XYZ, MODE_RPY, MODE_VISION, MODE_RESET,
                    TARGET_RESET_RPY_LEFT, TARGET_RESET_RPY_RIGHT)
# 假设 desire_left_pose 和 desire_right_pose 在 CPS.py 或 robot_control.py 中导入
try:
    from robot_control import desire_left_pose, desire_right_pose
except ImportError:
    # 如果 robot_control.py 中也没有，需要处理
    print("错误: desire_left_pose/desire_right_pose 未在任何地方定义!")
    exit()


class DualArmController:
    """
    使用 Pygame 手柄和配置文件控制双臂机器人（带模式切换和音频反馈）的主类。
    """
    def __init__(self, config_path=config.CONFIG_FILE):
        """初始化控制器"""
        self.config_path = config_path
        self.config = None # 将由 config.load_and_set_config_variables 加载
        self.ui_manager = UIManager(self) # 创建 UI 管理器实例

        # 机器人控制器实例
        self.controller_left = None
        self.controller_right = None

        # 状态变量 (部分由 config 加载，部分在运行时设置)
        self.running = False
        self.control_modes = [MODE_XYZ, MODE_RPY, MODE_RESET, MODE_VISION]
        self.current_mode_index = 0
        self.control_mode = self.control_modes[self.current_mode_index]

        # 速度和参数 (将由 config 加载)
        self.current_speed_xy = config.DEFAULT_XY_SPEED
        self.current_speed_z = config.DEFAULT_Z_SPEED
        self.rpy_speed = config.DEFAULT_RPY_SPEED
        self.reset_speed = config.DEFAULT_RESET_SPEED
        self.min_speed = config.DEFAULT_MIN_SPEED
        self.max_speed = config.DEFAULT_MAX_SPEED
        self.speed_increment = config.DEFAULT_SPEED_INCREMENT
        self.acc = config.DEFAULT_ACC
        self.arot = config.DEFAULT_AROT
        self.t_interval = config.DEFAULT_T
        self.trigger_threshold = config.DEFAULT_TRIGGER_THRESHOLD
        self.gripper_speed = config.DEFAULT_GRIPPER_SPEED
        self.gripper_force = config.DEFAULT_GRIPPER_FORCE
        self.long_press_duration = config.DEFAULT_LONG_PRESS_DURATION

        # 夹爪状态
        self.left_gripper_open = True
        self.right_gripper_open = True
        self.left_gripper_active = False
        self.right_gripper_active = False

        # 机器人状态
        self.left_init_ok = False
        self.right_init_ok = False

        # 控制映射 (将由 config 加载)
        self.controls_map = {}
        self.audio_files_config = {}
        self.mode_switch_control = {}
        self.speed_inc_control = {}
        self.speed_dec_control = {}
        self.gripper_toggle_left_ctrl = {}
        self.gripper_toggle_right_ctrl = {}
        self.reset_left_arm_ctrl = None
        self.reset_right_arm_ctrl = None

        # UI 相关状态 (将由 config 加载)
        self.font_path = config.DEFAULT_FONT_PATH
        self.window_width = config.DEFAULT_WINDOW_WIDTH
        self.window_height = config.DEFAULT_WINDOW_HEIGHT
        self.font_size = config.DEFAULT_FONT_SIZE

        # 机器人连接信息 (将由 config 加载)
        self.left_robot_ip = config.DEFAULT_IP
        self.right_robot_ip = config.DEFAULT_IP
        self.left_gripper_id = config.DEFAULT_GRIPPER_ID
        self.right_gripper_id = config.DEFAULT_GRIPPER_ID

        # 内部计时器
        self.mode_button_press_time = None

    def _threaded_process_vision_audio(self, audio_filepath):
        """
        此方法在单独的线程中运行，以发送音频、接收和处理响应，
        避免阻塞 Pygame 主循环。
        """
        print(f"[Controller] Thread: Processing recorded audio: {audio_filepath}")
        # 可以在这里安全地调用 self.ui_manager.update_display_message 如果该方法是线程安全的
        # 或者通过队列/事件机制更新UI状态

        received_audio_path, command_json = vision_interaction.send_audio_and_receive_response(audio_filepath)

        if os.path.exists(audio_filepath):  # 清理发送的临时文件
            try:
                os.remove(audio_filepath)
            except Exception as e:
                print(f"[Controller] Thread: Error deleting sent audio temp file: {e}")

        if received_audio_path:
            print("[Controller] Thread: Playing server audio response.")
            self.ui_manager.play_sound('server_response_received')  # 可选：提示音
            vision_interaction.play_audio_file(received_audio_path)  # vision_interaction 负责播放
            if os.path.exists(received_audio_path):  # 清理接收的临时文件
                try:
                    os.remove(received_audio_path)
                except Exception as e:
                    print(f"[Controller] Thread: Error deleting received audio temp file: {e}")

        if command_json:
            print(f"[Controller] Thread: Handling server JSON command: {command_json}")
            vision_interaction.handle_command_json(command_json)  # vision_interaction 负责处理JSON

        print("[Controller] Thread: Vision audio processing finished.")
        # 可以在这里更新UI提示用户操作完成
        # self.ui_manager.update_display_message("视觉模式: 操作完成. 按X(手柄)开始下一次录音")

    def setup(self):
        """执行所有初始化步骤"""
        print("开始设置双臂控制器...")
        # 1. 加载配置
        if not config.load_and_set_config_variables(self, self.config_path):
            print("错误：加载配置失败，程序终止。")
            return False

        # 2. 初始化 Pygame 和 UI
        if not self.ui_manager.init_pygame():
            print("错误：初始化 Pygame 失败，程序终止。")
            return False

        # 3. 初始化机器人
        if not self._init_robots():
            print("警告：机器人初始化未完全成功。")
            # 这里可以选择是否继续运行，取决于需求
            # return False # 如果必须完全初始化才能运行

        # 4. 播放准备就绪声音
        self.ui_manager.play_sound('system_ready')
        self.running = True
        print("--- 控制器设置完成 ---")
        return True

    def _init_robots(self):
        """连接并初始化机器人和夹爪"""
        print("\n正在连接和初始化机器人...")
        all_ok = True
        try:
            # 初始化左臂
            print(f"连接左臂 ({self.left_robot_ip})...")
            self.controller_left = CPSClient(self.left_robot_ip, gripper_slave_id=self.left_gripper_id)
            if not self.controller_left.connect():
                raise ConnectionError(f"左臂 ({self.left_robot_ip}) 连接失败")
            print("左臂连接成功.")
            self.left_init_ok = initialize_robot(self.controller_left, "左臂")
            if self.left_init_ok:
                self.left_gripper_active = connect_arm_gripper(self.controller_left, "左臂")
            else:
                all_ok = False

            # 初始化右臂
            print(f"连接右臂 ({self.right_robot_ip})...")
            self.controller_right = CPSClient(self.right_robot_ip, gripper_slave_id=self.right_gripper_id)
            if not self.controller_right.connect():
                raise ConnectionError(f"右臂 ({self.right_robot_ip}) 连接失败")
            print("右臂连接成功.")
            self.right_init_ok = initialize_robot(self.controller_right, "右臂")
            if self.right_init_ok:
                self.right_gripper_active = connect_arm_gripper(self.controller_right, "右臂")
            else:
                all_ok = False

            if not all_ok:
                print("警告：一个或两个机械臂未能完全初始化。")

            print("机器人初始化流程结束。")
            return all_ok # 返回整体初始化状态

        except ConnectionError as ce:
            print(f"机器人连接错误: {ce}")
            all_ok = False
        except Exception as e:
            print(f"机器人连接或初始化过程中断: {e}")
            traceback.print_exc()
            # 确保即使出错，状态也被标记为失败
            if not self.left_init_ok: self.left_init_ok = False
            if not self.right_init_ok: self.right_init_ok = False
            all_ok = False
        finally:
             # 返回最终的初始化状态，即使有异常也返回
             return all_ok


    def switch_control_mode(self):
        """切换到下一个控制模式"""
        self.current_mode_index = (self.current_mode_index + 1) % len(self.control_modes)
        self.control_mode = self.control_modes[self.current_mode_index]
        print(f"切换到模式: {self.control_mode}")

        # 播放模式切换声音
        if self.control_mode == MODE_XYZ:
            self.ui_manager.play_sound('xyz_mode')
        elif self.control_mode == MODE_RPY:
            self.ui_manager.play_sound('rpy_mode')
        elif self.control_mode == MODE_RESET:
            self.ui_manager.play_sound('reset_mode')
        elif self.control_mode == MODE_VISION:
            self.ui_manager.play_sound('vision_enter') # 进入视觉模式的声音

        # 更新窗口标题
        if self.ui_manager.screen:
             pygame.display.set_caption(f"双臂控制 (模式: {self.control_mode})")


    def toggle_gripper(self, side):
        """切换指定侧夹爪的状态"""
        if side == 'left':
            if self.left_gripper_active and self.controller_left:
                sound_event = 'left_close' if self.left_gripper_open else 'left_open'
                self.ui_manager.play_sound(sound_event)
                action = self.controller_left.close_gripper if self.left_gripper_open else self.controller_left.open_gripper
                try:
                    action(speed=self.gripper_speed, force=self.gripper_force, wait=False)
                    self.left_gripper_open = not self.left_gripper_open
                    print(f"左夹爪切换为: {'打开' if self.left_gripper_open else '关闭'}")
                except Exception as e:
                     print(f"切换左夹爪时出错: {e}")
            else:
                self.ui_manager.play_sound('gripper_inactive')
                print("左夹爪无效或未初始化")
        elif side == 'right':
            if self.right_gripper_active and self.controller_right:
                sound_event = 'right_close' if self.right_gripper_open else 'right_open'
                self.ui_manager.play_sound(sound_event)
                action = self.controller_right.close_gripper if self.right_gripper_open else self.controller_right.open_gripper
                try:
                    action(speed=self.gripper_speed, force=self.gripper_force, wait=False)
                    self.right_gripper_open = not self.right_gripper_open
                    print(f"右夹爪切换为: {'打开' if self.right_gripper_open else '关闭'}")
                except Exception as e:
                     print(f"切换右夹爪时出错: {e}")
            else:
                self.ui_manager.play_sound('gripper_inactive')
                print("右夹爪无效或未初始化")


    def attempt_reset_left_arm(self):
        """尝试将左臂回正"""
        if not self.left_init_ok or not self.controller_left:
            print("左臂未初始化，无法回正。")
            self.ui_manager.play_sound('left_reset_fail')
            return
        attempt_reset_arm(self.controller_left, "左臂", TARGET_RESET_RPY_LEFT,
                          self.reset_speed, desire_left_pose, 'move_robot',
                          self.ui_manager.play_sound, 'left_reset_success', 'left_reset_fail')

    def attempt_reset_right_arm(self):
        """尝试将右臂回正"""
        if not self.right_init_ok or not self.controller_right:
            print("右臂未初始化，无法回正。")
            self.ui_manager.play_sound('right_reset_fail')
            return
        # 注意: move_func_name 可能需要根据 CPSClient 的实现调整 (例如 'move_right_robot')
        attempt_reset_arm(self.controller_right, "右臂", TARGET_RESET_RPY_RIGHT,
                          self.reset_speed, desire_right_pose, 'move_right_robot', # 假设右臂用 move_right_robot
                          self.ui_manager.play_sound, 'right_reset_success', 'right_reset_fail')

    def _calculate_speed_commands(self):
        """根据当前模式和输入计算原始速度指令"""
        speed_left_cmd = np.zeros(6)
        speed_right_cmd = np.zeros(6)
        current_mode_prefix = self.control_mode.lower() + "_" # e.g., "xyz_" or "rpy_"

        if self.control_mode in [MODE_XYZ, MODE_RPY]:
            for action, control in self.controls_map.items():
                if not isinstance(control, dict) or 'type' not in control: continue # 跳过无效配置

                # 检查是否是当前模式的操作，并且指定了手臂
                if action.startswith(current_mode_prefix) and ('_arm' in action):
                     # 使用 UI Manager 检查输入状态
                     if self.ui_manager.get_joystick_input_state(control):
                        target_speed_array = None
                        axis_index = -1
                        base_speed = 0.0
                        direction_sign = 1.0

                        # 判断目标手臂
                        if 'left_arm' in action:
                            target_speed_array = speed_left_cmd
                        elif 'right_arm' in action:
                            target_speed_array = speed_right_cmd

                        if target_speed_array is not None:
                            # 判断目标轴和基础速度
                            if self.control_mode == MODE_XYZ:
                                if '_x' in action: axis_index = 0; base_speed = self.current_speed_xy
                                elif '_y' in action: axis_index = 1; base_speed = self.current_speed_xy
                                elif '_z' in action: axis_index = 2; base_speed = self.current_speed_z
                            elif self.control_mode == MODE_RPY:
                                if '_roll' in action: axis_index = 3; base_speed = self.rpy_speed
                                elif '_pitch' in action: axis_index = 4; base_speed = self.rpy_speed
                                elif '_yaw' in action: axis_index = 5; base_speed = self.rpy_speed

                            # 判断方向
                            if '_pos' in action: direction_sign = 1.0
                            elif '_neg' in action: direction_sign = -1.0

                            # 设置速度
                            if axis_index != -1:
                                # 对于轴输入，可以根据轴的值缩放速度
                                if control.get('type') == 'axis' and self.ui_manager.joystick:
                                    try:
                                        axis_val = self.ui_manager.joystick.get_axis(control.get('index', -1))
                                        # 根据阈值和方向调整基础速度的缩放
                                        # threshold = control.get('threshold', self.trigger_threshold)
                                        # effective_axis_val = max(0, abs(axis_val) - threshold) / (1 - threshold) if threshold < 1 else abs(axis_val)
                                        # scaled_speed = base_speed * effective_axis_val
                                        # 使用原始轴值可能更直观
                                        scaled_speed = base_speed * abs(axis_val)
                                        target_speed_array[axis_index] = scaled_speed * direction_sign
                                    except pygame.error:
                                        target_speed_array[axis_index] = 0 # 手柄读取错误时速度为0
                                else: # 按钮或其他输入，直接使用基础速度
                                    target_speed_array[axis_index] = base_speed * direction_sign

        elif self.control_mode == MODE_VISION:
            # run_vision_mode() # 执行视觉模式逻辑（当前是占位符）
            speed_left_cmd = np.zeros(6) # 视觉模式下通常由视觉算法生成目标，手柄不直接控制速度
            speed_right_cmd = np.zeros(6)
        elif self.control_mode == MODE_RESET:
            # 回正模式下，手柄不产生速度指令，而是触发回正动作
            speed_left_cmd = np.zeros(6)
            speed_right_cmd = np.zeros(6)

        return speed_left_cmd, speed_right_cmd


    def _apply_transformations(self, speed_left_cmd, speed_right_cmd):
        """对速度指令应用坐标变换 (根据需要)"""
        # 注意：这里的坐标变换是示例性的，你需要根据你的机器人基座和工具坐标系来确定正确的变换
        speed_left_final = np.zeros(6)
        speed_right_final = np.zeros(6)

        if self.control_mode == MODE_XYZ:
            # --- 示例：应用旋转矩阵到 XYZ 速度 ---
            speed_left_final = speed_left_cmd.copy()
            speed_right_final = speed_right_cmd.copy()
            # 注意：这些旋转是硬编码的示例，实际应用中可能需要从配置加载或动态计算
            try:
                # 左臂旋转 (示例)
                rot_left_matrix = R.from_euler('xyz', [65, 0, 10], degrees=True).as_matrix()
                speed_left_final[0:3] = speed_left_cmd[0:3] @ rot_left_matrix # 速度向量右乘旋转矩阵
            except Exception as e:
                print(f"左臂 XYZ 转换错误: {e}")
                speed_left_final[0:3] = [0, 0, 0] # 出错时清零

            try:
                # 右臂旋转 (示例)
                rot_right_matrix = R.from_euler('xyz', [65.334, -4.208, -9.079], degrees=True).as_matrix()
                speed_right_final[0:3] = speed_right_cmd[0:3] @ rot_right_matrix
            except Exception as e:
                print(f"右臂 XYZ 转换错误: {e}")
                speed_right_final[0:3] = [0, 0, 0]

            # XYZ 模式下，RPY 速度通常为 0
            speed_left_final[3:] = 0.0
            speed_right_final[3:] = 0.0
            # --- 示例结束 ---

        elif self.control_mode == MODE_RPY:
            # RPY 模式下，通常不应用 XYZ 变换，直接使用 RPY 速度
            # XYZ 速度通常为 0
            speed_left_final[0:3] = 0.0
            speed_right_final[0:3] = 0.0
            # 使用原始的 RPY 速度指令
            speed_left_final[3:] = speed_left_cmd[3:]
            speed_right_final[3:] = speed_right_cmd[3:]

        elif self.control_mode in [MODE_RESET, MODE_VISION]:
            # 这些模式下速度指令通常为 0，由特定动作控制
            speed_left_final = np.zeros(6)
            speed_right_final = np.zeros(6)

        return speed_left_final, speed_right_final

    def _send_robot_commands(self, speed_left_final, speed_right_final):
        """根据当前模式发送最终的速度指令到机器人"""
        # 只在 XYZ 或 RPY 模式下发送持续的速度指令
        if self.control_mode == MODE_XYZ:
            # 使用 moveBySpeedl 发送笛卡尔空间速度
            if self.controller_left and self.left_init_ok:
                try:
                    # print(f"左 Speedl: {list(speed_left_final)}") # Debug
                    self.controller_left.moveBySpeedl(list(speed_left_final), self.acc, self.arot, self.t_interval)
                except Exception as e:
                    print(f"发送左臂 Speedl 指令失败: {e}")
                    # self.left_init_ok = False # 可选：标记为通信失败
            if self.controller_right and self.right_init_ok:
                try:
                    # print(f"右 Speedl: {list(speed_right_final)}") # Debug
                    self.controller_right.moveBySpeedl(list(speed_right_final), self.acc, self.arot, self.t_interval)
                except Exception as e:
                    print(f"发送右臂 Speedl 指令失败: {e}")
                    # self.right_init_ok = False # 可选

        elif self.control_mode == MODE_RPY:
            # 使用 jog 发送基于轴或笛卡尔方向的连续运动指令
            if self.controller_left and self.left_init_ok:
                # print(f"左 Jog Vec: {speed_left_final}") # Debug
                 send_jog_command(self.controller_left, speed_left_final, self.min_speed, self.max_speed)
            if self.controller_right and self.right_init_ok:
                # print(f"右 Jog Vec: {speed_right_final}") # Debug
                 send_jog_command(self.controller_right, speed_right_final, self.min_speed, self.max_speed)

        # 对于 VISION 和 RESET 模式，通常不在这里发送持续速度指令
        # 它们的动作由特定事件触发 (如按钮按下或视觉处理完成)

    def run_main_loop(self):
        """主运行循环"""
        print("\n--- 控制循环开始 ---")
        speed_left_final = np.zeros(6)
        speed_right_final = np.zeros(6)

        while self.running:
            start_time = time.time()

            # 1. 处理事件 (退出、按钮、模式切换等)
            self.ui_manager.handle_events()
            if not self.running: break # 如果事件处理中设置了退出标志

            # 2. 计算速度指令 (基于手柄输入和当前模式)
            speed_left_cmd, speed_right_cmd = self._calculate_speed_commands()

            # 3. 应用坐标变换 (如果需要)
            speed_left_final, speed_right_final = self._apply_transformations(speed_left_cmd, speed_right_cmd)

            # 4. 发送指令到机器人 (仅在 XYZ/RPY 模式)
            # 检查机器人是否初始化成功才发送指令
            if self.left_init_ok or self.right_init_ok:
                self._send_robot_commands(speed_left_final, speed_right_final)
            # else:
            #     # 如果机器人未初始化，确保最终速度为0，避免显示错误信息
            #     speed_left_final = np.zeros(6)
            #     speed_right_final = np.zeros(6)


            # 5. 更新显示
            self.ui_manager.draw_display(speed_left_final, speed_right_final)

            # 6. 控制循环速率
            self.ui_manager.clock.tick(30) # 目标 30 FPS

            # 可选：打印循环时间
            # loop_time = time.time() - start_time
            # print(f"Loop time: {loop_time:.4f} s")

        print("--- 控制循环结束 ---")


    def cleanup(self):
        """停止机器人并清理资源"""
        print("\n正在停止机器人并执行清理操作...")
        stop_speed = [0.0] * 6

        # 停止机器人运动
        try:
            # 尝试发送停止指令到左臂
            if self.left_init_ok and self.controller_left:
                print("  发送停止指令到左臂...")
                try:
                    # 尝试多种停止方法，以防某些方法无效
                    self.controller_left.stop_jog() # 尝试停止 Jog
                    time.sleep(0.05)
                    self.controller_left.stopl() # 尝试停止 Speedl/MoveL
                    time.sleep(0.05)
                    # 发送零速度指令作为最后手段
                    self.controller_left.moveBySpeedl(stop_speed, 100, 10, 0.1) # 使用较大的 acc/arot 确保快速停止
                except AttributeError: pass # 忽略没有 stop_jog/stopl 的情况
                except Exception as e: print(f"  左臂停止指令失败: {e}")
                time.sleep(0.1) # 等待指令生效
                self.controller_left.disconnect()
                print("  左臂已断开。")
            elif self.controller_left:
                 print("  左臂已断开 (无需发送停止指令)。")

            # 尝试发送停止指令到右臂
            if self.right_init_ok and self.controller_right:
                print("  发送停止指令到右臂...")
                try:
                    self.controller_right.stop_jog()
                    time.sleep(0.05)
                    self.controller_right.stopl()
                    time.sleep(0.05)
                    self.controller_right.moveBySpeedl(stop_speed, 100, 10, 0.1)
                except AttributeError: pass
                except Exception as e: print(f"  右臂停止指令失败: {e}")
                time.sleep(0.1)
                self.controller_right.disconnect()
                print("  右臂已断开。")
            elif self.controller_right:
                 print("  右臂已断开 (无需发送停止指令)。")

        except Exception as stop_e:
            print(f"  停止或断开机器人时出错: {stop_e}")
            traceback.print_exc() # 打印详细错误信息

        # 清理 Pygame 资源
        self.ui_manager.quit() # 调用 UI Manager 的退出方法

        print("程序退出。")