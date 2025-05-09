# ui.py
# -*- coding: utf-8 -*-

import pygame
import os
import time
import traceback
import threading
from typing import Optional, Any  # 确保导入 Optional 和 Any
import numpy as np
# 从 config 导入颜色和布局常量及模式常量
from config import (C_WHITE, C_BLACK, C_GREEN, C_RED, C_BLUE, C_YELLOW,
                    C_MAGENTA, C_CYAN, C_GRAY, INFO_X_MARGIN, INFO_Y_START,
                    LINE_SPACING, MODE_XYZ, MODE_RPY, MODE_VISION, MODE_RESET,
                     C_GRAY)  # 安全导入 C_GRAY_ALT
# 从 robot_control 导入格式化函数
from robot_control import format_speed
# 导入视觉交互模块
import vision_interaction
# 导入 config 模块以访问模式常量
import config


class UIManager:
    def __init__(self, controller_instance: Any):  # 使用 Any 来避免循环导入的类型提示问题
        self.controller = controller_instance
        self.screen: Optional[pygame.Surface] = None
        self.info_font: Optional[pygame.font.Font] = None
        self.clock: Optional[pygame.time.Clock] = None
        self.mixer_initialized: bool = False
        self.sound_cache: dict[str, pygame.mixer.Sound] = {}
        self.joystick: Optional[pygame.joystick.Joystick] = None
        self.num_hats: int = 0
        self.num_axes: int = 0
        self.num_buttons: int = 0
        self.status_message: str = ""

        # 用于处理 axis 和 hat 一次性触发的标志位 (更鲁棒的方案)
        # key: (type, index, axis_char, direction) or (type, index) for button
        # value: True if action was triggered and awaiting release/neutral
        self.action_triggered_flags: dict[tuple, bool] = {}

    def init_pygame(self) -> bool:
        """初始化 Pygame 相关模块"""
        print("正在初始化 Pygame...")
        try:
            pygame.init()
            pygame.font.init()

            try:
                pygame.mixer.init()
                self.mixer_initialized = True
                print("  Pygame Mixer 初始化成功.")
                self._load_sounds()
            except pygame.error as e:
                print(f"  错误：初始化 Mixer 失败: {e}")
                self.mixer_initialized = False

            self.screen = pygame.display.set_mode(
                (self.controller.window_width, self.controller.window_height)
            )
            pygame.display.set_caption(f"双臂控制 (模式: {self.controller.control_mode})")

            font_to_load = None
            if self.controller.font_path and os.path.exists(self.controller.font_path):
                font_to_load = self.controller.font_path
                print(f"  尝试加载指定字体: {font_to_load}")
            else:
                if self.controller.font_path:
                    print(f"  警告: 指定字体文件 '{self.controller.font_path}' 未找到.")
                else:
                    print("  未指定字体路径.")

                # 尝试一些常见的系统备用中文字体 (Linux, Windows)
                common_fonts = [
                    "WenQuanYi Zen Hei", "WenQuanYi Micro Hei",  # Linux
                    "SimHei", "Microsoft YaHei", "DengXian",  # Windows
                    "PingFang SC", "Hiragino Sans GB"  # macOS
                ]
                # Pygame's font.match_font() 也可以用来找系统字体，但直接尝试加载更可靠
                # for font_name in common_fonts:
                #     try:
                #         system_font_path = pygame.font.match_font(font_name)
                #         if system_font_path:
                #             print(f"  找到备用系统字体: {font_name} ({system_font_path})")
                #             font_to_load = system_font_path
                #             break
                #     except Exception: # some systems might error on match_font
                #         continue
                # if not font_to_load:
                print("  将使用 Pygame 默认字体 (可能不支持中文)。")

            try:
                self.info_font = pygame.font.Font(font_to_load, self.controller.font_size)
                if font_to_load:
                    print(f"  字体 '{font_to_load}' 加载成功。")
                else:
                    print("  Pygame 默认字体已加载。")
            except Exception as e:
                print(f"  加载字体 '{font_to_load}' 失败: {e}. 将使用最终备用 Pygame 默认字体。")
                self.info_font = pygame.font.Font(None, self.controller.font_size)

            if pygame.joystick.get_count() == 0:
                self.status_message = "错误: 未检测到手柄！"
                print(self.status_message)
                # 对于开发，可以允许无手柄继续，但许多功能将不可用
                # return False # 如果手柄是必需的
            else:
                self.joystick = pygame.joystick.Joystick(0)
                self.joystick.init()
                joystick_name = self.joystick.get_name()
                print(f"  已初始化手柄: {joystick_name}")
                self.num_hats = self.joystick.get_numhats()
                self.num_axes = self.joystick.get_numaxes()
                self.num_buttons = self.joystick.get_numbuttons()
                print(f"    能力: Hats={self.num_hats}, Axes={self.num_axes}, Buttons={self.num_buttons}")
                self.status_message = f"手柄: {joystick_name} 已连接"

            self.clock = pygame.time.Clock()
            print("Pygame 初始化完成.")
            return True
        except Exception as e:
            self.status_message = f"Pygame 初始化失败: {e}"
            print(self.status_message)
            traceback.print_exc()
            return False

    def _load_sounds(self):
        if not self.mixer_initialized: return
        print("\n正在加载音频文件...")
        loaded_count = 0
        for event_name, filepath in self.controller.audio_files_config.items():
            if not filepath or not isinstance(filepath, str):
                print(f"  警告: 音频事件 '{event_name}' 的路径无效或缺失。")
                continue
            try:
                if os.path.exists(filepath):
                    self.sound_cache[event_name] = pygame.mixer.Sound(filepath)
                    loaded_count += 1
                else:
                    print(f"  错误: 音频文件未找到 '{event_name}': {filepath}")
            except pygame.error as e:
                print(f"  错误: 加载 Pygame 声音 '{event_name}' ({filepath}) 时: {e}")
            except Exception as e:
                print(f"  错误: 加载音频 '{event_name}' ({filepath}) 时发生未知错误: {e}")
        print(f"音频文件加载完成 ({loaded_count} 个)。")

    def play_sound(self, event_name: str, loops: int = 0):
        if not self.mixer_initialized: return
        sound = self.sound_cache.get(event_name)
        if sound:
            try:
                sound.play(loops=loops)
            except Exception as e:
                print(f"错误: 播放声音 '{event_name}' 时: {e}")

    def update_status_message(self, message: str):
        self.status_message = message
        # print(f"[UI STATUS] {message}") # 减少控制台输出

    def _check_button_event(self, control_config: Optional[dict], event: pygame.event.Event) -> bool:
        """检查按钮按下事件是否匹配控制配置。"""
        if not control_config: return False
        return control_config.get('type') == 'button' and \
            event.type == pygame.JOYBUTTONDOWN and \
            event.button == control_config.get('index', -1)

    def get_joystick_input_state(self, control_config: Optional[dict],
                                 event: Optional[pygame.event.Event] = None) -> bool:
        """
        检查单个控制配置是否被激活。
        如果提供了 event, 则基于事件触发 (适合一次性动作，需要外部状态管理防重复)。
        如果未提供 event, 则基于当前轮询状态 (适合持续性动作)。
        """
        if not self.joystick or not pygame.joystick.get_init() or not control_config:
            return False

        ctrl_type = control_config.get('type')
        ctrl_index = control_config.get('index', -1)

        # 为一次性动作生成唯一的键
        action_key = None
        if ctrl_type == 'button':
            action_key = ('button', ctrl_index)
        elif ctrl_type == 'axis':
            action_key = ('axis', ctrl_index, control_config.get('direction', 1))
        elif ctrl_type == 'hat':
            action_key = ('hat', ctrl_index, control_config.get('axis', 'x'), control_config.get('direction', 1))

        is_currently_active = False  # 当前帧输入是否激活

        if ctrl_type == 'button':
            if not (0 <= ctrl_index < self.num_buttons): return False
            is_currently_active = self.joystick.get_button(ctrl_index) == 1
        elif ctrl_type == 'axis':
            if not (0 <= ctrl_index < self.num_axes): return False
            axis_val = self.joystick.get_axis(ctrl_index)
            threshold = control_config.get('threshold', self.controller.trigger_threshold)
            direction = control_config.get('direction', 1)
            if (direction == 1 and axis_val > threshold) or \
                    (direction == -1 and axis_val < -threshold):
                is_currently_active = True
        elif ctrl_type == 'hat':
            if not (0 <= ctrl_index < self.num_hats): return False
            hat_val_tuple = self.joystick.get_hat(ctrl_index)  # (x, y)
            hat_axis_cfg = control_config.get('axis', 'x')
            direction = control_config.get('direction', 1)
            if (hat_axis_cfg == 'x' and hat_val_tuple[0] == direction) or \
                    (hat_axis_cfg == 'y' and hat_val_tuple[1] == direction):
                is_currently_active = True

        # 对于事件驱动的一次性动作 (如RPY重置)
        if event:  # 仅在事件发生时评估是否首次触发
            if is_currently_active:
                if not self.action_triggered_flags.get(action_key, False):
                    self.action_triggered_flags[action_key] = True  # 标记为已触发
                    return True  # 首次触发
            else:  # 控制器回到非激活状态
                if action_key in self.action_triggered_flags:
                    self.action_triggered_flags[action_key] = False  # 重置标志
            return False  # 非首次触发或未激活
        else:  # 对于持续性动作，直接返回当前状态
            return is_currently_active

    def handle_events(self):
        current_time = time.time()
        if not pygame.get_init() or not self.controller.running:
            self.controller.running = False;
            return

        for event in pygame.event.get():
            if event.type == pygame.QUIT: self.controller.running = False; return
            if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                self.controller.running = False;
                return

            if not self.joystick: continue

            # --- 通用控制 (主要基于按钮按下/松开) ---
            if event.type == pygame.JOYBUTTONDOWN:
                # 模式切换按钮按下
                if self._check_button_event(self.controller.mode_switch_control, event):
                    if self.controller.mode_button_press_time is None:
                        self.controller.mode_button_press_time = current_time
                # 速度增加
                elif self._check_button_event(self.controller.speed_inc_control, event) and \
                        self.controller.control_mode in [config.MODE_XYZ, config.MODE_RPY]:
                    self.controller.current_speed_xy = min(self.controller.max_speed,
                                                           self.controller.current_speed_xy + self.controller.speed_increment)
                    self.controller.current_speed_z = min(self.controller.max_speed,
                                                          self.controller.current_speed_z + self.controller.speed_increment)
                    self.play_sound('speed_change_confirm')  # 确保此声音在YAML中定义
                # 速度减少
                elif self._check_button_event(self.controller.speed_dec_control, event) and \
                        self.controller.control_mode in [config.MODE_XYZ, config.MODE_RPY]:
                    self.controller.current_speed_xy = max(self.controller.min_speed,
                                                           self.controller.current_speed_xy - self.controller.speed_increment)
                    self.controller.current_speed_z = max(self.controller.min_speed,
                                                          self.controller.current_speed_z - self.controller.speed_increment)
                    self.play_sound('speed_change_confirm')
                # 左夹爪
                elif self._check_button_event(self.controller.gripper_toggle_left_ctrl, event):
                    self.controller.toggle_gripper('left')
                # 右夹爪
                elif self._check_button_event(self.controller.gripper_toggle_right_ctrl, event):
                    self.controller.toggle_gripper('right')

            if event.type == pygame.JOYBUTTONUP:
                # 模式切换按钮松开
                if self.controller.mode_switch_control and \
                        event.button == self.controller.mode_switch_control.get('index', -1):
                    if self.controller.mode_button_press_time is not None:
                        press_duration = current_time - self.controller.mode_button_press_time
                        if press_duration < self.controller.long_press_duration:
                            self.controller.switch_control_mode()
                        self.controller.mode_button_press_time = None

            # --- 模式专属控制 (RPY重置使用 get_joystick_input_state 并传递 event) ---
            if self.controller.control_mode == config.MODE_RESET:
                # 对于RPY重置，我们希望在事件（按钮按下、轴移动到阈值、hat按下）发生时触发一次
                # get_joystick_input_state(config, event) 现在处理这种一次性触发逻辑
                if self.get_joystick_input_state(self.controller.reset_left_arm_default_rpy_ctrl, event):
                    self.controller.attempt_reset_left_arm_default_rpy()
                elif self.get_joystick_input_state(self.controller.reset_left_arm_forward_rpy_ctrl, event):
                    self.controller.attempt_reset_left_arm_forward_rpy()
                elif self.get_joystick_input_state(self.controller.reset_left_arm_backward_rpy_ctrl, event):
                    self.controller.attempt_reset_left_arm_backward_rpy()
                elif self.get_joystick_input_state(self.controller.reset_left_arm_to_left_rpy_ctrl, event):
                    self.controller.attempt_reset_left_arm_to_left_rpy()
                elif self.get_joystick_input_state(self.controller.reset_left_arm_to_right_rpy_ctrl, event):
                    self.controller.attempt_reset_left_arm_to_right_rpy()
                elif self.get_joystick_input_state(self.controller.reset_left_arm_up_rpy_ctrl, event):
                    self.controller.attempt_reset_left_arm_up_rpy()
                elif self.get_joystick_input_state(self.controller.reset_left_arm_down_rpy_ctrl, event):
                    self.controller.attempt_reset_left_arm_down_rpy()

                elif self.get_joystick_input_state(self.controller.reset_right_arm_default_rpy_ctrl, event):
                    self.controller.attempt_reset_right_arm_default_rpy()
                elif self.get_joystick_input_state(self.controller.reset_right_arm_forward_rpy_ctrl, event):
                    self.controller.attempt_reset_right_arm_forward_rpy()
                elif self.get_joystick_input_state(self.controller.reset_right_arm_backward_rpy_ctrl, event):
                    self.controller.attempt_reset_right_arm_backward_rpy()
                elif self.get_joystick_input_state(self.controller.reset_right_arm_to_left_rpy_ctrl, event):
                    self.controller.attempt_reset_right_arm_to_left_rpy()
                elif self.get_joystick_input_state(self.controller.reset_right_arm_to_right_rpy_ctrl, event):
                    self.controller.attempt_reset_right_arm_to_right_rpy()
                elif self.get_joystick_input_state(self.controller.reset_right_arm_up_rpy_ctrl, event):
                    self.controller.attempt_reset_right_arm_up_rpy()
                elif self.get_joystick_input_state(self.controller.reset_right_arm_down_rpy_ctrl, event):
                    self.controller.attempt_reset_right_arm_down_rpy()

                # 原始关节回正 (如果按键不冲突且配置存在) - 通常是按钮类型
                elif self._check_button_event(self.controller.reset_left_arm_ctrl, event):
                    print("[UI Event] 左臂原始关节回正按钮按下...")
                    self.controller.attempt_reset_left_arm()
                elif self._check_button_event(self.controller.reset_right_arm_ctrl, event):
                    print("[UI Event] 右臂原始关节回正按钮按下...")
                    self.controller.attempt_reset_right_arm()

            elif self.controller.control_mode == config.MODE_VISION:
                # 视觉模式按钮通常是一次性按下触发
                if event.type == pygame.JOYBUTTONDOWN:
                    controls_map = self.controller.controls_map
                    start_rec_cfg = controls_map.get('vision_start_record')
                    stop_rec_cfg = controls_map.get('vision_stop_record_confirm')
                    cancel_rec_cfg = controls_map.get('vision_cancel_record')

                    if self._check_button_event(start_rec_cfg, event):
                        if not vision_interaction.is_recording:
                            print("[UI Event] Vision Mode: 'Start Record' button pressed.")
                            self.play_sound('vision_record_start')  # 确保此声音在YAML中定义
                            if vision_interaction.start_recording_thread():
                                self.update_status_message("录音中...按[确认键]结束, [取消键]取消")
                            else:
                                self.update_status_message("启动录音失败"); self.play_sound('action_fail_general')
                        else:
                            print("[UI Event] Vision Mode: Already recording."); self.play_sound(
                                'already_recording_error'); self.update_status_message("已在录音中!")
                    elif self._check_button_event(stop_rec_cfg, event):
                        if vision_interaction.is_recording:
                            print("[UI Event] Vision Mode: 'Stop Record & Confirm' button pressed.")
                            self.play_sound('vision_record_stop')  # 确保此声音在YAML中定义
                            saved_audio_path = vision_interaction.stop_recording_and_save()
                            if saved_audio_path:
                                self.update_status_message("处理语音指令...")
                                if hasattr(self.controller, '_threaded_process_vision_audio'):
                                    thread = threading.Thread(target=self.controller._threaded_process_vision_audio,
                                                              args=(saved_audio_path,))
                                    thread.daemon = True;
                                    thread.start()
                                else:
                                    msg = "错误: 控制器缺少 _threaded_process_vision_audio 方法."; print(
                                        f"[UI Event Error] {msg}"); self.update_status_message(msg); self.play_sound(
                                        'action_fail_general')
                            else:
                                msg = "录音数据处理失败 (无数据/保存失败)"; print(
                                    f"[UI Event] {msg}"); self.update_status_message(msg); self.play_sound(
                                    'action_fail_general')
                        else:
                            print("[UI Event] Vision Mode: Not recording, cannot stop."); self.play_sound(
                                'not_recording_error'); self.update_status_message("未开始录音")
                    elif self._check_button_event(cancel_rec_cfg, event):
                        if vision_interaction.is_recording:
                            print("[UI Event] Vision Mode: 'Cancel Record' button pressed.")
                            if vision_interaction.cancel_recording():
                                self.play_sound('vision_record_cancel'); self.update_status_message(
                                    "录音已取消. 按[开始键]重新开始.")
                            else:
                                self.update_status_message("取消录音失败."); self.play_sound('action_fail_general')
                        else:
                            print("[UI Event] Vision Mode: No recording to cancel."); self.update_status_message(
                                "无录音可取消")

    def draw_display(self, speed_left_final: np.ndarray, speed_right_final: np.ndarray):
        if not self.screen or not self.info_font or not pygame.get_init(): return
        self.screen.fill(C_BLACK)
        lines_to_draw: list[tuple[str, tuple[int, int, int]] | str] = []
        y_pos = INFO_Y_START
        controller = self.controller

        mode_color, mode_name_cn = C_WHITE, "未知模式"
        if controller.control_mode == config.MODE_XYZ:
            mode_color, mode_name_cn = C_WHITE, "XYZ模式"
        elif controller.control_mode == config.MODE_RPY:
            mode_color, mode_name_cn = C_YELLOW, "RPY(点动)模式"
        elif controller.control_mode == config.MODE_VISION:
            mode_color, mode_name_cn = C_MAGENTA, "视觉模式"
        elif controller.control_mode == config.MODE_RESET:
            mode_color, mode_name_cn = C_CYAN, "姿态设置模式"

        lines_to_draw.append((f"当前模式: {mode_name_cn}", mode_color))
        lines_to_draw.append((f"左臂({controller.left_robot_ip}): {'OK' if controller.left_init_ok else 'ERR'} | "
                              f"右臂({controller.right_robot_ip}): {'OK' if controller.right_init_ok else 'ERR'}"))
        lines_to_draw.append((f"左夹爪: {'打开' if controller.left_gripper_open else '关闭'} "
                              f"({'活动' if controller.left_gripper_active else '无效'}) | "
                              f"右夹爪: {'打开' if controller.right_gripper_open else '关闭'} "
                              f"({'活动' if controller.right_gripper_active else '无效'})", C_YELLOW))

        if controller.control_mode in [config.MODE_XYZ, config.MODE_RPY]:
            speed_dec_idx = controller.speed_dec_control.get('index', '?') if controller.speed_dec_control else '?'
            speed_inc_idx = controller.speed_inc_control.get('index', '?') if controller.speed_inc_control else '?'
            lines_to_draw.append((
                                 f"速度 XY: {controller.current_speed_xy:.1f} | Z: {controller.current_speed_z:.1f} | RPY点动: {controller.rpy_speed:.1f} (B{speed_dec_idx}/B{speed_inc_idx} 调速)",
                                 C_BLUE))
            lines_to_draw.append("-")
            lines_to_draw.append("[当前速度指令 (已发送)]")
            lines_to_draw.append(f"  左臂速度: {format_speed(speed_left_final)}")
            lines_to_draw.append(f"  右臂速度: {format_speed(speed_right_final)}")
            lines_to_draw.append("-")

        elif controller.control_mode == config.MODE_RESET:
            lines_to_draw.append(
                (f"RPY姿态设置速度: {controller.reset_rpy_speed:.1f}, 加速度: {controller.reset_rpy_acc:.0f}", C_BLUE))
            lines_to_draw.append("-")
            lines_to_draw.append(("[RPY姿态设置 - 选择姿态]", C_CYAN))

            def fmt_reset_ctrl(cfg: Optional[dict], desc_suffix: str) -> Optional[str]:
                if not cfg: return None
                idx = cfg.get('index', '?')
                ctrl_type = cfg.get('type')

                if ctrl_type == 'button':
                    return f"B{idx} {desc_suffix}"
                elif ctrl_type == 'axis':
                    ax_dir = "+" if cfg.get('direction', 1) == 1 else "-"
                    return f"轴{idx}{ax_dir} {desc_suffix}"
                elif ctrl_type == 'hat':
                    hat_ax = cfg.get('axis', '?').upper()
                    hat_dir_val = cfg.get('direction', 0)
                    hat_dir_str = ""
                    if hat_ax == 'X':
                        hat_dir_str = "右" if hat_dir_val == 1 else ("左" if hat_dir_val == -1 else "?")
                    elif hat_ax == 'Y':
                        hat_dir_str = "下" if hat_dir_val == 1 else ("上" if hat_dir_val == -1 else "?")
                    return f"H{idx}{hat_dir_str} {desc_suffix}"
                return f"未知控件 {desc_suffix}"

            lines_to_draw.append(("--- 左臂 RPY ---", C_WHITE))
            left_ctrls = [
                (controller.reset_left_arm_default_rpy_ctrl, "默认"),
                (controller.reset_left_arm_forward_rpy_ctrl, "向前"),
                (controller.reset_left_arm_backward_rpy_ctrl, "向后"),
                (controller.reset_left_arm_to_left_rpy_ctrl, "向左"),
                (controller.reset_left_arm_to_right_rpy_ctrl, "向右"), (controller.reset_left_arm_up_rpy_ctrl, "向上"),
                (controller.reset_left_arm_down_rpy_ctrl, "向下")]
            valid_left_texts = [fmt_reset_ctrl(cfg, desc) for cfg, desc in left_ctrls if
                                cfg]  # Filter out None configs first
            valid_left_texts = [text for text in valid_left_texts if
                                text]  # Filter out None results from fmt_reset_ctrl

            for i in range(0, len(valid_left_texts), 2): lines_to_draw.append(
                (" | ".join(valid_left_texts[i:i + 2]), C_CYAN))

            lines_to_draw.append(("--- 右臂 RPY ---", C_WHITE))
            right_ctrls = [
                (controller.reset_right_arm_default_rpy_ctrl, "默认"),
                (controller.reset_right_arm_forward_rpy_ctrl, "向前"),
                (controller.reset_right_arm_backward_rpy_ctrl, "向后"),
                (controller.reset_right_arm_to_left_rpy_ctrl, "向左"),
                (controller.reset_right_arm_to_right_rpy_ctrl, "向右"),
                (controller.reset_right_arm_up_rpy_ctrl, "向上"),
                (controller.reset_right_arm_down_rpy_ctrl, "向下")]
            valid_right_texts = [fmt_reset_ctrl(cfg, desc) for cfg, desc in right_ctrls if cfg]
            valid_right_texts = [text for text in valid_right_texts if text]
            for i in range(0, len(valid_right_texts), 2): lines_to_draw.append(
                (" | ".join(valid_right_texts[i:i + 2]), C_CYAN))

            if controller.reset_left_arm_ctrl or controller.reset_right_arm_ctrl:  # Original joint reset
                lines_to_draw.append(("--- 原始关节回正 ---", C_GRAY_ALT))
                if controller.reset_left_arm_ctrl: lines_to_draw.append(
                    (f"  B{controller.reset_left_arm_ctrl.get('index', '?')} 左臂原始回正", C_GRAY_ALT))
                if controller.reset_right_arm_ctrl: lines_to_draw.append(
                    (f"  B{controller.reset_right_arm_ctrl.get('index', '?')} 右臂原始回正", C_GRAY_ALT))
            lines_to_draw.append("-")

        elif controller.control_mode == config.MODE_VISION:
            lines_to_draw.append("-");
            lines_to_draw.append(("[视觉模式活动]", C_MAGENTA))
            vision_status_msg = self.status_message
            if not vision_status_msg or vision_status_msg == f"模式: {config.MODE_VISION}":  # Avoid default mode message
                if vision_interaction.is_recording:
                    vision_status_msg = "录音中...按[确认键]结束, [取消键]取消"
                else:
                    start_rec_btn_cfg = controller.controls_map.get('vision_start_record')
                    start_rec_btn_idx = start_rec_btn_cfg.get('index', '?') if start_rec_btn_cfg else '?'
                    vision_status_msg = f"按 B{start_rec_btn_idx} 开始语音指令"
            lines_to_draw.append((f"  状态: {vision_status_msg}", C_GRAY))
            lines_to_draw.append("-")

        mode_switch_btn_idx = controller.mode_switch_control.get('index',
                                                                 '?') if controller.mode_switch_control else '?'
        gripper_l_btn_idx = controller.gripper_toggle_left_ctrl.get('index',
                                                                    '?') if controller.gripper_toggle_left_ctrl else '?'
        gripper_r_btn_idx = controller.gripper_toggle_right_ctrl.get('index',
                                                                     '?') if controller.gripper_toggle_right_ctrl else '?'
        hints = f"提示: B{mode_switch_btn_idx}(模式) | L夹(B{gripper_l_btn_idx}) | R夹(B{gripper_r_btn_idx})"
        if controller.control_mode in [config.MODE_XYZ, config.MODE_RPY]:
            speed_dec_btn_idx = controller.speed_dec_control.get('index', '?') if controller.speed_dec_control else '?'
            speed_inc_btn_idx = controller.speed_inc_control.get('index', '?') if controller.speed_inc_control else '?'
            hints += f" | 速度(B{speed_dec_btn_idx}/B{speed_inc_btn_idx})"
        lines_to_draw.append((hints, C_GRAY))

        # 渲染所有行
        for item in lines_to_draw:
            line_text, line_color_tuple = (item, C_WHITE) if isinstance(item, str) else item
            if line_text == "-":
                pygame.draw.line(self.screen, C_GRAY, (INFO_X_MARGIN, y_pos + LINE_SPACING // 2 - 1),
                                 (controller.window_width - INFO_X_MARGIN, y_pos + LINE_SPACING // 2 - 1), 1)
                y_pos += LINE_SPACING;
                continue

            final_color = line_color_tuple
            if "ERR" in line_text or "无效" in line_text or "失败" in line_text:
                final_color = C_RED
            elif "OK" in line_text and "ERR" not in line_text:
                final_color = C_GREEN

            try:
                if self.info_font is None: raise ValueError(
                    "Font not initialized")  # Should not happen if init_pygame is successful
                text_surface = self.info_font.render(line_text, True, final_color)
                self.screen.blit(text_surface, (INFO_X_MARGIN, y_pos))
            except Exception as render_e:
                if y_pos == INFO_Y_START: print(f"渲染文本 '{line_text}' (颜色: {final_color}) 时出错: {render_e}")
                try:
                    self.screen.blit(
                        pygame.font.Font(None, self.controller.font_size).render("!RENDER_ERR!", True, C_RED),
                        (INFO_X_MARGIN, y_pos))
                except:
                    pass  # Ultimate fallback if even default font rendering fails
            y_pos += LINE_SPACING
        pygame.display.flip()

    def quit(self):
        print("正在退出 UIManager...")
        if self.mixer_initialized: pygame.mixer.quit(); print("  Pygame Mixer 已退出。")
        if pygame.get_init():
            pygame.quit(); print("  Pygame 已退出。")
        else:
            print("  Pygame 已提前退出或未初始化。")