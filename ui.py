# ui.py
# -*- coding: utf-8 -*-

import pygame
import os
import time
import traceback
import threading

from config import (C_WHITE, C_BLACK, C_GREEN, C_RED, C_BLUE, C_YELLOW,
                    C_MAGENTA, C_CYAN, C_GRAY, INFO_X_MARGIN, INFO_Y_START,
                    LINE_SPACING, MODE_XYZ, MODE_RPY, MODE_VISION, MODE_RESET)
from robot_control import format_speed
import vision_interaction
import config


class UIManager:
    def __init__(self, controller_instance):
        self.controller = controller_instance
        self.screen = None
        self.info_font = None
        self.clock = None
        self.mixer_initialized = False
        self.sound_cache = {}
        self.joystick = None
        self.num_hats = 0
        self.num_axes = 0
        self.num_buttons = 0
        self.status_message = ""

    def init_pygame(self):
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

            if self.controller.font_path and os.path.exists(self.controller.font_path):
                try:
                    self.info_font = pygame.font.Font(self.controller.font_path, self.controller.font_size)
                    print(f"  字体加载成功: {self.controller.font_path}")
                except Exception as e:
                    print(f"  字体加载失败: {e}. 将使用默认字体。")
                    self.info_font = pygame.font.Font(None, self.controller.font_size)
            else:
                if self.controller.font_path:
                    print(f"  警告: 字体文件 '{self.controller.font_path}' 未找到. 将使用默认字体。")
                else:
                    print("  未指定字体路径，将使用默认字体。")
                self.info_font = pygame.font.Font(None, self.controller.font_size)

            if pygame.joystick.get_count() == 0:
                self.status_message = "错误: 未检测到手柄！"
                print(self.status_message)
                return False

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
                print(f"  错误: 加载 Pygame 声音 '{event_name}' ({filepath}) 时: {e}")  # Corrected typo
            except Exception as e:
                print(f"  错误: 加载音频 '{event_name}' ({filepath}) 时发生未知错误: {e}")
        print(f"音频文件加载完成 ({loaded_count} 个)。")

    def play_sound(self, event_name, loops=0):
        if not self.mixer_initialized: return
        sound = self.sound_cache.get(event_name)
        if sound:
            try:
                sound.play(loops=loops)
            except Exception as e:
                print(f"错误: 播放声音 '{event_name}' 时: {e}")

    def update_status_message(self, message):
        self.status_message = message
        # print(f"[UI STATUS] {message}") # Optionally reduce console spam for status

    def _check_button(self, control_config, button_index_pressed):
        if control_config and control_config.get('type') == 'button' and \
                button_index_pressed == control_config.get('index', -1):
            return True
        return False

    def handle_events(self):
        current_time = time.time()
        if not pygame.get_init() or not self.controller.running:
            self.controller.running = False
            return

        for event in pygame.event.get():
            if event.type == pygame.QUIT: self.controller.running = False; return
            if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                self.controller.running = False;
                return

            if not self.joystick: continue

            if event.type == pygame.JOYBUTTONDOWN:
                button_index = event.button
                mode_switch_cfg = self.controller.mode_switch_control
                speed_inc_cfg = self.controller.speed_inc_control
                speed_dec_cfg = self.controller.speed_dec_control
                left_gripper_cfg = self.controller.gripper_toggle_left_ctrl
                right_gripper_cfg = self.controller.gripper_toggle_right_ctrl

                if self._check_button(mode_switch_cfg, button_index):
                    if self.controller.mode_button_press_time is None:
                        self.controller.mode_button_press_time = current_time
                elif self._check_button(speed_inc_cfg, button_index) and \
                        self.controller.control_mode in [config.MODE_XYZ, config.MODE_RPY]:
                    self.controller.current_speed_xy = min(self.controller.max_speed,
                                                           self.controller.current_speed_xy + self.controller.speed_increment)
                    self.controller.current_speed_z = min(self.controller.max_speed,
                                                          self.controller.current_speed_z + self.controller.speed_increment)
                    self.play_sound('speed_change_confirm')
                elif self._check_button(speed_dec_cfg, button_index) and \
                        self.controller.control_mode in [config.MODE_XYZ, config.MODE_RPY]:
                    self.controller.current_speed_xy = max(self.controller.min_speed,
                                                           self.controller.current_speed_xy - self.controller.speed_increment)
                    self.controller.current_speed_z = max(self.controller.min_speed,
                                                          self.controller.current_speed_z - self.controller.speed_increment)
                    self.play_sound('speed_change_confirm')
                elif self._check_button(left_gripper_cfg, button_index):
                    self.controller.toggle_gripper('left')
                elif self._check_button(right_gripper_cfg, button_index):
                    self.controller.toggle_gripper('right')

                # --- RPY Reset Mode Controls ---
                elif self.controller.control_mode == config.MODE_RESET:
                    # Left Arm RPY Resets
                    if self._check_button(self.controller.reset_left_arm_default_rpy_ctrl, button_index):
                        self.controller.attempt_reset_left_arm_default_rpy()
                    elif self._check_button(self.controller.reset_left_arm_forward_rpy_ctrl, button_index):
                        self.controller.attempt_reset_left_arm_forward_rpy()
                    elif self._check_button(self.controller.reset_left_arm_backward_rpy_ctrl, button_index):
                        self.controller.attempt_reset_left_arm_backward_rpy()
                    elif self._check_button(self.controller.reset_left_arm_to_left_rpy_ctrl, button_index):
                        self.controller.attempt_reset_left_arm_to_left_rpy()
                    elif self._check_button(self.controller.reset_left_arm_to_right_rpy_ctrl, button_index):
                        self.controller.attempt_reset_left_arm_to_right_rpy()
                    elif self._check_button(self.controller.reset_left_arm_up_rpy_ctrl, button_index):
                        self.controller.attempt_reset_left_arm_up_rpy()
                    elif self._check_button(self.controller.reset_left_arm_down_rpy_ctrl, button_index):
                        self.controller.attempt_reset_left_arm_down_rpy()
                    # Right Arm RPY Resets
                    elif self._check_button(self.controller.reset_right_arm_default_rpy_ctrl, button_index):
                        self.controller.attempt_reset_right_arm_default_rpy()
                    elif self._check_button(self.controller.reset_right_arm_forward_rpy_ctrl, button_index):
                        self.controller.attempt_reset_right_arm_forward_rpy()
                    elif self._check_button(self.controller.reset_right_arm_backward_rpy_ctrl, button_index):
                        self.controller.attempt_reset_right_arm_backward_rpy()
                    elif self._check_button(self.controller.reset_right_arm_to_left_rpy_ctrl, button_index):
                        self.controller.attempt_reset_right_arm_to_left_rpy()
                    elif self._check_button(self.controller.reset_right_arm_to_right_rpy_ctrl, button_index):
                        self.controller.attempt_reset_right_arm_to_right_rpy()
                    elif self._check_button(self.controller.reset_right_arm_up_rpy_ctrl, button_index):
                        self.controller.attempt_reset_right_arm_up_rpy()
                    elif self._check_button(self.controller.reset_right_arm_down_rpy_ctrl, button_index):
                        self.controller.attempt_reset_right_arm_down_rpy()
                    # Fallback to original joint reset if specific RPY buttons aren't matched
                    # And if those original controls are still defined
                    elif self._check_button(self.controller.reset_left_arm_ctrl, button_index):
                        print("[UI Event] 左臂原始回正按钮按下...")
                        self.controller.attempt_reset_left_arm()
                    elif self._check_button(self.controller.reset_right_arm_ctrl, button_index):
                        print("[UI Event] 右臂原始回正按钮按下...")
                        self.controller.attempt_reset_right_arm()


                elif self.controller.control_mode == config.MODE_VISION:
                    controls_map = self.controller.controls_map
                    start_rec_cfg = controls_map.get('vision_start_record')
                    stop_rec_cfg = controls_map.get('vision_stop_record_confirm')
                    cancel_rec_cfg = controls_map.get('vision_cancel_record')

                    if self._check_button(start_rec_cfg, button_index):
                        if not vision_interaction.is_recording:
                            print("[UI Event] Vision Mode: 'Start Record' button pressed.")
                            self.play_sound('vision_record_start')
                            if vision_interaction.start_recording_thread():
                                self.update_status_message("录音中...按[确认键]结束, [取消键]取消")
                            else:
                                self.update_status_message("启动录音失败"); self.play_sound('action_fail_general')
                        else:
                            print("[UI Event] Vision Mode: Already recording."); self.play_sound(
                                'already_recording_error'); self.update_status_message("已在录音中!")
                    elif self._check_button(stop_rec_cfg, button_index):
                        if vision_interaction.is_recording:
                            print("[UI Event] Vision Mode: 'Stop Record & Confirm' button pressed.")
                            self.play_sound('vision_record_stop')
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
                    elif self._check_button(cancel_rec_cfg, button_index):
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

            if event.type == pygame.JOYBUTTONUP:
                button_index = event.button
                mode_switch_cfg = self.controller.mode_switch_control
                if self._check_button(mode_switch_cfg, button_index):
                    if self.controller.mode_button_press_time is not None:
                        press_duration = current_time - self.controller.mode_button_press_time
                        if press_duration < self.controller.long_press_duration:
                            self.controller.switch_control_mode()
                        self.controller.mode_button_press_time = None

    def get_joystick_input_state(self, control_config):
        if not self.joystick or not pygame.joystick.get_init() or not control_config: return False
        active, ctrl_type, ctrl_index = False, control_config.get('type'), control_config.get('index', -1)
        if ctrl_type == 'button':
            if not (0 <= ctrl_index < self.num_buttons): return False
            active = self.joystick.get_button(ctrl_index) == 1
        elif ctrl_type == 'axis':
            if not (0 <= ctrl_index < self.num_axes): return False
            axis_val = self.joystick.get_axis(ctrl_index)
            threshold = control_config.get('threshold', self.controller.trigger_threshold if hasattr(self.controller,
                                                                                                     'trigger_threshold') else 0.1)
            direction = control_config.get('direction', 1)
            if (direction == 1 and axis_val > threshold) or \
                    (direction == -1 and axis_val < -threshold): active = True
        elif ctrl_type == 'hat':
            if not (0 <= ctrl_index < self.num_hats): return False
            hat_val, hat_axis_cfg, direction = self.joystick.get_hat(ctrl_index), control_config.get('axis',
                                                                                                     'x'), control_config.get(
                'direction', 1)
            if (hat_axis_cfg == 'x' and hat_val[0] == direction) or \
                    (hat_axis_cfg == 'y' and hat_val[1] == direction): active = True
        return active

    def draw_display(self, speed_left_final, speed_right_final):
        if not self.screen or not self.info_font or not pygame.get_init(): return
        self.screen.fill(C_BLACK)
        lines_to_draw, y_pos, controller = [], INFO_Y_START, self.controller
        fmt_speed_l, fmt_speed_r = format_speed(speed_left_final), format_speed(speed_right_final)
        mode_color, mode_name_cn = C_WHITE, "未知模式"
        if controller.control_mode == config.MODE_XYZ:
            mode_color, mode_name_cn = C_WHITE, "XYZ模式"
        elif controller.control_mode == config.MODE_RPY:
            mode_color, mode_name_cn = C_YELLOW, "RPY(点动)模式"  # Clarified RPY jog
        elif controller.control_mode == config.MODE_VISION:
            mode_color, mode_name_cn = C_MAGENTA, "视觉模式"
        elif controller.control_mode == config.MODE_RESET:
            mode_color, mode_name_cn = C_CYAN, "姿态设置模式"  # Renamed

        lines_to_draw.append((f"当前模式: {mode_name_cn}", mode_color))
        lines_to_draw.append((f"左臂({controller.left_robot_ip}): {'OK' if controller.left_init_ok else 'ERR'} | "
                              f"右臂({controller.right_robot_ip}): {'OK' if controller.right_init_ok else 'ERR'}"))
        lines_to_draw.append((f"左夹爪: {'打开' if controller.left_gripper_open else '关闭'} "
                              f"({'活动' if controller.left_gripper_active else '无效'}) | "
                              f"右夹爪: {'打开' if controller.right_gripper_open else '关闭'} "
                              f"({'活动' if controller.right_gripper_active else '无效'})", C_YELLOW))

        if controller.control_mode in [config.MODE_XYZ, config.MODE_RPY]:  # Jog modes
            speed_dec_idx = controller.speed_dec_control.get('index', '?') if controller.speed_dec_control else '?'
            speed_inc_idx = controller.speed_inc_control.get('index', '?') if controller.speed_inc_control else '?'
            lines_to_draw.append((
                                 f"速度 XY: {controller.current_speed_xy:.1f} | Z: {controller.current_speed_z:.1f} | RPY: {controller.rpy_speed:.1f} (B{speed_dec_idx}/B{speed_inc_idx} 调速)",
                                 C_BLUE))
            lines_to_draw.append(("-"));
            lines_to_draw.append(("[当前速度指令 (已发送)]"))
            lines_to_draw.append((f"  左臂速度: {fmt_speed_l}"));
            lines_to_draw.append((f"  右臂速度: {fmt_speed_r}"))
            lines_to_draw.append(("-"))

        elif controller.control_mode == config.MODE_RESET:  # RPY Preset Pose Mode
            lines_to_draw.append(
                (f"RPY姿态设置速度: {controller.reset_rpy_speed:.1f}, 加速度: {controller.reset_rpy_acc:.0f}", C_BLUE))
            lines_to_draw.append(("-"));
            lines_to_draw.append(("[RPY姿态设置 - 选择姿态]", C_CYAN))

            def fmt_reset_btn(cfg, desc_suffix):
                idx = cfg.get('index', '?') if cfg else '?'
                return f"B{idx} {desc_suffix}" if idx != '?' else None

            lines_to_draw.append(("--- 左臂 RPY ---", C_WHITE))
            btn_texts_left = [
                fmt_reset_btn(controller.reset_left_arm_default_rpy_ctrl, "默认"),
                fmt_reset_btn(controller.reset_left_arm_forward_rpy_ctrl, "向前"),
                fmt_reset_btn(controller.reset_left_arm_backward_rpy_ctrl, "向后"),
                fmt_reset_btn(controller.reset_left_arm_to_left_rpy_ctrl, "向左"),
                fmt_reset_btn(controller.reset_left_arm_to_right_rpy_ctrl, "向右"),
                fmt_reset_btn(controller.reset_left_arm_up_rpy_ctrl, "向上"),
                fmt_reset_btn(controller.reset_left_arm_down_rpy_ctrl, "向下")]
            valid_btn_texts_left = [t for t in btn_texts_left if t]
            for i in range(0, len(valid_btn_texts_left), 2): lines_to_draw.append(
                (" | ".join(valid_btn_texts_left[i:i + 2]), C_CYAN))

            lines_to_draw.append(("--- 右臂 RPY ---", C_WHITE))
            btn_texts_right = [
                fmt_reset_btn(controller.reset_right_arm_default_rpy_ctrl, "默认"),
                fmt_reset_btn(controller.reset_right_arm_forward_rpy_ctrl, "向前"),
                fmt_reset_btn(controller.reset_right_arm_backward_rpy_ctrl, "向后"),
                fmt_reset_btn(controller.reset_right_arm_to_left_rpy_ctrl, "向左"),
                fmt_reset_btn(controller.reset_right_arm_to_right_rpy_ctrl, "向右"),
                fmt_reset_btn(controller.reset_right_arm_up_rpy_ctrl, "向上"),
                fmt_reset_btn(controller.reset_right_arm_down_rpy_ctrl, "向下")]
            valid_btn_texts_right = [t for t in btn_texts_right if t]
            for i in range(0, len(valid_btn_texts_right), 2): lines_to_draw.append(
                (" | ".join(valid_btn_texts_right[i:i + 2]), C_CYAN))

            # Optionally display original joint reset buttons if still used
            if controller.reset_left_arm_ctrl or controller.reset_right_arm_ctrl:
                lines_to_draw.append(("--- 原始关节回正 ---", C_GRAY_ALT if hasattr(config,
                                                                                    'C_GRAY_ALT') else C_GRAY))  # Use a different gray if defined
                left_orig_idx = controller.reset_left_arm_ctrl.get('index',
                                                                   '?') if controller.reset_left_arm_ctrl else '?'
                right_orig_idx = controller.reset_right_arm_ctrl.get('index',
                                                                     '?') if controller.reset_right_arm_ctrl else '?'
                if left_orig_idx != '?': lines_to_draw.append(
                    (f"  B{left_orig_idx} 左臂原始回正", C_GRAY_ALT if hasattr(config, 'C_GRAY_ALT') else C_GRAY))
                if right_orig_idx != '?': lines_to_draw.append(
                    (f"  B{right_orig_idx} 右臂原始回正", C_GRAY_ALT if hasattr(config, 'C_GRAY_ALT') else C_GRAY))
            lines_to_draw.append(("-"))

        elif controller.control_mode == config.MODE_VISION:
            lines_to_draw.append(("-"));
            lines_to_draw.append(("[视觉模式活动]", C_MAGENTA))
            vision_status_msg = self.status_message
            if not vision_status_msg:
                if vision_interaction.is_recording:
                    vision_status_msg = "录音中...按[确认键]结束, [取消键]取消"
                else:
                    controls_map = controller.controls_map
                    start_rec_btn_cfg = controls_map.get('vision_start_record')
                    start_rec_btn_idx = start_rec_btn_cfg.get('index', '?') if start_rec_btn_cfg else '?'
                    vision_status_msg = f"按 B{start_rec_btn_idx} 开始语音指令"
            lines_to_draw.append((f"  状态: {vision_status_msg}", C_GRAY))
            lines_to_draw.append(("-"))

        mode_switch_btn_idx = controller.mode_switch_control.get('index',
                                                                 '?') if controller.mode_switch_control else '?'
        gripper_l_btn_idx = controller.gripper_toggle_left_ctrl.get('index',
                                                                    '?') if controller.gripper_toggle_left_ctrl else '?'
        gripper_r_btn_idx = controller.gripper_toggle_right_ctrl.get('index',
                                                                     '?') if controller.gripper_toggle_right_ctrl else '?'
        hints = f"提示: B{mode_switch_btn_idx}(模式切换) | L夹爪(B{gripper_l_btn_idx}) | R夹爪(B{gripper_r_btn_idx})"
        if controller.control_mode in [config.MODE_XYZ, config.MODE_RPY]:
            speed_dec_btn_idx = controller.speed_dec_control.get('index', '?') if controller.speed_dec_control else '?'
            speed_inc_btn_idx = controller.speed_inc_control.get('index', '?') if controller.speed_inc_control else '?'
            hints += f" | 速度(B{speed_dec_btn_idx}/B{speed_inc_btn_idx})"
        lines_to_draw.append((hints, C_GRAY))

        for item in lines_to_draw:
            line_text, line_color = (item, C_WHITE) if isinstance(item, str) else item
            if line_text == "-":
                pygame.draw.line(self.screen, C_GRAY, (INFO_X_MARGIN, y_pos + LINE_SPACING // 2 - 1),
                                 (controller.window_width - INFO_X_MARGIN, y_pos + LINE_SPACING // 2 - 1), 1)
                y_pos += LINE_SPACING;
                continue
            final_color = line_color
            if "ERR" in line_text or "无效" in line_text or "失败" in line_text:
                final_color = C_RED
            elif "OK" in line_text and "ERR" not in line_text:
                final_color = C_GREEN
            try:
                text_surface = self.info_font.render(line_text, True, final_color)
                self.screen.blit(text_surface, (INFO_X_MARGIN, y_pos))
            except Exception as render_e:
                if y_pos == INFO_Y_START: print(f"渲染文本 '{line_text}' 时出错: {render_e}")
                try:
                    self.screen.blit(
                        pygame.font.Font(None, self.controller.font_size).render("!RENDER_ERR!", True, C_RED),
                        (INFO_X_MARGIN, y_pos))
                except:
                    pass
            y_pos += LINE_SPACING
        pygame.display.flip()

    def quit(self):
        print("正在退出 UIManager...")
        if self.mixer_initialized: pygame.mixer.quit(); print("  Pygame Mixer 已退出。")
        if pygame.get_init():
            pygame.quit(); print("  Pygame 已退出。")
        else:
            print("  Pygame 已提前退出或未初始化。")