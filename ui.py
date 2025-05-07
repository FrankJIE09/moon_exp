# ui.py
# -*- coding: utf-8 -*-

import pygame
import os
import time
import traceback
import threading  # 确保导入 threading

# 从 config 导入颜色和布局常量及模式常量
from config import (C_WHITE, C_BLACK, C_GREEN, C_RED, C_BLUE, C_YELLOW,
                    C_MAGENTA, C_CYAN, C_GRAY, INFO_X_MARGIN, INFO_Y_START,
                    LINE_SPACING, MODE_XYZ, MODE_RPY, MODE_VISION, MODE_RESET)
# 从 robot_control 导入格式化函数
from robot_control import format_speed
# 导入视觉交互模块 (假设它与 ui.py 在同一级或Python路径可达)
import vision_interaction
# 导入 config 模块以访问模式常量 (虽然已从from config import...导入，但明确一下)
import config


class UIManager:
    def __init__(self, controller_instance):
        self.controller = controller_instance  # 引用主控制器实例以访问其状态
        self.screen = None
        self.info_font = None
        self.clock = None
        self.mixer_initialized = False
        self.sound_cache = {}
        self.joystick = None
        self.num_hats = 0
        self.num_axes = 0
        self.num_buttons = 0
        # 可选: 用于在屏幕上显示状态信息的变量
        self.status_message = ""  # 由 controller 或 UIManager 更新

    def init_pygame(self):
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
            # 初始标题，模式切换时会更新
            pygame.display.set_caption(f"双臂控制 (模式: {self.controller.control_mode})")

            if self.controller.font_path and os.path.exists(self.controller.font_path):
                try:
                    self.info_font = pygame.font.Font(self.controller.font_path, self.controller.font_size)
                    print(f"  字体加载成功: {self.controller.font_path}")
                except Exception as e:
                    print(f"  字体加载失败: {e}. 将使用默认字体。")
                    self.info_font = pygame.font.Font(None, self.controller.font_size)  # 使用 Pygame 默认字体
            else:
                if self.controller.font_path:  # 如果指定了路径但文件不存在
                    print(f"  警告: 字体文件 '{self.controller.font_path}' 未找到. 将使用默认字体。")
                else:  # 如果路径未指定
                    print("  未指定字体路径，将使用默认字体。")
                self.info_font = pygame.font.Font(None, self.controller.font_size)

            joystick_count = pygame.joystick.get_count()
            if joystick_count == 0:
                self.status_message = "错误: 未检测到手柄！"
                print(self.status_message)
                # 这里可以选择是否抛出异常或允许程序在无手柄模式下运行（如果支持）
                # raise RuntimeError("未检测到手柄！")
                return False  # 或者返回 False 表示初始化未完全成功

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
        """加载音频文件"""
        if not self.mixer_initialized:
            # print("Mixer 未初始化，无法加载声音。") # 已经被 init_pygame 打印过了
            return
        print("\n正在加载音频文件...")
        loaded_count = 0
        # 使用主控制器中的 audio_files_config
        for event_name, filepath in self.controller.audio_files_config.items():  # 假设 controller 有 audio_files_config
            if not filepath or not isinstance(filepath, str):
                print(f"  警告: 音频事件 '{event_name}' 的路径无效或缺失。")
                continue
            try:
                if os.path.exists(filepath):
                    sound = pygame.mixer.Sound(filepath)
                    self.sound_cache[event_name] = sound
                    loaded_count += 1
                else:
                    print(f"  错误: 音频文件未找到 '{event_name}': {filepath}")
            except pygame.error as e:  # 更具体的 Pygame 声音加载错误
                print(f"  错误: 加载 Pygame 声音 '{event_name}' ({filepath}) 时: {聲e}")
            except Exception as e:
                print(f"  错误: 加载音频 '{event_name}' ({filepath}) 时发生未知错误: {e}")
        print(f"音频文件加载完成 ({loaded_count} 个)。")

    def play_sound(self, event_name, loops=0):  # 添加 loops 参数
        """播放音频"""
        if not self.mixer_initialized:
            return
        sound = self.sound_cache.get(event_name)
        if sound:
            try:
                sound.play(loops=loops)
            except Exception as e:
                print(f"错误: 播放声音 '{event_name}' 时: {e}")
        # else:
        # print(f"警告: 音频事件 '{event_name}' 未在 sound_cache 中找到。")

    def update_status_message(self, message):
        """更新要在屏幕上显示的状态信息"""
        self.status_message = message
        print(f"[UI STATUS] {message}")

    def handle_events(self):
        """处理 Pygame 事件（退出、按钮按下/松开），并更新控制器状态"""
        current_time = time.time()
        if not pygame.get_init() or not self.controller.running:
            self.controller.running = False  # 确保控制器知道已停止
            return

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.controller.running = False
                return
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self.controller.running = False
                    return

            if not self.joystick:  # 如果没有手柄，不处理手柄事件
                continue

            # 手柄按钮按下
            if event.type == pygame.JOYBUTTONDOWN:
                button_index = event.button

                # --- 通用控制 ---
                mode_switch_cfg = self.controller.mode_switch_control
                speed_inc_cfg = self.controller.speed_inc_control
                speed_dec_cfg = self.controller.speed_dec_control
                left_gripper_cfg = self.controller.gripper_toggle_left_ctrl
                right_gripper_cfg = self.controller.gripper_toggle_right_ctrl

                # 模式切换按钮按下
                if mode_switch_cfg and mode_switch_cfg.get('type') == 'button' and \
                        button_index == mode_switch_cfg.get('index', -1):
                    if self.controller.mode_button_press_time is None:  # 避免重复计时
                        self.controller.mode_button_press_time = current_time

                # 速度增加 (仅在特定模式下)
                elif speed_inc_cfg and speed_inc_cfg.get('type') == 'button' and \
                        button_index == speed_inc_cfg.get('index', -1) and \
                        self.controller.control_mode in [config.MODE_XYZ, config.MODE_RPY]:
                    self.controller.current_speed_xy = min(self.controller.max_speed,
                                                           self.controller.current_speed_xy + self.controller.speed_increment)
                    self.controller.current_speed_z = min(self.controller.max_speed,
                                                          self.controller.current_speed_z + self.controller.speed_increment)
                    self.play_sound('speed_change_confirm')  # 可选: 调速确认音

                # 速度减少 (仅在特定模式下)
                elif speed_dec_cfg and speed_dec_cfg.get('type') == 'button' and \
                        button_index == speed_dec_cfg.get('index', -1) and \
                        self.controller.control_mode in [config.MODE_XYZ, config.MODE_RPY]:
                    self.controller.current_speed_xy = max(self.controller.min_speed,
                                                           self.controller.current_speed_xy - self.controller.speed_increment)
                    self.controller.current_speed_z = max(self.controller.min_speed,
                                                          self.controller.current_speed_z - self.controller.speed_increment)
                    self.play_sound('speed_change_confirm')  # 可选

                # 左夹爪切换
                elif left_gripper_cfg and left_gripper_cfg.get('type') == 'button' and \
                        button_index == left_gripper_cfg.get('index', -1):
                    self.controller.toggle_gripper('left')  # toggle_gripper 内部会播放声音

                # 右夹爪切换
                elif right_gripper_cfg and right_gripper_cfg.get('type') == 'button' and \
                        button_index == right_gripper_cfg.get('index', -1):
                    self.controller.toggle_gripper('right')  # toggle_gripper 内部会播放声音

                # --- 模式专属控制 ---
                current_control_mode = self.controller.control_mode

                if current_control_mode == config.MODE_RESET:
                    left_reset_cfg = self.controller.reset_left_arm_ctrl
                    right_reset_cfg = self.controller.reset_right_arm_ctrl
                    # 确保配置存在且类型正确
                    if left_reset_cfg and left_reset_cfg.get('type') == 'button' and \
                            button_index == left_reset_cfg.get('index', -1):  # 使用 -1 作为无效索引的默认值
                        print("[UI Event] 左臂回正按钮按下...")
                        self.update_status_message("左臂回正中...")
                        self.controller.attempt_reset_left_arm()  # 此方法应处理声音反馈
                    elif right_reset_cfg and right_reset_cfg.get('type') == 'button' and \
                            button_index == right_reset_cfg.get('index', -1):
                        print("[UI Event] 右臂回正按钮按下...")
                        self.update_status_message("右臂回正中...")
                        self.controller.attempt_reset_right_arm()  # 此方法应处理声音反馈

                elif current_control_mode == config.MODE_VISION:
                    # 从控制器加载的 controls_map 中获取视觉模式按键配置
                    # 确保 self.controller.controls_map 在 DualArmController 初始化时已从配置加载
                    controls_map = self.controller.controls_map if hasattr(self.controller, 'controls_map') else {}

                    start_rec_cfg = controls_map.get('vision_start_record')
                    stop_rec_cfg = controls_map.get('vision_stop_record_confirm')
                    cancel_rec_cfg = controls_map.get('vision_cancel_record')

                    if start_rec_cfg and start_rec_cfg.get('type') == 'button' and \
                            button_index == start_rec_cfg.get('index', -1):
                        if not vision_interaction.is_recording:
                            print("[UI Event] Vision Mode: 'Start Record' button pressed.")
                            self.play_sound('vision_record_start')
                            if vision_interaction.start_recording_thread():
                                self.update_status_message("录音中...按[确认键]结束, [取消键]取消")
                            else:
                                self.update_status_message("启动录音失败")
                                self.play_sound('action_fail_general')  # 通用失败音
                        else:
                            print("[UI Event] Vision Mode: Already recording.")
                            self.play_sound('already_recording_error')
                            self.update_status_message("已在录音中!")

                    elif stop_rec_cfg and stop_rec_cfg.get('type') == 'button' and \
                            button_index == stop_rec_cfg.get('index', -1):
                        if vision_interaction.is_recording:
                            print("[UI Event] Vision Mode: 'Stop Record & Confirm' button pressed.")
                            self.play_sound('vision_record_stop')
                            # stop_recording_and_save 现在应该只停止和保存，不处理发送
                            saved_audio_path = vision_interaction.stop_recording_and_save()
                            if saved_audio_path:
                                self.update_status_message("处理语音指令...")
                                # 调用 DualArmController 中的方法在线程中处理网络和后续逻辑
                                if hasattr(self.controller, '_threaded_process_vision_audio'):
                                    # 将任务交给后台线程
                                    thread = threading.Thread(
                                        target=self.controller._threaded_process_vision_audio,
                                        args=(saved_audio_path,)
                                    )
                                    thread.daemon = True
                                    thread.start()
                                else:
                                    msg = "错误: 控制器缺少 _threaded_process_vision_audio 方法。"
                                    print(f"[UI Event Error] {msg}")
                                    self.update_status_message(msg)
                                    self.play_sound('action_fail_general')
                            else:  # stop_recording_and_save 返回 None，可能保存失败或无数据
                                msg = "录音数据处理失败 (无数据/保存失败)"
                                print(f"[UI Event] {msg}")
                                self.update_status_message(msg)
                                self.play_sound('action_fail_general')
                        else:  # 不在录音中，却按了停止键
                            print("[UI Event] Vision Mode: Not recording, cannot stop.")
                            self.play_sound('not_recording_error')
                            self.update_status_message("未开始录音")

                    elif cancel_rec_cfg and cancel_rec_cfg.get('type') == 'button' and \
                            button_index == cancel_rec_cfg.get('index', -1):
                        if vision_interaction.is_recording:
                            print("[UI Event] Vision Mode: 'Cancel Record' button pressed.")
                            if vision_interaction.cancel_recording():
                                self.play_sound('vision_record_cancel')
                                self.update_status_message("录音已取消. 按[开始键]重新开始.")
                            else:  # cancel_recording 理论上应该总能成功如果 is_recording 是 true
                                self.update_status_message("取消录音失败.")
                                self.play_sound('action_fail_general')
                        else:  # 不在录音中，却按了取消键
                            print("[UI Event] Vision Mode: No recording to cancel.")
                            self.update_status_message("无录音可取消")

            # 手柄按钮松开
            if event.type == pygame.JOYBUTTONUP:
                button_index = event.button
                mode_switch_cfg = self.controller.mode_switch_control  # 重新获取，以防万一

                # 模式切换按钮松开 (短按切换模式)
                if mode_switch_cfg and mode_switch_cfg.get('type') == 'button' and \
                        button_index == mode_switch_cfg.get('index', -1):
                    if self.controller.mode_button_press_time is not None:
                        press_duration = current_time - self.controller.mode_button_press_time
                        if press_duration < self.controller.long_press_duration:
                            self.controller.switch_control_mode()  # switch_control_mode 应该负责播放声音和更新标题
                        # 重置计时器，无论长按短按
                        self.controller.mode_button_press_time = None

        # 在事件循环外，可以根据 self.status_message 更新屏幕上的文本显示
        # draw_display 方法会用到 self.status_message

    def get_joystick_input_state(self, control_config):
        """检查单个控制配置是否被激活"""
        if not self.joystick or not pygame.joystick.get_init() or not control_config:  # 增加检查 joystick 是否 init
            return False

        active = False
        ctrl_type = control_config.get('type')
        ctrl_index = control_config.get('index', -1)

        if ctrl_type == 'button':
            if not (0 <= ctrl_index < self.num_buttons): return False
            active = self.joystick.get_button(ctrl_index) == 1
        elif ctrl_type == 'axis':
            if not (0 <= ctrl_index < self.num_axes): return False
            axis_val = self.joystick.get_axis(ctrl_index)
            # 使用 controller 的 trigger_threshold，因为它可能从配置加载
            threshold = control_config.get('threshold', self.controller.trigger_threshold if hasattr(self.controller,
                                                                                                     'trigger_threshold') else 0.1)
            direction = control_config.get('direction', 1)
            if direction == 1 and axis_val > threshold:
                active = True
            elif direction == -1 and axis_val < -threshold:
                active = True
        elif ctrl_type == 'hat':
            if not (0 <= ctrl_index < self.num_hats): return False
            hat_val = self.joystick.get_hat(ctrl_index)
            hat_axis_cfg = control_config.get('axis', 'x')
            direction = control_config.get('direction', 1)
            if hat_axis_cfg == 'x' and hat_val[0] == direction:
                active = True
            elif hat_axis_cfg == 'y' and hat_val[1] == direction:
                active = True
        return active

    def draw_display(self, speed_left_final, speed_right_final):
        """绘制 Pygame 显示窗口"""
        if not self.screen or not self.info_font or not pygame.get_init():
            return

        self.screen.fill(C_BLACK)
        lines_to_draw = []
        y_pos = INFO_Y_START

        controller = self.controller  # 简化引用
        fmt_speed_l = format_speed(speed_left_final)
        fmt_speed_r = format_speed(speed_right_final)

        mode_color = C_WHITE
        mode_name_cn = "未知模式"
        # ... (模式名称和颜色设置，与之前相同) ...
        if controller.control_mode == config.MODE_XYZ:
            mode_color, mode_name_cn = C_WHITE, "XYZ模式"
        elif controller.control_mode == config.MODE_RPY:
            mode_color, mode_name_cn = C_YELLOW, "RPY模式"
        elif controller.control_mode == config.MODE_VISION:
            mode_color, mode_name_cn = C_MAGENTA, "视觉模式"
        elif controller.control_mode == config.MODE_RESET:
            mode_color, mode_name_cn = C_CYAN, "回正模式"

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
            lines_to_draw.append((f"速度 XY: {controller.current_speed_xy:.1f} | "
                                  f"Z: {controller.current_speed_z:.1f} | "
                                  f"RPY: {controller.rpy_speed:.1f} "
                                  f"(B{speed_dec_idx}/B{speed_inc_idx} 调速)", C_BLUE))
            lines_to_draw.append(("-"))
            lines_to_draw.append(("[当前速度指令 (已发送)]"))
            lines_to_draw.append((f"  左臂速度: {fmt_speed_l}"))
            lines_to_draw.append((f"  右臂速度: {fmt_speed_r}"))
            lines_to_draw.append(("-"))
        elif controller.control_mode == config.MODE_RESET:
            lines_to_draw.append((f"回正速度: {controller.reset_speed:.1f}", C_BLUE))
            lines_to_draw.append(("-"))
            lines_to_draw.append(("[回正模式]"))
            left_reset_btn_idx = controller.reset_left_arm_ctrl.get('index',
                                                                    '?') if controller.reset_left_arm_ctrl else '?'
            right_reset_btn_idx = controller.reset_right_arm_ctrl.get('index',
                                                                      '?') if controller.reset_right_arm_ctrl else '?'
            lines_to_draw.append((f"  按下按钮 B{left_reset_btn_idx} 左臂回正", C_CYAN))
            lines_to_draw.append((f"  按下按钮 B{right_reset_btn_idx} 右臂回正", C_CYAN))
            lines_to_draw.append(("-"))
        elif controller.control_mode == config.MODE_VISION:
            lines_to_draw.append(("-"))
            lines_to_draw.append(("[视觉模式活动]", C_MAGENTA))  # 视觉模式用洋红色
            # 显示来自 vision_interaction 的状态或提示
            vision_status_msg = self.status_message  # 使用 UIManager 的 status_message
            if not vision_status_msg:  # 如果没有特定状态，显示默认
                if vision_interaction.is_recording:
                    vision_status_msg = "录音中...按[确认键]结束, [取消键]取消"
                else:
                    controls_map = self.controller.controls_map if hasattr(self.controller, 'controls_map') else {}
                    start_rec_btn_cfg = controls_map.get('vision_start_record')
                    start_rec_btn_idx = start_rec_btn_cfg.get('index', '?') if start_rec_btn_cfg else '?'
                    vision_status_msg = f"按 B{start_rec_btn_idx} 开始语音指令"

            lines_to_draw.append((f"  状态: {vision_status_msg}", C_GRAY))
            lines_to_draw.append(("-"))

        # 通用提示信息
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

        # 渲染所有行 (与之前类似，但对颜色判断做细微调整)
        for item in lines_to_draw:
            line_text, line_color = (item, C_WHITE) if isinstance(item, str) else item

            if line_text == "-":
                pygame.draw.line(self.screen, C_GRAY,
                                 (INFO_X_MARGIN, y_pos + LINE_SPACING // 2 - 1),  # -1 for slightly better centering
                                 (controller.window_width - INFO_X_MARGIN, y_pos + LINE_SPACING // 2 - 1), 1)
                y_pos += LINE_SPACING
                continue

            # 根据内容动态调整颜色 (可以覆盖 item 中指定的颜色，如果需要更强的规则)
            final_color = line_color  # 默认使用 item 中指定的颜色
            if "ERR" in line_text or "无效" in line_text or "失败" in line_text:
                final_color = C_RED
            elif "OK" in line_text and "ERR" not in line_text:  # 避免 "OK" 和 "ERR" 同时存在时颜色冲突
                final_color = C_GREEN
            # (其他特定颜色规则可以加在这里)

            try:
                text_surface = self.info_font.render(line_text, True, final_color)
                self.screen.blit(text_surface, (INFO_X_MARGIN, y_pos))
            except Exception as render_e:
                if y_pos == INFO_Y_START: print(f"渲染文本 '{line_text}' 时出错: {render_e}")
                try:  # Fallback rendering
                    error_font = pygame.font.Font(None, self.controller.font_size)
                    error_surface = error_font.render("!RENDER_ERR!", True, C_RED)
                    self.screen.blit(error_surface, (INFO_X_MARGIN, y_pos))
                except:
                    pass  # 如果连 fallback 都失败，就没办法了
            y_pos += LINE_SPACING

        pygame.display.flip()

    def quit(self):
        """退出 Pygame"""
        print("正在退出 UIManager...")
        if self.mixer_initialized:
            pygame.mixer.quit()
            print("  Pygame Mixer 已退出。")
        if pygame.get_init():  # 确保 pygame 仍然初始化
            pygame.quit()
            print("  Pygame 已退出。")
        else:
            print("  Pygame 已提前退出或未初始化。")