# -*- coding: utf-8 -*- # <--- 建议添加文件编码声明

import pygame
from CPS import * # 确保 CPS.py 可用
import time
import numpy as np
from scipy.spatial.transform import Rotation as R  # 明确导入 Rotation
import traceback
import sys
import os
import yaml

# --- 常量定义 ---
CONFIG_FILE = 'dual_arm_config.yaml'  # 配置文件名

# 默认值 (如果配置文件缺少)
# ... (默认值保持不变) ...
DEFAULT_FONT_PATH = None
DEFAULT_IP = "127.0.0.1"
DEFAULT_GRIPPER_ID = 9
DEFAULT_WINDOW_WIDTH = 800
DEFAULT_WINDOW_HEIGHT = 400
DEFAULT_FONT_SIZE = 18
DEFAULT_XY_SPEED = 40.0
DEFAULT_Z_SPEED = 30.0
DEFAULT_RPY_SPEED = 20.0
DEFAULT_SPEED_INCREMENT = 5.0
DEFAULT_MIN_SPEED = 5.0
DEFAULT_MAX_SPEED = 100.0
DEFAULT_ACC = 100
DEFAULT_AROT = 10
DEFAULT_T = 0.1
DEFAULT_TRIGGER_THRESHOLD = 0.1
DEFAULT_GRIPPER_SPEED = 150
DEFAULT_GRIPPER_FORCE = 100
DEFAULT_LONG_PRESS_DURATION = 0.8
DEFAULT_MODE_SWITCH_BUTTON = 7
DEFAULT_SPEED_INC_BUTTON = 1
DEFAULT_SPEED_DEC_BUTTON = 6
DEFAULT_GRIPPER_L_BUTTON = 9
DEFAULT_GRIPPER_R_BUTTON = 10
DEFAULT_RESET_SPEED = 50 # 新增默认回正速度


# Pygame 颜色
# ... (颜色定义保持不变) ...
C_WHITE = (255, 255, 255)
C_BLACK = (0, 0, 0)
C_GREEN = (0, 255, 0)
C_RED = (255, 0, 0)
C_BLUE = (100, 100, 255)
C_YELLOW = (200, 200, 0)
C_MAGENTA = (200, 0, 200)
C_CYAN = (0, 200, 200) # 新增青色用于回正模式
C_GRAY = (150, 150, 150)


# Pygame 显示布局
INFO_X_MARGIN = 20
INFO_Y_START = 20
LINE_SPACING = 25

# 控制模式
MODE_XYZ = 'XYZ'
MODE_RPY = 'RPY'
MODE_VISION = 'VISION'
MODE_RESET = 'RESET' # 新增回正模式

# 定义回正位置 (示例关节角度，请根据你的机器人和实际需求修改)
# !!! 确保这些值对于你的机器人是安全且合适的回正位置 !!!
RESET_JOINT_LEFT = [90.0, -45.0, 90.0, 0.0, 90.0, 0.0] # 示例左臂回正角度
RESET_JOINT_RIGHT = [-90.0, -45.0, -90.0, 0.0, 90.0, 0.0] # 示例右臂回正角度


# --- 辅助函数 ---
def format_speed(speed_array):
    """格式化速度向量以便打印"""
    # ... (函数保持不变) ...
    if not hasattr(speed_array, '__len__') or len(speed_array) < 6:
        return "[无效速度数据]"
    try:
        return f"[{float(speed_array[0]):>6.1f}, {float(speed_array[1]):>6.1f}, {float(speed_array[2]):>6.1f}, {float(speed_array[3]):>6.1f}, {float(speed_array[4]):>6.1f}, {float(speed_array[5]):>6.1f}]"
    except (ValueError, IndexError):
        return "[格式化错误]"


def load_config(filepath):
    """加载 YAML 配置文件并返回配置字典"""
    # ... (函数保持不变) ...
    print(f"正在加载配置文件: {filepath}")
    try:
        with open(filepath, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)
        if config is None: raise ValueError("配置文件为空或格式错误")
        print(f"成功加载配置文件: {filepath}")
        return config
    except FileNotFoundError:
        print(f"错误: 配置文件未找到: {filepath}")
        return None
    except yaml.YAMLError as e:
        print(f"错误: 解析配置文件 {filepath} 失败: {e}")
        return None
    except Exception as e:
        print(f"加载配置文件时发生未知错误: {e}")
        traceback.print_exc()
        return None


def initialize_robot(controller, arm_name):
    """初始化单个机器人（上电、清报警、同步、使能）"""
    # ... (函数保持不变) ...
    print(f"\n--- 初始化 {arm_name} ---")
    try:
        print(f"[{arm_name}] 1/4: 上电...")
        if not controller.power_on(): raise RuntimeError(f"{arm_name} 上电失败")
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
        if not controller.set_servo(1): raise RuntimeError(f"{arm_name} 伺服使能失败")
        print(f"[{arm_name}] 伺服使能成功.")
        print(f"--- {arm_name} 初始化完成 ---")
        return True
    except Exception as e:
        print(f"错误: 初始化 {arm_name} 时: {e}")
        traceback.print_exc()
        return False



def connect_arm_gripper(controller, arm_name):
    """尝试连接并激活单个夹爪"""
    # ... (函数保持不变) ...
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


def run_vision_mode():
    """视觉模式占位符"""
    # ... (函数保持不变) ...
    print("== 进入视觉模式 (占位符) ==")
    # print("== 视觉模式处理中... ==") # 减少打印
    pass


# --- 主控制器类 ---
class DualArmController:
    """
    使用 Pygame 手柄和配置文件控制双臂机器人（带模式切换和音频反馈）的主类。
    """

    def __init__(self, config_path):
        """初始化控制器"""
        # ... (大部分初始化不变) ...
        self.config_path = config_path
        self.config = None
        self.joystick = None
        self.controller_left = None
        self.controller_right = None
        self.screen = None
        self.info_font = None
        self.clock = None
        self.sound_cache = {}
        self.mixer_initialized = False
        self.num_hats = 0
        self.num_axes = 0
        self.num_buttons = 0

        self.running = False
        self.control_modes = [MODE_XYZ, MODE_RPY, MODE_RESET, MODE_VISION]
        self.current_mode_index = 0
        self.control_mode = self.control_modes[self.current_mode_index]
        # --- DEBUG PRINT ---
        print(f"[DEBUG] Initial mode set in __init__: {self.control_mode}")

        self.current_speed_xy = DEFAULT_XY_SPEED
        self.current_speed_z = DEFAULT_Z_SPEED
        self.rpy_speed = DEFAULT_RPY_SPEED
        self.reset_speed = DEFAULT_RESET_SPEED

        self.left_gripper_open = True
        self.right_gripper_open = True
        self.left_gripper_active = False
        self.right_gripper_active = False
        self.mode_button_press_time = None
        self.left_init_ok = False
        self.right_init_ok = False

        # ... (加载默认值部分不变) ...
        self.font_path = DEFAULT_FONT_PATH
        self.left_robot_ip = DEFAULT_IP
        self.right_robot_ip = DEFAULT_IP
        self.left_gripper_id = DEFAULT_GRIPPER_ID
        self.right_gripper_id = DEFAULT_GRIPPER_ID
        self.window_width = DEFAULT_WINDOW_WIDTH
        self.window_height = DEFAULT_WINDOW_HEIGHT
        self.font_size = DEFAULT_FONT_SIZE
        self.speed_increment = DEFAULT_SPEED_INCREMENT
        self.min_speed = DEFAULT_MIN_SPEED
        self.max_speed = DEFAULT_MAX_SPEED
        self.acc = DEFAULT_ACC
        self.arot = DEFAULT_AROT
        self.t_interval = DEFAULT_T
        self.trigger_threshold = DEFAULT_TRIGGER_THRESHOLD
        self.gripper_speed = DEFAULT_GRIPPER_SPEED
        self.gripper_force = DEFAULT_GRIPPER_FORCE
        self.long_press_duration = DEFAULT_LONG_PRESS_DURATION
        self.controls_map = {}
        self.audio_files_config = {}
        self.mode_switch_control = {'type': 'button', 'index': DEFAULT_MODE_SWITCH_BUTTON}
        self.speed_inc_control = {'type': 'button', 'index': DEFAULT_SPEED_INC_BUTTON}
        self.speed_dec_control = {'type': 'button', 'index': DEFAULT_SPEED_DEC_BUTTON}
        self.gripper_toggle_left_ctrl = {'type': 'button', 'index': DEFAULT_GRIPPER_L_BUTTON}
        self.gripper_toggle_right_ctrl = {'type': 'button', 'index': DEFAULT_GRIPPER_R_BUTTON}
        self.reset_left_arm_ctrl = None
        self.reset_right_arm_ctrl = None


    def _load_config(self):
        """加载 YAML 配置文件并设置实例变量"""
        # ... (方法保持不变) ...
        print(f"正在加载配置文件: {self.config_path}")
        try:
            with open(self.config_path, 'r', encoding='utf-8') as f:
                self.config = yaml.safe_load(f)
            if self.config is None: raise ValueError("配置文件为空或格式错误")
            setup_cfg = self.config.get('setup', {})
            settings_cfg = self.config.get('settings', {})
            self.font_path = setup_cfg.get('font_path', self.font_path)
            self.left_robot_ip = setup_cfg.get('left_robot_ip', self.left_robot_ip)
            self.right_robot_ip = setup_cfg.get('right_robot_ip', self.right_robot_ip)
            self.left_gripper_id = setup_cfg.get('left_gripper_id', self.left_gripper_id)
            self.right_gripper_id = setup_cfg.get('right_gripper_id', self.right_gripper_id)
            self.window_width = setup_cfg.get('window_width', self.window_width)
            self.window_height = setup_cfg.get('window_height', self.window_height)
            self.font_size = setup_cfg.get('font_size', self.font_size)

            self.current_speed_xy = settings_cfg.get('initial_xy_speed', self.current_speed_xy)
            self.current_speed_z = settings_cfg.get('initial_z_speed', self.current_speed_z)
            self.rpy_speed = settings_cfg.get('rpy_speed', self.rpy_speed)
            self.speed_increment = settings_cfg.get('speed_increment', self.speed_increment)
            self.min_speed = settings_cfg.get('min_speed', self.min_speed)
            self.max_speed = settings_cfg.get('max_speed', self.max_speed)
            self.acc = settings_cfg.get('acc', self.acc)
            self.arot = settings_cfg.get('arot', self.arot)
            self.t_interval = settings_cfg.get('t', self.t_interval)
            self.trigger_threshold = settings_cfg.get('trigger_threshold', self.trigger_threshold)
            self.gripper_speed = settings_cfg.get('gripper_speed', self.gripper_speed)
            self.gripper_force = settings_cfg.get('gripper_force', self.gripper_force)
            self.long_press_duration = settings_cfg.get('long_press_duration', self.long_press_duration)
            self.reset_speed = settings_cfg.get('reset_speed', DEFAULT_RESET_SPEED) # Load reset speed

            self.controls_map = self.config.get('controls', {})
            self.audio_files_config = self.config.get('audio_files', {})

            # Load specific controls, using defaults if not in config
            self.mode_switch_control = self.controls_map.get('mode_switch_button', self.mode_switch_control)
            self.speed_inc_control = self.controls_map.get('speed_increase_alt', self.speed_inc_control)
            self.speed_dec_control = self.controls_map.get('speed_decrease', self.speed_dec_control)
            self.gripper_toggle_left_ctrl = self.controls_map.get('gripper_toggle_left', self.gripper_toggle_left_ctrl)
            self.gripper_toggle_right_ctrl = self.controls_map.get('gripper_toggle_right', self.gripper_toggle_right_ctrl)
            self.reset_left_arm_ctrl = self.controls_map.get('reset_left_arm') # Load new reset controls
            self.reset_right_arm_ctrl = self.controls_map.get('reset_right_arm') # Load new reset controls


            for control in self.controls_map.values():
                if isinstance(control, dict) and control.get('type') == 'axis' and 'threshold' not in control: control['threshold'] = self.trigger_threshold
            print("配置文件加载成功。")
            return True
        except FileNotFoundError:
            print(f"错误: 配置文件未找到: {self.config_path}")
            return False
        except yaml.YAMLError as e:
            print(f"错误: 解析配置文件 {self.config_path} 失败: {e}")
            return False
        except Exception as e:
            print(f"加载配置文件时发生未知错误: {e}")
            traceback.print_exc()
            return False


    def _init_pygame(self):
        """初始化 Pygame 相关模块"""
        # ... (方法保持不变) ...
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
            self.screen = pygame.display.set_mode((self.window_width, self.window_height))
            pygame.display.set_caption(f"双臂控制 (模式: {self.control_mode})")
            if self.font_path and os.path.exists(self.font_path):
                try:
                    self.info_font = pygame.font.Font(self.font_path, self.font_size)
                    print(f"  字体加载成功: {self.font_path}")
                except Exception as e:
                    print(f"  字体加载失败: {e}")
                    self.info_font = None
            if self.info_font is None:
                print(f"  警告: 将使用默认字体。")
                try:
                    self.info_font = pygame.font.Font(None, self.font_size)
                except Exception as e:
                    raise RuntimeError(f"加载默认字体失败: {e}") from e
            joystick_count = pygame.joystick.get_count()
            if joystick_count == 0: raise RuntimeError("未检测到手柄！")
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            print(f"  已初始化手柄: {self.joystick.get_name()}")
            self.num_hats = self.joystick.get_numhats()
            self.num_axes = self.joystick.get_numaxes()
            self.num_buttons = self.joystick.get_numbuttons()
            print(f"    能力: Hats={self.num_hats}, Axes={self.num_axes}, Buttons={self.num_buttons}")
            self.clock = pygame.time.Clock()
            print("Pygame 初始化完成。")
            return True
        except Exception as e:
            print(f"Pygame 初始化失败: {e}")
            traceback.print_exc()
            return False


    def _load_sounds(self):
        """加载音频文件"""
        # ... (方法保持不变) ...
        if not self.mixer_initialized: return
        print("\n正在加载音频文件...")
        loaded_count = 0
        sound_events_to_load = ['xyz_mode', 'rpy_mode', 'vision_enter', 'vision_exit',
                                'left_open', 'left_close', 'right_open', 'right_close',
                                'gripper_inactive', 'system_ready',
                                'reset_mode', 'left_reset_fail', 'right_reset_fail']

        for event_name in sound_events_to_load:
            filepath = self.audio_files_config.get(event_name)
            if not filepath or not isinstance(filepath, str):
                 print(f"  警告: 配置文件中未找到 '{event_name}' 的音频路径或格式错误.")
                 continue
            try:
                if os.path.exists(filepath):
                    sound = pygame.mixer.Sound(filepath)
                    self.sound_cache[event_name] = sound
                    loaded_count += 1
                else:
                    print(f"  错误: 文件未找到 '{event_name}': {filepath}")
            except Exception as e:
                print(f"  错误: 加载 '{event_name}' ({filepath}) 时: {e}")
        print(f"音频文件加载完成 ({loaded_count} 个)。")


    def _play_sound(self, event_name):
        """播放音频"""
        # ... (方法保持不变) ...
        if not self.mixer_initialized: return
        sound = self.sound_cache.get(event_name)
        if sound:
            try:
                sound.play()
            except Exception as e:
                print(f"错误: 播放声音 '{event_name}' 时: {e}")


    def _init_robots(self):
        """连接并初始化机器人和夹爪"""
        print("\n正在连接和初始化机器人...")
        # --- DEBUG PRINT ---
        print("[DEBUG] Entering _init_robots()...")
        try:
            print(f"连接左臂 ({self.left_robot_ip})...")
            self.controller_left = CPSClient(self.left_robot_ip, gripper_slave_id=self.left_gripper_id)
            if not self.controller_left.connect(): raise ConnectionError("左臂连接失败")
            print("左臂连接成功.")
            self.left_init_ok = initialize_robot(self.controller_left, "左臂")
            if self.left_init_ok: self.left_gripper_active = connect_arm_gripper(self.controller_left, "左臂")

            print(f"连接右臂 ({self.right_robot_ip})...")
            self.controller_right = CPSClient(self.right_robot_ip, gripper_slave_id=self.right_gripper_id)
            if not self.controller_right.connect(): raise ConnectionError("右臂连接失败")
            print("右臂连接成功.")
            self.right_init_ok = initialize_robot(self.controller_right, "右臂")
            if self.right_init_ok: self.right_gripper_active = connect_arm_gripper(self.controller_right, "右臂")

            if not (self.left_init_ok and self.right_init_ok): print("警告：一个或两个机械臂未能完全初始化。")
            print("机器人初始化流程结束。")
            # --- DEBUG: 确认这里没有调用回正 ---
            print("[DEBUG] _init_robots() finished. No reset commands called here.")
        except Exception as e:
            print(f"机器人连接或初始化过程中断: {e}")
            traceback.print_exc()
            self.left_init_ok = False
            self.right_init_ok = False


    def _handle_events(self):
        """处理 Pygame 事件（退出、按钮按下/松开）"""
        current_time = time.time()
        for event in pygame.event.get():
            # --- DEBUG PRINT ---
            # 取消下面的注释可以查看所有事件
            # print(f"[DEBUG] Event: {pygame.event.event_name(event.type)}")

            if event.type == pygame.QUIT: self.running = False; return
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE: self.running = False; return

            if event.type == pygame.JOYBUTTONDOWN:
                button_index = event.button
                # --- DEBUG PRINT ---
                print(f"[DEBUG] JOYBUTTONDOWN detected: Button Index = {button_index}")

                # Record press time for mode switch button
                if self.mode_switch_control.get('type') == 'button' and button_index == self.mode_switch_control.get('index', -1):
                    if self.mode_button_press_time is None: # Only record if not already pressed
                        self.mode_button_press_time = current_time
                        print(f"[DEBUG] Mode switch button (Index {button_index}) PRESSED at {current_time}")

                # Speed controls
                elif self.speed_inc_control.get('type') == 'button' and button_index == self.speed_inc_control.get('index', -1):
                    self.current_speed_xy = min(self.max_speed, self.current_speed_xy + self.speed_increment)
                    self.current_speed_z = min(self.max_speed, self.current_speed_z + self.speed_increment)
                elif self.speed_dec_control.get('type') == 'button' and button_index == self.speed_dec_control.get('index', -1):
                    self.current_speed_xy = max(self.min_speed, self.current_speed_xy - self.speed_increment)
                    self.current_speed_z = max(self.min_speed, self.current_speed_z - self.speed_increment)

                # Gripper controls
                elif self.gripper_toggle_left_ctrl and self.gripper_toggle_left_ctrl.get('type') == 'button' and button_index == self.gripper_toggle_left_ctrl.get('index', -1):
                    if self.left_gripper_active:
                        sound_event = 'left_close' if self.left_gripper_open else 'left_open'
                        self._play_sound(sound_event)
                        (self.controller_left.close_gripper if self.left_gripper_open else self.controller_left.open_gripper)(
                            speed=self.gripper_speed, force=self.gripper_force, wait=False)
                        self.left_gripper_open = not self.left_gripper_open
                    else: self._play_sound('gripper_inactive')
                elif self.gripper_toggle_right_ctrl and self.gripper_toggle_right_ctrl.get('type') == 'button' and button_index == self.gripper_toggle_right_ctrl.get('index', -1):
                    if self.right_gripper_active:
                        sound_event = 'right_close' if self.right_gripper_open else 'right_open'
                        self._play_sound(sound_event)
                        (self.controller_right.close_gripper if self.right_gripper_open else self.controller_right.open_gripper)(
                            speed=self.gripper_speed, force=self.gripper_force, wait=False)
                        self.right_gripper_open = not self.right_gripper_open
                    else: self._play_sound('gripper_inactive')

                # === 回正按钮触发逻辑 ===
                # 只有在 RESET 模式下才检查回正按钮
                if self.control_mode == MODE_RESET:
                    # --- DEBUG PRINT ---
                    print(f"[DEBUG] Now in RESET mode. Checking if button {button_index} matches reset controls.")
                    left_reset_idx = self.reset_left_arm_ctrl.get('index', -99) if self.reset_left_arm_ctrl else -99 # Use unlikely index if None
                    right_reset_idx = self.reset_right_arm_ctrl.get('index', -99) if self.reset_right_arm_ctrl else -99

                    if self.reset_left_arm_ctrl and self.reset_left_arm_ctrl.get('type') == 'button' and button_index == left_reset_idx:
                        # --- DEBUG PRINT ---
                        print(f"[DEBUG] Left reset button (Index {left_reset_idx}) pressed IN reset mode. Calling _attempt_reset_left...")
                        print("左臂回正按钮按下...")
                        self._attempt_reset_left() # 调用回正方法
                    elif self.reset_right_arm_ctrl and self.reset_right_arm_ctrl.get('type') == 'button' and button_index == right_reset_idx:
                        # --- DEBUG PRINT ---
                        print(f"[DEBUG] Right reset button (Index {right_reset_idx}) pressed IN reset mode. Calling _attempt_reset_right...")
                        print("右臂回正按钮按下...")
                        self._attempt_reset_right() # 调用回正方法
                    # --- DEBUG PRINT ---
                    # else:
                    #    print(f"[DEBUG] Button {button_index} pressed in RESET mode, but it's not a defined reset button (L:{left_reset_idx}, R:{right_reset_idx}).")
                # --- DEBUG PRINT ---
                # else: # 如果需要，可以取消注释，看是否在其他模式按下了回正键
                #    left_reset_idx = self.reset_left_arm_ctrl.get('index', -99) if self.reset_left_arm_ctrl else -99
                #    right_reset_idx = self.reset_right_arm_ctrl.get('index', -99) if self.reset_right_arm_ctrl else -99
                #    if button_index == left_reset_idx or button_index == right_reset_idx:
                #        print(f"[DEBUG] Reset button (Index {button_index}) pressed, but current mode is {self.control_mode}. Ignored.")


            if event.type == pygame.JOYBUTTONUP:
                button_index = event.button
                # --- DEBUG PRINT ---
                # print(f"[DEBUG] JOYBUTTONUP detected: Button Index = {button_index}")

                # Mode switching logic on button UP (Short press cycles all modes)
                if self.mode_switch_control.get('type') == 'button' and button_index == self.mode_switch_control.get('index', -1):
                    if self.mode_button_press_time is not None:
                        press_duration = current_time - self.mode_button_press_time
                        print(f"[DEBUG] Mode switch button (Index {button_index}) RELEASED. Duration: {press_duration:.3f}s") # DEBUG PRINT

                        # Check if it was a short press
                        if press_duration < self.long_press_duration:
                            # --- DEBUG PRINT ---
                            print(f"[DEBUG] Short press detected. Current mode: {self.control_mode}, Index: {self.current_mode_index}")
                            # Simply increment index to cycle through ALL modes in the list
                            self.current_mode_index = (self.current_mode_index + 1) % len(self.control_modes)
                            self.control_mode = self.control_modes[self.current_mode_index]
                            # --- DEBUG PRINT ---
                            print(f"[DEBUG] New mode selected: {self.control_mode}, New Index: {self.current_mode_index}")

                            # Play sound for the NEW mode
                            if self.control_mode == MODE_XYZ: self._play_sound('xyz_mode')
                            elif self.control_mode == MODE_RPY: self._play_sound('rpy_mode')
                            elif self.control_mode == MODE_RESET: self._play_sound('reset_mode')
                            elif self.control_mode == MODE_VISION: self._play_sound('vision_enter')

                            # Update window caption
                            pygame.display.set_caption(f"双臂控制 (模式: {self.control_mode})")
                        # else: # Optional DEBUG for long press release
                        #    print(f"[DEBUG] Long press release detected. No action on button UP for long press.")

                        # Always reset press time after button up is processed
                        self.mode_button_press_time = None
                        print("[DEBUG] mode_button_press_time reset to None.") # DEBUG PRINT
                    # else: # Optional DEBUG if button up without recorded press time
                    #    print(f"[DEBUG] Mode switch button {button_index} UP event, but no press_time recorded.")


    def _attempt_reset_left(self):
        """尝试将左臂回正到预设位置"""
        # --- DEBUG PRINT ---
        print("[DEBUG] Entered _attempt_reset_left()")
        if not self.left_init_ok or not self.controller_left:
            print("左臂未初始化，无法回正。")
            self._play_sound('left_reset_fail')
            return

        print(f"尝试将左臂回正到关节角度: {RESET_JOINT_LEFT}")
        try:
            # Blocking call to wait for movement completion
            rpy_angles = desire_left_pose(rpy_array=[180,0,180]) # left 垂直向下
            pose = self.controller_left.getTCPPose()
            pose[3:6] = rpy_angles

            success = self.controller_left.move_robot(pose, speed=self.reset_speed, block=True)
            if success: print("左臂回正成功。")
            else: print("左臂回正失败。"); self._play_sound('left_reset_fail')
        except Exception as e:
            print(f"左臂回正过程中发生异常: {e}")
            traceback.print_exc()
            self._play_sound('left_reset_fail')


    def _attempt_reset_right(self):
        """尝试将右臂回正到预设位置"""
        # --- DEBUG PRINT ---
        print("[DEBUG] Entered _attempt_reset_right()")
        if not self.right_init_ok or not self.controller_right:
            print("右臂未初始化，无法回正。")
            self._play_sound('right_reset_fail')
            return

        print(f"尝试将右臂回正到关节角度")
        try:
            rpy_angles = desire_right_pose(rpy_array=[180, 0, 0])  # left 垂直向下
            pose = self.controller_right.getTCPPose()
            pose[3:6] = rpy_angles

            # Assuming moveByJoint_right is correct for the right arm:
            success = self.controller_right.move_right_robot(pose, speed=self.reset_speed, block=True)
            if success: print("右臂回正成功。")
            else: print("右臂回正失败。"); self._play_sound('right_reset_fail')
        except Exception as e:
            print(f"右臂回正过程中发生异常: {e}")
            traceback.print_exc()
            self._play_sound('right_reset_fail')


    def _get_joystick_input_state(self, control_config):
        """检查单个控制配置是否被激活"""
        # ... (方法保持不变) ...
        if not control_config: return False
        active = False
        ctrl_type = control_config.get('type')
        ctrl_index = control_config.get('index', -1)
        if ctrl_type == 'button' and not (0 <= ctrl_index < self.num_buttons): return False
        if ctrl_type == 'axis' and not (0 <= ctrl_index < self.num_axes): return False
        if ctrl_type == 'hat' and not (0 <= ctrl_index < self.num_hats): return False
        try:
            if ctrl_type == 'button':
                active = self.joystick.get_button(ctrl_index) == 1
            elif ctrl_type == 'axis':
                axis_val = self.joystick.get_axis(ctrl_index)
                threshold = control_config.get('threshold', self.trigger_threshold)
                direction = control_config.get('direction', 1)
                if direction == 1 and axis_val > threshold: active = True
                elif direction == -1 and axis_val < -threshold: active = True
            elif ctrl_type == 'hat':
                hat_val = self.joystick.get_hat(ctrl_index)
                hat_axis = control_config.get('axis', 'x')
                direction = control_config.get('direction', 1)
                if hat_axis == 'x' and hat_val[0] == direction: active = True
                elif hat_axis == 'y' and hat_val[1] == direction: active = True
        except pygame.error as e:
            print(f"读取手柄输入时出错({ctrl_type},{ctrl_index}): {e}")
            return False
        return active


    def _calculate_speed_commands(self):
        """根据当前模式和输入计算原始速度指令"""
        # ... (方法保持不变) ...
        speed_left_cmd = np.zeros(6)
        speed_right_cmd = np.zeros(6)
        current_mode_prefix = self.control_mode.lower() + "_"

        if self.control_mode in [MODE_XYZ, MODE_RPY]:
            for action, control in self.controls_map.items():
                if not isinstance(control, dict): continue
                if action.startswith(current_mode_prefix) and ('_arm' in action or '_gripper' in action): # '_gripper' check seems redundant here
                     if self._get_joystick_input_state(control):
                        target_speed_array = None; axis_index = -1; base_speed = 0.0; direction_sign = 1.0
                        if 'left_arm' in action: target_speed_array = speed_left_cmd
                        elif 'right_arm' in action: target_speed_array = speed_right_cmd
                        if target_speed_array is not None:
                            if self.control_mode == MODE_XYZ:
                                if '_x' in action: axis_index = 0; base_speed = self.current_speed_xy
                                elif '_y' in action: axis_index = 1; base_speed = self.current_speed_xy
                                elif '_z' in action: axis_index = 2; base_speed = self.current_speed_z
                            elif self.control_mode == MODE_RPY:
                                if '_roll' in action: axis_index = 3; base_speed = self.rpy_speed
                                elif '_pitch' in action: axis_index = 4; base_speed = self.rpy_speed
                                elif '_yaw' in action: axis_index = 5; base_speed = self.rpy_speed
                            if '_pos' in action: direction_sign = 1.0
                            elif '_neg' in action: direction_sign = -1.0
                            if axis_index != -1:
                                if control.get('type') == 'axis':
                                    axis_val = self.joystick.get_axis(control.get('index', -1))
                                    scaled_speed = base_speed * abs(axis_val)
                                    target_speed_array[axis_index] = scaled_speed * direction_sign
                                else: target_speed_array[axis_index] = base_speed * direction_sign
        elif self.control_mode == MODE_VISION:
            run_vision_mode()
            speed_left_cmd = np.zeros(6)
            speed_right_cmd = np.zeros(6)
        elif self.control_mode == MODE_RESET:
            speed_left_cmd = np.zeros(6)
            speed_right_cmd = np.zeros(6)
        return speed_left_cmd, speed_right_cmd


    def _apply_transformations(self, speed_left_cmd, speed_right_cmd):
        """对速度指令应用坐标变换"""
        # ... (方法基本保持不变, RPY 部分的逻辑简化为直接传递角速度) ...
        speed_left_final = np.zeros(6)
        speed_right_final = np.zeros(6)

        if self.control_mode == MODE_XYZ:
            speed_left_final = speed_left_cmd.copy()
            speed_right_final = speed_right_cmd.copy()
            try:
                rot_left = R.from_euler('xyz', [65, 0, 10], degrees=True).as_matrix() # Example
                speed_left_final[0:3] = speed_left_cmd[0:3] @ rot_left
            except Exception: print("左臂XYZ转换错误"); speed_left_final[0:3] = [0, 0, 0]
            try:
                rot_right = R.from_euler('xyz', [65.334, -4.208, -9.079], degrees=True).as_matrix() # Example
                speed_right_final[0:3] = speed_right_cmd[0:3] @ rot_right
            except Exception: print("右臂XYZ转换错误"); speed_right_final[0:3] = [0, 0, 0]
            speed_left_final[3:] = [0.0, 0.0, 0.0]
            speed_right_final[3:] = [0.0, 0.0, 0.0]
        elif self.control_mode == MODE_RPY:
            # RPY 模式直接使用计算出的角速度，假设是基坐标系下的控制
            speed_left_final[0:3] = [0.0, 0.0, 0.0]
            speed_right_final[0:3] = [0.0, 0.0, 0.0]
            speed_left_final[3:] = speed_left_cmd[3:]
            speed_right_final[3:] = speed_right_cmd[3:]
        elif self.control_mode in [MODE_RESET, MODE_VISION]:
            speed_left_final = np.zeros(6)
            speed_right_final = np.zeros(6)
        return speed_left_final, speed_right_final

    def _send_robot_commands(self, speed_left_final, speed_right_final):
        """发送速度指令到机器人 (使用 jog)"""

        # 定义一个速度转换函数 (示例)
        def map_speed(speed):
            # 将速度从 m/s 或 rad/s 转换为 jog 的速度百分比 (0.05 - 100)
            # 这只是一个示例，你需要根据实际情况调整
            jog_speed = abs(speed)  # 取绝对值
            jog_speed = max(self.min_speed, min(self.max_speed, jog_speed))  # 限制在范围内
            return jog_speed

        if self.control_mode == MODE_XYZ:
            if self.controller_left and self.left_init_ok:
                try:
                    self.controller_left.moveBySpeedl(list(speed_left_final), self.acc, self.arot, self.t_interval)
                except Exception as e:
                    print(f"发送左臂指令失败: {e}")
            if self.controller_right and self.right_init_ok:
                try:
                    self.controller_right.moveBySpeedl(list(speed_right_final), self.acc, self.arot, self.t_interval)
                except Exception as e:
                    print(f"发送右臂指令失败: {e}")
        elif self.control_mode == MODE_RPY:
            # 左臂控制
            if self.controller_left and self.left_init_ok:
                try:
                    # 线性轴
                    if speed_left_final[0] > 0.05:
                        self.controller_left.jog(index=0, speed=map_speed(speed_left_final[0]))  # x+
                    elif speed_left_final[0] < 0:
                        self.controller_left.jog(index=1, speed=map_speed(-speed_left_final[0]))  # x-
                    # 停止
                    if speed_left_final[1] > 0:
                        self.controller_left.jog(index=2, speed=map_speed(speed_left_final[1]))  # y+
                    elif speed_left_final[1] < 0:
                        self.controller_left.jog(index=3, speed=map_speed(-speed_left_final[1]))  # y-

                    if speed_left_final[2] > 0:
                        self.controller_left.jog(index=4, speed=map_speed(speed_left_final[2]))  # z+
                    elif speed_left_final[2] < 0:
                        self.controller_left.jog(index=5, speed=map_speed(-speed_left_final[2]))  # z-

                    # 旋转轴
                    if speed_left_final[3] > 0:
                        self.controller_left.jog(index=6, speed=map_speed(speed_left_final[3]))  # rx+
                    elif speed_left_final[3] < 0:
                        self.controller_left.jog(index=7, speed=map_speed(-speed_left_final[3]))  # rx-

                    if speed_left_final[4] > 0:
                        self.controller_left.jog(index=8, speed=map_speed(speed_left_final[4]))  # ry+
                    elif speed_left_final[4] < 0:
                        self.controller_left.jog(index=9, speed=map_speed(-speed_left_final[4]))  # ry-
                    if speed_left_final[5] > 0:
                        self.controller_left.jog(index=10, speed=map_speed(speed_left_final[5]))  # rz+
                    elif speed_left_final[5] < 0:
                        self.controller_left.jog(index=11, speed=map_speed(-speed_left_final[5]))  # rz-
                except Exception as e:
                    print(f"发送左臂 jog 指令失败: {e}")

            # 右臂控制 (类似左臂)
            if self.controller_right and self.right_init_ok:
                try:
                    # 线性轴
                    if speed_right_final[0] > 0:
                        self.controller_right.jog(index=0, speed=map_speed(speed_right_final[0]))  # x+
                    elif speed_right_final[0] < 0:
                        self.controller_right.jog(index=1, speed=map_speed(-speed_right_final[0]))  # x-
                    # 停止
                    if speed_right_final[1] > 0:
                        self.controller_right.jog(index=2, speed=map_speed(speed_right_final[1]))  # y+
                    elif speed_right_final[1] < 0:
                        self.controller_right.jog(index=3, speed=map_speed(-speed_right_final[1]))  # y-
                    if speed_right_final[2] > 0:
                        self.controller_right.jog(index=4, speed=map_speed(speed_right_final[2]))  # z+
                    elif speed_right_final[2] < 0:
                        self.controller_right.jog(index=5, speed=map_speed(-speed_right_final[2]))  # z-
                    # 旋转轴
                    if speed_right_final[3] > 0:
                        self.controller_right.jog(index=6, speed=map_speed(speed_right_final[3]))  # rx+
                    elif speed_right_final[3] < 0:
                        self.controller_right.jog(index=7, speed=map_speed(-speed_right_final[3]))  # rx-
                    if speed_right_final[4] > 0:
                        self.controller_right.jog(index=8, speed=map_speed(speed_right_final[4]))  # ry+
                    elif speed_right_final[4] < 0:
                        self.controller_right.jog(index=9, speed=map_speed(-speed_right_final[4]))  # ry-
                    if speed_right_final[5] > 0:
                        self.controller_right.jog(index=10, speed=map_speed(speed_right_final[5]))  # rz+
                    elif speed_right_final[5] < 0:
                        self.controller_right.jog(index=11, speed=map_speed(-speed_right_final[5]))  # rz-

                except Exception as e:
                    print(f"发送右臂 jog 指令失败: {e}")

    def _draw_display(self, speed_left_final, speed_right_final):
        """绘制 Pygame 显示窗口"""
        # ... (方法保持不变) ...
        if not self.screen or not self.info_font: return
        self.screen.fill(C_BLACK)
        lines_to_draw = []
        y_pos = INFO_Y_START

        fmt_speed_l = format_speed(speed_left_final)
        fmt_speed_r = format_speed(speed_right_final)

        mode_color = C_WHITE; mode_name_cn = ""
        if self.control_mode == MODE_XYZ: mode_color = C_WHITE; mode_name_cn = "XYZ模式"
        elif self.control_mode == MODE_RPY: mode_color = C_YELLOW; mode_name_cn = "RPY模式"
        elif self.control_mode == MODE_VISION: mode_color = C_MAGENTA; mode_name_cn = "视觉模式"
        elif self.control_mode == MODE_RESET: mode_color = C_CYAN; mode_name_cn = "回正模式"

        lines_to_draw.append((f"当前模式: {mode_name_cn}", mode_color))
        lines_to_draw.append((f"左臂({self.left_robot_ip}): {'OK' if self.left_init_ok else 'ERR'} | 右臂({self.right_robot_ip}): {'OK' if self.right_init_ok else 'ERR'}"))
        lines_to_draw.append((f"左夹爪: {'打开' if self.left_gripper_open else '关闭'} ({'活动' if self.left_gripper_active else '无效'}) | 右夹爪: {'打开' if self.right_gripper_open else '关闭'} ({'活动' if self.right_gripper_active else '无效'})", C_YELLOW))

        if self.control_mode in [MODE_XYZ, MODE_RPY]:
             lines_to_draw.append((f"速度 XY: {self.current_speed_xy:.1f} | Z: {self.current_speed_z:.1f} | RPY: {self.rpy_speed:.1f} (B{self.speed_dec_control.get('index', '?')}/B{self.speed_inc_control.get('index', '?')} 调速)", C_BLUE))
             lines_to_draw.append(("-"))
             lines_to_draw.append(("[当前速度指令 (已发送)]"))
             lines_to_draw.append((f"  左臂速度: {fmt_speed_l}"))
             lines_to_draw.append((f"  右臂速度: {fmt_speed_r}"))
             lines_to_draw.append(("-"))
        elif self.control_mode == MODE_RESET:
             lines_to_draw.append((f"回正速度: {self.reset_speed:.1f}", C_BLUE))
             lines_to_draw.append(("-"))
             lines_to_draw.append(("[回正模式]"))
             left_reset_btn_idx = self.reset_left_arm_ctrl.get('index', '?') if self.reset_left_arm_ctrl else '?'
             right_reset_btn_idx = self.reset_right_arm_ctrl.get('index', '?') if self.reset_right_arm_ctrl else '?'
             lines_to_draw.append((f"  按下按钮 B{left_reset_btn_idx} 左臂回正", C_CYAN))
             lines_to_draw.append((f"  按下按钮 B{right_reset_btn_idx} 右臂回正", C_CYAN))
             lines_to_draw.append(("-"))
        elif self.control_mode == MODE_VISION:
             lines_to_draw.append(("-"))
             lines_to_draw.append(("[视觉模式活动]"))
             lines_to_draw.append(("  等待视觉处理...", C_GRAY))
             lines_to_draw.append(("-"))

        mode_switch_btn_idx = self.mode_switch_control.get('index', '?')
        gripper_l_btn_idx = self.gripper_toggle_left_ctrl.get('index', '?') if self.gripper_toggle_left_ctrl else '?'
        gripper_r_btn_idx = self.gripper_toggle_right_ctrl.get('index', '?') if self.gripper_toggle_right_ctrl else '?'
        hints = f"提示: B{mode_switch_btn_idx}切换 | 夹爪L(B{gripper_l_btn_idx}) R(B{gripper_r_btn_idx})" # 移除了长按提示
        if self.control_mode in [MODE_XYZ, MODE_RPY]:
             speed_dec_btn_idx = self.speed_dec_control.get('index', '?')
             speed_inc_btn_idx = self.speed_inc_control.get('index', '?')
             hints += f" | 速度(B{speed_dec_btn_idx}/B{speed_inc_btn_idx})"
        lines_to_draw.append((hints, C_GRAY))

        for item in lines_to_draw:
            line_text = item; line_color = C_WHITE
            if isinstance(item, tuple): line_text, line_color = item
            if line_text == "-": pygame.draw.line(self.screen, C_GRAY, (INFO_X_MARGIN, y_pos + LINE_SPACING // 2), (self.window_width - INFO_X_MARGIN, y_pos + LINE_SPACING // 2), 1); y_pos += LINE_SPACING; continue
            try:
                if "ERR" in line_text or "无效" in line_text: line_color = C_RED
                elif "OK" in line_text and "ERR" not in line_text and "无效" not in line_text: line_color = C_GREEN
                elif "当前模式" in line_text: pass # 已设置
                elif "速度" in line_text and ":" in line_text: line_color = C_BLUE
                elif "夹爪:" in line_text: line_color = C_YELLOW
                elif "[回正模式]" in line_text or "按下按钮" in line_text: line_color = C_CYAN
                elif "[视觉模式活动]" in line_text: line_color = C_MAGENTA
                text_surface = self.info_font.render(line_text, True, line_color)
                self.screen.blit(text_surface, (INFO_X_MARGIN, y_pos))
            except Exception as render_e:
                if y_pos == INFO_Y_START: print(f"渲染文本时出错: {render_e}")
                try: error_font = pygame.font.Font(None, self.font_size); error_surface = error_font.render("! RENDER ERR !", True, C_RED); self.screen.blit(error_surface, (INFO_X_MARGIN, y_pos))
                except: pass
            y_pos += LINE_SPACING
        pygame.display.flip()


    def run(self):
        """主运行循环"""
        if not self._load_config(): return
        if not self._init_pygame(): return
        # --- DEBUG: 确认 _init_robots 不会调用回正 ---
        self._init_robots() # 这步不应引起回正
        # --- DEBUG: 确认初始模式 ---
        print(f"[DEBUG] After init, before loop. Mode: {self.control_mode}")

        # --- DEBUG: 检查是否有意外的回正调用 ---
        # self._attempt_reset_left() # 确保这里没有调用
        # self._attempt_reset_right() # 确保这里没有调用

        self._play_sound('system_ready')
        self.running = True
        print("\n--- 控制循环开始 ---")
        print(f"[DEBUG] Entering main loop. Initial mode should be {self.control_modes[0]}. Current mode is {self.control_mode}")

        while self.running:
            self._handle_events() # 处理输入，可能改变 self.control_mode 或调用回正
            if not self.running: break

            # --- DEBUG: 打印当前循环的模式 ---
            # print(f"[DEBUG] Loop start. Current mode: {self.control_mode}")

            speed_left_cmd, speed_right_cmd = self._calculate_speed_commands()
            speed_left_final, speed_right_final = self._apply_transformations(speed_left_cmd, speed_right_cmd)

            # 发送指令 (内部已处理 RESET/VISION 模式下发送零速度)
            if self.left_init_ok or self.right_init_ok:
                self._send_robot_commands(speed_left_final, speed_right_final)

            self._draw_display(speed_left_final, speed_right_final)
            self.clock.tick(30) # 控制循环频率

    def cleanup(self):
        """清理资源"""
        # ... (方法保持不变) ...
        print("\n正在停止机器人并执行清理操作...")
        stop_speed = [0.0] * 6
        try:
            # 尝试发送停止指令
            if self.left_init_ok and self.controller_left and self.controller_left.sock:
                print("  发送停止指令到左臂...")
                try: self.controller_left.moveBySpeedl(stop_speed, self.acc, self.arot, self.t_interval)
                except Exception as e: print(f"  左臂停止指令失败: {e}")
                time.sleep(0.1)
                self.controller_left.disconnect()
                print("  左臂已断开。")
            if self.right_init_ok and self.controller_right and self.controller_right.sock:
                print("  发送停止指令到右臂...")
                try: self.controller_right.moveBySpeedl(stop_speed, self.acc, self.arot, self.t_interval)
                except Exception as e: print(f"  右臂停止指令失败: {e}")
                time.sleep(0.1)
                self.controller_right.disconnect()
                print("  右臂已断开。")
        except Exception as stop_e:
            print(f"  断开连接时出错: {stop_e}")

        if self.mixer_initialized: pygame.mixer.quit(); print("Pygame Mixer 已退出。")
        pygame.quit()
        print("程序退出。")


# -- 主程序入口 --
if __name__ == "__main__":
    print("启动双臂控制器...")
    # --- DEBUG: 确保配置文件路径正确 ---
    print(f"[DEBUG] Using config file: {os.path.abspath(CONFIG_FILE)}")
    controller_app = DualArmController(CONFIG_FILE)
    try:
        controller_app.run()
    except KeyboardInterrupt: # 添加 KeyboardInterrupt 处理
        print("\n检测到手动中断 (Ctrl+C)...")
    except Exception as e:
        print("\n主程序运行时发生未捕获的异常:")
        traceback.print_exc()
    finally:
        controller_app.cleanup()