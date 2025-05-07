# config.py
# -*- coding: utf-8 -*-

import yaml
import traceback
import os

# --- 常量定义 ---
CONFIG_FILE = 'dual_arm_config.yaml'  # 配置文件名

# 默认值 (如果配置文件缺少)
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

# Pygame 颜色 (也可以移到 ui.py)
C_WHITE = (255, 255, 255)
C_BLACK = (0, 0, 0)
C_GREEN = (0, 255, 0)
C_RED = (255, 0, 0)
C_BLUE = (100, 100, 255)
C_YELLOW = (200, 200, 0)
C_MAGENTA = (200, 0, 200)
C_CYAN = (0, 200, 200) # 新增青色用于回正模式
C_GRAY = (150, 150, 150)

# Pygame 显示布局常量 (也可以移到 ui.py)
INFO_X_MARGIN = 20
INFO_Y_START = 20
LINE_SPACING = 25

# 控制模式
MODE_XYZ = 'XYZ'
MODE_RPY = 'RPY'
MODE_VISION = 'VISION'
MODE_RESET = 'RESET' # 新增回正模式

# 定义回正时的目标姿态 RPY (工具垂直向下)
# !!! 请务必根据你的实际坐标系定义调整这些值 !!!
TARGET_RESET_RPY_LEFT = [180.0, 0.0, 180.0]  # 左臂垂直向下示例 RPY
TARGET_RESET_RPY_RIGHT = [180.0, 0.0, 0.0]   # 右臂垂直向下示例 RPY


def load_config(filepath):
    """加载 YAML 配置文件并返回配置字典"""
    print(f"正在加载配置文件: {filepath}")
    try:
        with open(filepath, 'r', encoding='utf-8') as f:
            config_data = yaml.safe_load(f)
        if config_data is None:
            raise ValueError("配置文件为空或格式错误")
        print(f"成功加载配置文件: {filepath}")
        return config_data
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

def load_and_set_config_variables(controller_instance, config_path=CONFIG_FILE):
    """加载 YAML 配置文件并设置控制器实例变量"""
    print(f"正在加载配置文件并设置变量: {config_path}")
    try:
        config = load_config(config_path)
        if config is None:
            raise ValueError("无法加载配置")

        controller_instance.config = config # 保存原始配置

        setup_cfg = config.get('setup', {})
        settings_cfg = config.get('settings', {})
        controls_cfg = config.get('controls', {})
        audio_cfg = config.get('audio_files', {})

        # 从 setup 加载
        controller_instance.font_path = setup_cfg.get('font_path', DEFAULT_FONT_PATH)
        controller_instance.left_robot_ip = setup_cfg.get('left_robot_ip', DEFAULT_IP)
        controller_instance.right_robot_ip = setup_cfg.get('right_robot_ip', DEFAULT_IP)
        controller_instance.left_gripper_id = setup_cfg.get('left_gripper_id', DEFAULT_GRIPPER_ID)
        controller_instance.right_gripper_id = setup_cfg.get('right_gripper_id', DEFAULT_GRIPPER_ID)
        controller_instance.window_width = setup_cfg.get('window_width', DEFAULT_WINDOW_WIDTH)
        controller_instance.window_height = setup_cfg.get('window_height', DEFAULT_WINDOW_HEIGHT)
        controller_instance.font_size = setup_cfg.get('font_size', DEFAULT_FONT_SIZE)

        # 从 settings 加载
        controller_instance.current_speed_xy = settings_cfg.get('initial_xy_speed', DEFAULT_XY_SPEED)
        controller_instance.current_speed_z = settings_cfg.get('initial_z_speed', DEFAULT_Z_SPEED)
        controller_instance.rpy_speed = settings_cfg.get('rpy_speed', DEFAULT_RPY_SPEED)
        controller_instance.speed_increment = settings_cfg.get('speed_increment', DEFAULT_SPEED_INCREMENT)
        controller_instance.min_speed = settings_cfg.get('min_speed', DEFAULT_MIN_SPEED)
        controller_instance.max_speed = settings_cfg.get('max_speed', DEFAULT_MAX_SPEED)
        controller_instance.acc = settings_cfg.get('acc', DEFAULT_ACC)
        controller_instance.arot = settings_cfg.get('arot', DEFAULT_AROT)
        controller_instance.t_interval = settings_cfg.get('t', DEFAULT_T)
        controller_instance.trigger_threshold = settings_cfg.get('trigger_threshold', DEFAULT_TRIGGER_THRESHOLD)
        controller_instance.gripper_speed = settings_cfg.get('gripper_speed', DEFAULT_GRIPPER_SPEED)
        controller_instance.gripper_force = settings_cfg.get('gripper_force', DEFAULT_GRIPPER_FORCE)
        controller_instance.long_press_duration = settings_cfg.get('long_press_duration', DEFAULT_LONG_PRESS_DURATION)
        controller_instance.reset_speed = settings_cfg.get('reset_speed', DEFAULT_RESET_SPEED)

        # 从 controls 加载
        controller_instance.controls_map = controls_cfg
        controller_instance.mode_switch_control = controls_cfg.get('mode_switch_button', {'type': 'button', 'index': DEFAULT_MODE_SWITCH_BUTTON})
        controller_instance.speed_inc_control = controls_cfg.get('speed_increase_alt', {'type': 'button', 'index': DEFAULT_SPEED_INC_BUTTON})
        controller_instance.speed_dec_control = controls_cfg.get('speed_decrease', {'type': 'button', 'index': DEFAULT_SPEED_DEC_BUTTON})
        controller_instance.gripper_toggle_left_ctrl = controls_cfg.get('gripper_toggle_left', {'type': 'button', 'index': DEFAULT_GRIPPER_L_BUTTON})
        controller_instance.gripper_toggle_right_ctrl = controls_cfg.get('gripper_toggle_right', {'type': 'button', 'index': DEFAULT_GRIPPER_R_BUTTON})
        controller_instance.reset_left_arm_ctrl = controls_cfg.get('reset_left_arm')
        controller_instance.reset_right_arm_ctrl = controls_cfg.get('reset_right_arm')

        # 设置轴阈值默认值
        for control in controller_instance.controls_map.values():
             if isinstance(control, dict) and control.get('type') == 'axis' and 'threshold' not in control:
                 control['threshold'] = controller_instance.trigger_threshold

        # 从 audio_files 加载
        controller_instance.audio_files_config = audio_cfg

        print("配置文件加载并设置变量成功。")
        return True

    except FileNotFoundError:
        print(f"错误: 配置文件未找到: {config_path}")
        return False
    except yaml.YAMLError as e:
        print(f"错误: 解析配置文件 {config_path} 失败: {e}")
        return False
    except Exception as e:
        print(f"加载和设置配置时发生未知错误: {e}")
        traceback.print_exc()
        return False