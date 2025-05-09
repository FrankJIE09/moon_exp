# main_controller.py
# -*- coding: utf-8 -*-

"""
主控制器模块，负责协调双臂机器人的控制、用户界面、
配置加载以及视觉交互功能。
"""

import time
import numpy as np
import traceback
import threading
import os
import yaml
import pygame
from typing import Dict, Optional, Any, List, Tuple

# --- Vision/Camera/Math Imports ---
try:
    from camera.orbbec_camera import initialize_connected_cameras

    CAMERA_AVAILABLE = True
    CameraClientType = Any
except ImportError:
    print("警告: 未找到相机初始化函数 'initialize_connected_cameras'. 视觉功能将受限。")
    CAMERA_AVAILABLE = False
    CameraClientType = None


    def initialize_connected_cameras(serial):
        return None

try:
    from ultralytics import YOLO

    YOLO_AVAILABLE = True
    YoloModelType = YOLO
except ImportError:
    print("警告: 未找到 'ultralytics' (YOLO). 视觉检测功能将受限。")
    YOLO_AVAILABLE = False
    YoloModelType = None


    class YOLO:
        def __init__(self, *args, **kwargs): pass

        def predict(self, *args, **kwargs): return []

try:
    from scipy.spatial.transform import Rotation as R

    SCIPY_AVAILABLE = True
except ImportError:
    print("警告: 未找到 'scipy'. 坐标变换功能将受限。")
    SCIPY_AVAILABLE = False
    R = None

# --- Local Module Imports ---
import config  # Import your config.py
from robot_control import (initialize_robot, connect_arm_gripper, format_speed,
                           attempt_reset_arm, send_jog_command)
from ui import UIManager
from CPS import CPSClient, desire_right_pose, desire_left_pose

import vision_interaction

PoseType = List[float]
JointType = List[float]
ControllerDict = Dict[str, Optional[CPSClient]]
CameraDict = Dict[str, Optional[CameraClientType]]
ModelDict = Dict[str, Optional[YoloModelType]]
CalibrationDict = Dict[str, Optional[np.ndarray]]


class DualArmController:
    def __init__(self, config_path: str = config.CONFIG_FILE):
        self.config_path: str = config_path
        self.config: Optional[Dict] = None  # Will be populated by load_and_set_config_variables
        self.ui_manager: UIManager = UIManager(self)

        # Initialize attributes with Python-level defaults from config.py
        # These will be overridden by load_and_set_config_variables if present in YAML
        self.font_path: Optional[str] = config.DEFAULT_FONT_PATH
        self.left_robot_ip: str = config.DEFAULT_IP
        self.right_robot_ip: str = config.DEFAULT_IP
        self.left_gripper_id: int = config.DEFAULT_GRIPPER_ID
        self.right_gripper_id: int = config.DEFAULT_GRIPPER_ID
        self.window_width: int = config.DEFAULT_WINDOW_WIDTH
        self.window_height: int = config.DEFAULT_WINDOW_HEIGHT
        self.font_size: int = config.DEFAULT_FONT_SIZE
        self.current_speed_xy: float = config.DEFAULT_XY_SPEED
        self.current_speed_z: float = config.DEFAULT_Z_SPEED
        self.rpy_speed: float = config.DEFAULT_RPY_SPEED  # For RPY jogging
        self.reset_speed: float = config.DEFAULT_RESET_SPEED  # For original joint resets
        self.min_speed: float = config.DEFAULT_MIN_SPEED
        self.max_speed: float = config.DEFAULT_MAX_SPEED
        self.speed_increment: float = config.DEFAULT_SPEED_INCREMENT
        self.acc: int = config.DEFAULT_ACC
        self.arot: int = config.DEFAULT_AROT
        self.t_interval: float = config.DEFAULT_T
        self.trigger_threshold: float = config.DEFAULT_TRIGGER_THRESHOLD
        self.gripper_speed: int = config.DEFAULT_GRIPPER_SPEED
        self.gripper_force: int = config.DEFAULT_GRIPPER_FORCE
        self.long_press_duration: float = config.DEFAULT_LONG_PRESS_DURATION

        # RPY Reset specific parameters - initial defaults, will be updated from YAML settings
        self.reset_rpy_speed: float = 30.0  # Default RPY reset speed
        self.reset_rpy_acc: float = 50.0  # Default RPY reset acceleration
        self.reset_rpy_arot: float = 20.0  # Default RPY reset angular acceleration (if used)
        self.reset_rpy_t_interval: float = 0.1  # Default RPY reset time interval (if used)

        # Robot Controllers & Status
        self.controller_left: Optional[CPSClient] = None
        self.controller_right: Optional[CPSClient] = None
        self.left_init_ok: bool = False
        self.right_init_ok: bool = False
        self.left_gripper_active: bool = False
        self.right_gripper_active: bool = False
        self.left_gripper_open: bool = True
        self.right_gripper_open: bool = True

        self.cameras: CameraDict = {}
        self.models: ModelDict = {}
        self.calibration: CalibrationDict = {}

        self.running: bool = False
        self.control_modes: List[str] = [config.MODE_XYZ, config.MODE_RPY, config.MODE_RESET, config.MODE_VISION]
        self.current_mode_index: int = 0
        self.control_mode: str = self.control_modes[0]
        self.mode_button_press_time: Optional[float] = None
        self.status_message: str = "初始化中..."

        self.controls_map: Dict = {}  # Will be populated by load_and_set_config_variables
        self.audio_files_config: Dict = {}  # Will be populated
        self.reset_rpy_poses_config: Dict[str, List[float]] = {}  # Will be populated

        self._update_control_attributes()  # Initialize control attributes (they will use defaults set above for now)

    def _update_control_attributes(self):
        """从 self.controls_map 更新特定的控制属性。
           此时 self.controls_map 可能为空或已从配置加载。
        """
        # These specific control attributes are already set by config.load_and_set_config_variables
        # However, this method also loads the RPY specific ones which might not be in that function.
        # It's safer to ensure controls_map is used if populated.

        # General controls (likely set by load_and_set_config_variables already)
        self.mode_switch_control = self.controls_map.get('mode_switch_button',
                                                         {'type': 'button', 'index': config.DEFAULT_MODE_SWITCH_BUTTON})
        self.speed_inc_control = self.controls_map.get('speed_increase_alt',
                                                       {'type': 'button', 'index': config.DEFAULT_SPEED_INC_BUTTON})
        self.speed_dec_control = self.controls_map.get('speed_decrease',
                                                       {'type': 'button', 'index': config.DEFAULT_SPEED_DEC_BUTTON})
        self.gripper_toggle_left_ctrl = self.controls_map.get('gripper_toggle_left', {'type': 'button',
                                                                                      'index': config.DEFAULT_GRIPPER_L_BUTTON})
        self.gripper_toggle_right_ctrl = self.controls_map.get('gripper_toggle_right', {'type': 'button',
                                                                                        'index': config.DEFAULT_GRIPPER_R_BUTTON})
        self.reset_left_arm_ctrl = self.controls_map.get('reset_left_arm')  # Original reset
        self.reset_right_arm_ctrl = self.controls_map.get('reset_right_arm')  # Original reset

        # RPY Reset Button Configurations (from self.controls_map, which should be filled by load_and_set_config_variables)
        self.reset_left_arm_default_rpy_ctrl = self.controls_map.get('reset_left_arm_default_rpy')
        self.reset_left_arm_forward_rpy_ctrl = self.controls_map.get('reset_left_arm_forward_rpy')
        self.reset_left_arm_backward_rpy_ctrl = self.controls_map.get('reset_left_arm_backward_rpy')
        self.reset_left_arm_to_left_rpy_ctrl = self.controls_map.get('reset_left_arm_to_left_rpy')
        self.reset_left_arm_to_right_rpy_ctrl = self.controls_map.get('reset_left_arm_to_right_rpy')
        self.reset_left_arm_up_rpy_ctrl = self.controls_map.get('reset_left_arm_up_rpy')
        self.reset_left_arm_down_rpy_ctrl = self.controls_map.get('reset_left_arm_down_rpy')

        self.reset_right_arm_default_rpy_ctrl = self.controls_map.get('reset_right_arm_default_rpy')
        self.reset_right_arm_forward_rpy_ctrl = self.controls_map.get('reset_right_arm_forward_rpy')
        self.reset_right_arm_backward_rpy_ctrl = self.controls_map.get('reset_right_arm_backward_rpy')
        self.reset_right_arm_to_left_rpy_ctrl = self.controls_map.get('reset_right_arm_to_left_rpy')
        self.reset_right_arm_to_right_rpy_ctrl = self.controls_map.get('reset_right_arm_to_right_rpy')
        self.reset_right_arm_up_rpy_ctrl = self.controls_map.get('reset_right_arm_up_rpy')
        self.reset_right_arm_down_rpy_ctrl = self.controls_map.get('reset_right_arm_down_rpy')

    def setup(self) -> bool:
        print("=" * 10 + " 开始设置双臂控制器 " + "=" * 10)

        print("\n[Setup 1/5] 加载主配置文件...")
        try:
            # config.load_and_set_config_variables (from your config.py)
            # - Loads YAML into self.config
            # - Sets attributes on self (e.g., self.left_robot_ip, self.current_speed_xy)
            #   using YAML values or config.DEFAULT_... fallbacks.
            if not config.load_and_set_config_variables(self, self.config_path):
                self.status_message = "错误: 加载主配置失败"
                print(f"  {self.status_message}")
                return False
            # self.config should now be populated, and attributes like self.left_robot_ip should have correct values.
            print("  主配置文件已通过 config.py 加载并设置变量。")

            # Explicitly load RPY reset specific motion parameters from self.config (YAML settings)
            # This ensures they override the initial Python defaults if specified in YAML.
            # config.load_and_set_config_variables in your config.py does *not* yet load these RPY specific ones.
            if self.config and 'settings' in self.config:
                settings_cfg = self.config.get('settings', {})
                self.reset_rpy_speed = float(settings_cfg.get('reset_rpy_speed', self.reset_rpy_speed))
                self.reset_rpy_acc = float(settings_cfg.get('reset_rpy_acc', self.reset_rpy_acc))
                self.reset_rpy_arot = float(settings_cfg.get('reset_rpy_arot', self.reset_rpy_arot))
                self.reset_rpy_t_interval = float(settings_cfg.get('reset_rpy_t_interval', self.reset_rpy_t_interval))
                print(f"  RPY 重置参数已从 settings 更新: speed={self.reset_rpy_speed}, acc={self.reset_rpy_acc}")

            # _update_control_attributes is called to ensure specific control dicts (like reset_left_arm_default_rpy_ctrl)
            # are populated from self.controls_map, which was filled by load_and_set_config_variables.
            self._update_control_attributes()
            print("  控制属性已更新。")

            # Load RPY reset poses from self.config (populated by load_and_set_config_variables)
            if isinstance(self.config.get('reset_rpy_poses'), dict):
                self.reset_rpy_poses_config = self.config['reset_rpy_poses']
                print(f"  已加载自定义RPY回正姿态: {list(self.reset_rpy_poses_config.keys())}")
            else:
                print("  警告: 配置文件中 'reset_rpy_poses' 未找到或格式不正确。")

        except Exception as e_cfg:
            self.status_message = f"错误: 加载配置阶段异常: {e_cfg}"
            print(f"  {self.status_message}")
            traceback.print_exc()
            return False

        # Print effective IP addresses before robot initialization for debugging
        print(f"  [调试] 左臂IP将使用: {self.left_robot_ip}")
        print(f"  [调试] 右臂IP将使用: {self.right_robot_ip}")
        print(f"  [调试] 字体路径将使用: {self.font_path}")
        print(f"  [调试] XYZ 速度: {self.current_speed_xy}")

        print("\n[Setup 2/5] 初始化 Pygame 和 UI...")
        try:
            if not self.ui_manager.init_pygame():  # init_pygame uses self.controller.font_path etc.
                self.status_message = self.ui_manager.status_message
                print(f"  错误: Pygame 初始化失败 - {self.status_message}")
                return False
            self.status_message = self.ui_manager.status_message
            print("  Pygame 和 UI 初始化成功.")
        except Exception as e_pygame:
            self.status_message = f"错误: Pygame 初始化异常: {e_pygame}"
            print(f"  {self.status_message}")
            traceback.print_exc()
            return False

        print("\n[Setup 3/5] 初始化视觉组件...")
        vision_ok = self._initialize_vision_components()
        print(f"  视觉组件初始化完成 (状态: {'OK' if vision_ok else '存在问题'}).")

        print("\n[Setup 4/5] 初始化机器人硬件...")
        robots_ok = self._init_robots()  # This uses self.left_robot_ip, self.right_robot_ip
        if not robots_ok:
            print("  警告: 机器人硬件初始化失败!")
        else:
            print("  机器人硬件初始化成功.")

        print("\n[Setup 5/5] 完成设置.")
        if robots_ok and vision_ok:
            self.status_message = "系统就绪"
            self.ui_manager.play_sound('system_ready')
        elif robots_ok and not vision_ok:
            self.status_message = self._append_status("机器人OK, 但视觉组件失败")
            # self.ui_manager.play_sound('system_ready_partial')
        else:
            self.status_message = self._append_status("错误: 机器人初始化失败")
            # self.ui_manager.play_sound('system_init_error')

        self.ui_manager.update_status_message(self.status_message)
        self.running = True
        print(f"\n==== 控制器设置完成 (最终状态: {self.status_message}) ====")
        return True

    def _initialize_vision_components(self) -> bool:
        # This method relies on self.config being populated by load_and_set_config_variables
        all_vision_ok = True
        setup_cfg = self.config.get('setup', {})
        if not isinstance(setup_cfg, dict):
            print("  错误: 'setup' 配置部分无效或缺失 (在 _initialize_vision_components)。")
            return False

        print("  初始化相机...")
        if CAMERA_AVAILABLE:
            camera_serials = setup_cfg.get('camera_serials', {})  # From YAML via self.config
            if not camera_serials or not isinstance(camera_serials, dict):
                print("    警告: 未配置 'camera_serials' 或配置无效。")
                all_vision_ok = False
            else:
                failed_cams_details = []
                for cam_name, serial in camera_serials.items():
                    print(f"    尝试 '{cam_name}' (SN: {serial})...")
                    try:
                        cam_client = initialize_connected_cameras(serial)
                        if cam_client:
                            required = ['get_frames', 'get_depth_for_color_pixel', 'rgb_fx', 'rgb_fy', 'ppx', 'ppy']
                            missing = [a for a in required if not hasattr(cam_client, a)]
                            if not missing:
                                self.cameras[cam_name] = cam_client
                                print(f"    -> '{cam_name}' OK.")
                            else:
                                print(f"    -> 错误: '{cam_name}' 缺少接口: {missing}.")
                                failed_cams_details.append(f"{cam_name}(接口)")
                                if hasattr(cam_client, 'close'):
                                    try:
                                        cam_client.close()
                                    except:
                                        pass
                        else:
                            print(f"    -> 错误: '{cam_name}' 初始化返回 None.")
                            failed_cams_details.append(f"{cam_name}(None)")
                    except Exception as e:
                        print(f"    -> 错误: '{cam_name}' 初始化异常: {e}")
                        failed_cams_details.append(f"{cam_name}(异常)")
                if failed_cams_details:
                    self.status_message = self._append_status(f"警告: 相机失败({len(failed_cams_details)})")
                    all_vision_ok = False
        else:
            print("    跳过相机初始化 (库不可用).")
            self.status_message = self._append_status("警告: 相机库不可用")
            all_vision_ok = False

        print("\n  加载YOLO模型...")
        if YOLO_AVAILABLE:
            yolo_models_cfg = setup_cfg.get('yolo_models', {})  # From YAML via self.config
            if not yolo_models_cfg:
                print("    警告: 未配置 'yolo_models'。")
                all_vision_ok = False
            else:
                loaded_model_count = 0
                for name, path in yolo_models_cfg.items():
                    print(f"    加载 '{name}' from {path}...")
                    if path and isinstance(path, str) and os.path.exists(path):
                        try:
                            self.models[name] = YOLO(path)
                            print(f"    -> '{name}' OK.")
                            loaded_model_count += 1
                        except Exception as e:
                            print(f"    -> 错误: 加载 '{name}' 异常: {e}")
                    else:
                        print(f"    -> 错误: 路径 '{path}' 无效.")
                if loaded_model_count == 0 and yolo_models_cfg:
                    print("    警告: 未能加载任何配置的YOLO模型。")
                    self.status_message = self._append_status("警告: 模型加载失败")
                    all_vision_ok = False
        else:
            print("    跳过YOLO模型加载 (库不可用).")
            self.status_message = self._append_status("警告: YOLO库不可用")
            all_vision_ok = False

        print("\n  加载标定数据...")
        if not SCIPY_AVAILABLE:
            print("    跳过标定加载 (scipy库不可用).")
            self.status_message = self._append_status("警告: scipy库不可用")
            all_vision_ok = False
        else:
            calibration_files = setup_cfg.get('calibration_files', {})  # From YAML via self.config
            if not calibration_files or not isinstance(calibration_files, dict) or not (
                    'left' in calibration_files and 'right' in calibration_files):
                print("    警告: 未配置或配置不全 'calibration_files' (需要'left'和'right').")
                self.status_message = self._append_status("警告: 标定配置不完整")
                all_vision_ok = False
            else:
                loaded_calib_count = 0
                for arm_name, path in calibration_files.items():
                    if arm_name not in ['left', 'right']: continue
                    print(f"    加载 {arm_name} 臂标定: {path}...")
                    if path and isinstance(path, str) and os.path.exists(path):
                        try:
                            with open(path, 'r') as f:
                                calib_yaml = yaml.safe_load(f)
                            matrix = calib_yaml.get('hand_eye_transformation_matrix')
                            matrix_np = np.array(matrix, dtype=float)
                            if matrix_np.shape == (4, 4):
                                self.calibration[arm_name] = matrix_np
                                print(f"    -> {arm_name} 臂标定 OK.")
                                loaded_calib_count += 1
                            else:
                                print(f"    -> 错误: 文件 '{path}' 中矩阵结构无效 ({matrix_np.shape}).")
                        except Exception as e:
                            print(f"    -> 错误: 加载或解析标定 '{path}' 失败: {e}")
                    else:
                        print(f"    -> 错误: 标定文件路径无效 '{path}'.")
                if loaded_calib_count < 2 and (
                        'left' in calibration_files or 'right' in calibration_files):  # Check if at least one was configured but failed
                    print("    警告: 未能加载所有必需的臂标定矩阵。")
                    self.status_message = self._append_status("警告: 标定加载不完整")
                    all_vision_ok = False
        return all_vision_ok

    def _init_robots(self) -> bool:
        print("\n[Robot Init] 开始连接和初始化机器人...")
        self.left_init_ok, self.right_init_ok = False, False
        self.left_gripper_active, self.right_gripper_active = False, False
        all_ok = True
        # IPs are now from self.left_robot_ip, set by __init__ and then load_and_set_config_variables
        try:
            print(f"  连接左臂 ({self.left_robot_ip})...")  # Uses the potentially YAML-loaded IP
            self.controller_left = CPSClient(self.left_robot_ip, gripper_slave_id=self.left_gripper_id)
            if self.controller_left.connect():
                print("  左臂连接成功.")
                self.left_init_ok = initialize_robot(self.controller_left, "左臂")
                if self.left_init_ok: self.left_gripper_active = connect_arm_gripper(self.controller_left, "左臂")
                all_ok &= self.left_init_ok
            else:
                print("  错误: 左臂连接失败"); all_ok = False
        except Exception as e:
            print(f"  左臂初始化异常: {e}"); traceback.print_exc(); all_ok = False

        try:
            print(f"  连接右臂 ({self.right_robot_ip})...")  # Uses the potentially YAML-loaded IP
            self.controller_right = CPSClient(self.right_robot_ip, gripper_slave_id=self.right_gripper_id)
            if self.controller_right.connect():
                print("  右臂连接成功.")
                self.right_init_ok = initialize_robot(self.controller_right, "右臂")
                if self.right_init_ok: self.right_gripper_active = connect_arm_gripper(self.controller_right, "右臂")
                all_ok &= self.right_init_ok
            else:
                print("  错误: 右臂连接失败"); all_ok = False
        except Exception as e:
            print(f"  右臂初始化异常: {e}"); traceback.print_exc(); all_ok = False

        if not all_ok: self.status_message = self._append_status("警告: 机器人初始化失败!")
        print("[Robot Init] 机器人初始化流程结束。")
        return all_ok

    def _append_status(self, new_status_part: str) -> str:
        if self.status_message and ("警告" in self.status_message or "错误" in self.status_message):
            if new_status_part not in self.status_message: return f"{self.status_message} | {new_status_part}"
            return self.status_message
        return new_status_part

    def _threaded_process_vision_audio(self, audio_filepath: str):
        current_thread = threading.current_thread()
        thread_name = current_thread.name
        print(f"\n[{thread_name}] 开始处理语音: {audio_filepath}")
        self.ui_manager.update_status_message("正在发送语音指令...")
        received_audio_path, command_json = vision_interaction.send_audio_and_receive_response(audio_filepath)
        if os.path.exists(audio_filepath):
            try:
                os.remove(audio_filepath)
            except OSError as e:
                print(f"[{thread_name}] 清理发送文件时出错: {e}")
        if received_audio_path:
            print(f"[{thread_name}] 收到音频回复, 准备播放...")
            self.ui_manager.update_status_message("正在播放回复...")
            self.ui_manager.play_sound('server_response_received')  # Make sure this sound is in YAML
            vision_interaction.play_audio_file(received_audio_path)
            if os.path.exists(received_audio_path):
                try:
                    os.remove(received_audio_path)
                except OSError as e:
                    print(f"[{thread_name}] 清理接收文件时出错: {e}")
        else:
            print(f"[{thread_name}] 未收到音频回复。")

        final_status = "操作完成"
        action_success = False
        if command_json and isinstance(command_json, dict) and "error" not in command_json:
            print(f"[{thread_name}] 收到有效JSON指令: {command_json}")
            action = command_json.get('action', '未知')
            self.ui_manager.update_status_message(f"处理指令: {action}...")
            robot_controllers_dict: ControllerDict = {
                'left': self.controller_left if self.left_init_ok else None,
                'right': self.controller_right if self.right_init_ok else None}
            robot_controllers_dict = {k: v for k, v in robot_controllers_dict.items() if v}
            camera_clients_dict: CameraDict = self.cameras
            models_dict: ModelDict = self.models
            calibration_dict: CalibrationDict = self.calibration

            if action == "grasp":
                target_details = command_json.get("target", {})
                arm_choice = target_details.get("arm_choice");
                object_id = target_details.get("id")
                required_cam_key = f"{arm_choice}_hand" if arm_choice else None
                required_model_key = object_id
                checks_passed = True
                if not robot_controllers_dict.get(arm_choice):
                    final_status = f"错误: {arm_choice}臂未就绪"; checks_passed = False
                elif not camera_clients_dict.get(required_cam_key):
                    final_status = f"错误: {required_cam_key}相机未就绪"; checks_passed = False
                elif not models_dict.get(required_model_key) and not models_dict.get('default'):
                    final_status = f"错误: 无'{required_model_key}'或默认模型"; checks_passed = False
                elif calibration_dict.get(arm_choice) is None:
                    final_status = f"错误: {arm_choice}臂标定缺失"; checks_passed = False

                if checks_passed:
                    try:
                        print(f"[{thread_name}] 调用 vision_interaction.handle_command_json (grasp)...")
                        action_success = vision_interaction.handle_command_json(command_json, robot_controllers_dict,
                                                                                camera_clients_dict, models_dict,
                                                                                calibration_dict)
                        final_status = f"抓取指令 " + ("成功" if action_success else "失败")
                        self.ui_manager.play_sound(
                            'action_success_general' if action_success else 'action_fail_general')
                    except Exception as e_handle:
                        print(
                            f"[{thread_name}] 调用 handle_command_json (grasp) 时发生错误: {e_handle}"); traceback.print_exc(); final_status = f"错误: 处理抓取指令失败"; self.ui_manager.play_sound(
                            'action_fail_general')
                else:
                    print(f"[{thread_name}] 抓取指令前置检查失败: {final_status}"); self.ui_manager.play_sound(
                        'action_fail_general')
            else:  # Handle other non-grasp actions
                try:
                    print(f"[{thread_name}] 调用 vision_interaction.handle_command_json ({action})...")
                    vision_interaction.handle_command_json(command_json, robot_controllers_dict, camera_clients_dict,
                                                           models_dict, calibration_dict)
                    final_status = f"指令 '{action}' 已处理";
                    action_success = True;
                    self.ui_manager.play_sound('action_success_general')
                except Exception as e_handle_other:
                    print(
                        f"[{thread_name}] 处理指令 '{action}' 时发生错误: {e_handle_other}"); traceback.print_exc(); final_status = f"错误: 处理 '{action}' 指令失败"; self.ui_manager.play_sound(
                        'action_fail_general')
        elif command_json and "error" in command_json:
            final_status = f"错误: 服务器返回JSON错误 ({command_json['error']})"; self.ui_manager.play_sound(
                'action_fail_general')
        else:
            final_status = "未收到有效指令或JSON无效"

        start_rec_cfg = self.controls_map.get('vision_start_record')  # self.controls_map should be filled
        start_rec_btn_idx = start_rec_cfg.get('index', '?') if start_rec_cfg else '?'
        self.ui_manager.update_status_message(f"{final_status}. 按 B{start_rec_btn_idx} 开始新指令.")
        print(f"[{thread_name}] 视觉交互处理完毕。")

    def switch_control_mode(self):
        self.stop_all_movement()
        old_mode = self.control_mode
        self.current_mode_index = (self.current_mode_index + 1) % len(self.control_modes)
        self.control_mode = self.control_modes[self.current_mode_index]
        print(f"切换模式: {old_mode} -> {self.control_mode}")
        sound_event, status_msg = None, f"模式: {self.control_mode}"
        if self.control_mode == config.MODE_XYZ:
            sound_event = 'xyz_mode'
        elif self.control_mode == config.MODE_RPY:
            sound_event = 'rpy_mode'
        elif self.control_mode == config.MODE_RESET:
            sound_event = 'reset_mode'
        elif self.control_mode == config.MODE_VISION:
            sound_event = 'vision_enter'
            start_rec_cfg = self.controls_map.get('vision_start_record')
            start_rec_btn_idx = start_rec_cfg.get('index', '?') if start_rec_cfg else '?'
            status_msg = f"视觉模式: 按 B{start_rec_btn_idx} 开始语音"
        if sound_event: self.ui_manager.play_sound(sound_event)
        self.ui_manager.update_status_message(status_msg)
        if self.ui_manager.screen: pygame.display.set_caption(f"双臂控制 (模式: {self.control_mode})")
        if old_mode == config.MODE_VISION and vision_interaction.is_recording:
            print("切换模式时取消进行中的录音...")
            if vision_interaction.cancel_recording(): self.ui_manager.play_sound('vision_record_cancel')

    def stop_all_movement(self):
        print("发送停止所有运动指令...")
        stop_payload = [0.0] * 6
        stop_acc, stop_arot, stop_t = 200, 20, 0.05  # Consider making these configurable
        try:
            if self.left_init_ok and self.controller_left: self.controller_left.moveBySpeedl(stop_payload, stop_acc,
                                                                                             stop_arot, stop_t)
            if self.right_init_ok and self.controller_right: self.controller_right.moveBySpeedl(stop_payload, stop_acc,
                                                                                                stop_arot, stop_t)
            time.sleep(0.1)
        except Exception as e:
            print(f"发送停止指令时出错: {e}")

    def toggle_gripper(self, side: str):
        controller = self.controller_left if side == 'left' else self.controller_right
        is_open = self.left_gripper_open if side == 'left' else self.right_gripper_open
        is_active = self.left_gripper_active if side == 'left' else self.right_gripper_active
        if not controller or not is_active:
            self.ui_manager.play_sound('gripper_inactive')
            msg = f"{side} 夹爪无效或未初始化";
            print(msg);
            self.ui_manager.update_status_message(msg)
            return
        action = controller.close_gripper if is_open else controller.open_gripper
        sound_event = ('left_close' if is_open else 'left_open') if side == 'left' else \
            ('right_close' if is_open else 'right_open')
        new_state_str = "关闭" if is_open else "打开"
        print(f"切换 {side} 夹爪为: {new_state_str}")
        self.ui_manager.play_sound(sound_event)
        try:
            action(speed=self.gripper_speed, force=self.gripper_force, wait=False)
            if side == 'left':
                self.left_gripper_open = not is_open
            else:
                self.right_gripper_open = not is_open
            self.ui_manager.update_status_message(f"{side} 夹爪: {new_state_str}")
        except Exception as e:
            print(f"切换 {side} 夹爪出错: {e}")
            self.ui_manager.play_sound('action_fail_general')
            self.ui_manager.update_status_message(f"{side} 夹爪切换失败")

    # Original attempt_reset_arm methods (for full joint/pose resets if still needed)
    def attempt_reset_left_arm(self):
        if not self.left_init_ok or not self.controller_left:  # Added controller_left check
            msg = "左臂回正失败: 未初始化或控制器无效";
            print(msg)
            self.ui_manager.play_sound('left_reset_fail');
            self.ui_manager.update_status_message(msg)
            return
        self.ui_manager.update_status_message("左臂回正中...")
        success = attempt_reset_arm(self.controller_left, "左臂", config.TARGET_RESET_RPY_LEFT,
                                    self.reset_speed, desire_left_pose, 'move_robot',  # Ensure 'move_robot' is correct
                                    self.ui_manager.play_sound, 'left_reset_success', 'left_reset_fail')
        self.ui_manager.update_status_message("左臂回正成功" if success else "左臂回正失败")

    def attempt_reset_right_arm(self):
        if not self.right_init_ok or not self.controller_right:  # Added controller_right check
            msg = "右臂回正失败: 未初始化或控制器无效";
            print(msg)
            self.ui_manager.play_sound('right_reset_fail');
            self.ui_manager.update_status_message(msg)
            return
        self.ui_manager.update_status_message("右臂回正中...")
        success = attempt_reset_arm(self.controller_right, "右臂", config.TARGET_RESET_RPY_RIGHT,
                                    self.reset_speed, desire_right_pose, 'move_right_robot',
                                    # Ensure 'move_right_robot' is correct
                                    self.ui_manager.play_sound, 'right_reset_success', 'right_reset_fail')
        self.ui_manager.update_status_message("右臂回正成功" if success else "右臂回正失败")

    def _attempt_reset_rpy_orientation(self, arm_side: str, pose_key_in_yaml: str,
                                       action_description: str,
                                       success_sound_key: Optional[str] = None,
                                       fail_sound_key: Optional[str] = None):

        robot_controller: Optional[CPSClient] = self.controller_left if arm_side == 'left' else self.controller_right
        is_init_ok = self.left_init_ok if arm_side == 'left' else self.right_init_ok

        final_success_sound = success_sound_key if success_sound_key else 'action_success_general'
        final_fail_sound = fail_sound_key if fail_sound_key else 'action_fail_general'

        if not is_init_ok or not robot_controller:
            msg = f"{action_description} RPY设置失败: 机器人未初始化或控制器无效"
            print(msg)
            if self.ui_manager: self.ui_manager.play_sound(final_fail_sound)
            if self.ui_manager: self.ui_manager.update_status_message(msg)
            return

        target_rpy_array = self.reset_rpy_poses_config.get(pose_key_in_yaml)
        if target_rpy_array is None or not isinstance(target_rpy_array, list) or len(target_rpy_array) != 3:
            msg = f"{action_description} RPY设置失败: '{pose_key_in_yaml}' RPY姿态定义无效或缺失"
            print(msg)
            if self.ui_manager: self.ui_manager.play_sound(final_fail_sound)
            if self.ui_manager: self.ui_manager.update_status_message(msg)
            return

        self.ui_manager.update_status_message(f"{action_description} RPY设置中 ({target_rpy_array})...")

        # 根据手臂选择对应的 desire_pose_func 和 move_func_name
        desire_function_for_arm = desire_left_pose if arm_side == 'left' else desire_right_pose

        # 使用您指定的特定方法名
        if arm_side == 'left':
            robot_move_method_name = 'move_robot'
        else:  # arm_side == 'right'
            robot_move_method_name = 'move_right_robot'

        arm_name_for_function = arm_side.capitalize() + "臂"

        print(
            f"  调用 attempt_reset_arm 进行RPY设置: arm={arm_name_for_function}, target_rpy={target_rpy_array}, speed={self.reset_rpy_speed}, move_func='{robot_move_method_name}'")

        # 调用您提供的、可工作的 attempt_reset_arm 函数
        # 注意：如果 attempt_reset_arm 需要 acc 参数，您需要在这里传递 self.reset_rpy_acc
        # 并确保 attempt_reset_arm 函数定义也接受并使用它。
        # 假设当前 attempt_reset_arm 的签名与您之前提供的一致（即不强制要求 acc）。
        success = attempt_reset_arm(
            controller=robot_controller,
            arm_name=arm_name_for_function,
            target_rpy=target_rpy_array,
            reset_speed=self.reset_rpy_speed,  # 使用为RPY设置的速度
            desire_pose_func=desire_function_for_arm,
            move_func_name=robot_move_method_name,  # 使用 'move_left_robot' 或 'move_right_robot'
            sound_player=self.ui_manager.play_sound if self.ui_manager else None,
            success_sound=final_success_sound,  # 正确传递处理后的声音key
            fail_sound=final_fail_sound  # 正确传递处理后的声音key
        )

        if success:
            self.ui_manager.update_status_message(f"{action_description} RPY设置成功")
        else:
            # attempt_reset_arm 内部会打印失败信息，这里只更新UI
            self.ui_manager.update_status_message(f"{action_description} RPY设置失败 (详情请查看控制台)")

    # ... (所有 attempt_reset_left/right_arm_..._rpy 方法保持不变) ...
    def attempt_reset_left_arm_default_rpy(self):
        self._attempt_reset_rpy_orientation(
            arm_side='left',
            pose_key_in_yaml='left_default',
            action_description="左臂默认RPY",
            success_sound_key='left_reset_success',  # 统一左臂成功语音
            fail_sound_key='left_reset_fail'  # 统一左臂失败语音
        )

    def attempt_reset_left_arm_forward_rpy(self):
        self._attempt_reset_rpy_orientation(
            arm_side='left',
            pose_key_in_yaml='left_forward',
            action_description="左臂向前RPY",
            success_sound_key='left_reset_success',
            fail_sound_key='left_reset_fail'
        )

    def attempt_reset_left_arm_backward_rpy(self):
        self._attempt_reset_rpy_orientation(
            arm_side='left',
            pose_key_in_yaml='left_backward',
            action_description="左臂向后RPY",
            success_sound_key='left_reset_success',
            fail_sound_key='left_reset_fail'
        )

    def attempt_reset_left_arm_to_left_rpy(self):
        self._attempt_reset_rpy_orientation(
            arm_side='left',
            pose_key_in_yaml='left_to_left',
            action_description="左臂向左RPY",
            success_sound_key='left_reset_success',
            fail_sound_key='left_reset_fail'
        )

    def attempt_reset_left_arm_to_right_rpy(self):
        self._attempt_reset_rpy_orientation(
            arm_side='left',
            pose_key_in_yaml='left_to_right',
            action_description="左臂向右RPY",
            success_sound_key='left_reset_success',
            fail_sound_key='left_reset_fail'
        )

    def attempt_reset_left_arm_up_rpy(self):
        self._attempt_reset_rpy_orientation(
            arm_side='left',
            pose_key_in_yaml='left_up',
            action_description="左臂向上RPY",
            success_sound_key='left_reset_success',
            fail_sound_key='left_reset_fail'
        )

    def attempt_reset_left_arm_down_rpy(self):
        self._attempt_reset_rpy_orientation(
            arm_side='left',
            pose_key_in_yaml='left_down',
            action_description="左臂向下RPY",
            success_sound_key='left_reset_success',
            fail_sound_key='left_reset_fail'
        )

    # --- 右臂 ---
    def attempt_reset_right_arm_default_rpy(self):
        self._attempt_reset_rpy_orientation(
            arm_side='right',
            pose_key_in_yaml='right_default',
            action_description="右臂默认RPY",
            success_sound_key='right_reset_success',  # 统一右臂成功语音
            fail_sound_key='right_reset_fail'  # 统一右臂失败语音
        )

    def attempt_reset_right_arm_forward_rpy(self):
        self._attempt_reset_rpy_orientation(
            arm_side='right',
            pose_key_in_yaml='right_forward',
            action_description="右臂向前RPY",
            success_sound_key='right_reset_success',
            fail_sound_key='right_reset_fail'
        )

    def attempt_reset_right_arm_backward_rpy(self):
        self._attempt_reset_rpy_orientation(
            arm_side='right',
            pose_key_in_yaml='right_backward',
            action_description="右臂向后RPY",
            success_sound_key='right_reset_success',
            fail_sound_key='right_reset_fail'
        )

    def attempt_reset_right_arm_to_left_rpy(self):
        self._attempt_reset_rpy_orientation(
            arm_side='right',
            pose_key_in_yaml='right_to_left',
            action_description="右臂向左RPY",
            success_sound_key='right_reset_success',
            fail_sound_key='right_reset_fail'
        )

    def attempt_reset_right_arm_to_right_rpy(self):
        self._attempt_reset_rpy_orientation(
            arm_side='right',
            pose_key_in_yaml='right_to_right',
            action_description="右臂向右RPY",
            success_sound_key='right_reset_success',
            fail_sound_key='right_reset_fail'
        )

    def attempt_reset_right_arm_up_rpy(self):
        self._attempt_reset_rpy_orientation(
            arm_side='right',
            pose_key_in_yaml='right_up',
            action_description="右臂向上RPY",
            success_sound_key='right_reset_success',
            fail_sound_key='right_reset_fail'
        )

    def attempt_reset_right_arm_down_rpy(self):
        self._attempt_reset_rpy_orientation(
            arm_side='right',
            pose_key_in_yaml='right_down',
            action_description="右臂向下RPY",
            success_sound_key='right_reset_success',
            fail_sound_key='right_reset_fail'
        )
    def _calculate_speed_commands(self) -> Tuple[np.ndarray, np.ndarray]:
        speed_left_cmd, speed_right_cmd = np.zeros(6), np.zeros(6)
        if not self.ui_manager.joystick: return speed_left_cmd, speed_right_cmd
        current_mode_prefix = self.control_mode.lower() + "_"
        if self.control_mode in [config.MODE_XYZ, config.MODE_RPY]:
            for action, control_cfg in self.controls_map.items():  # self.controls_map is from YAML via load_and_set_config_variables
                if not isinstance(control_cfg, dict): continue
                if action.startswith(current_mode_prefix) and ('_arm' in action):
                    if self.ui_manager.get_joystick_input_state(
                            control_cfg):  # get_joystick_input_state uses self.trigger_threshold
                        target_array = None
                        if 'left_arm' in action:
                            target_array = speed_left_cmd
                        elif 'right_arm' in action:
                            target_array = speed_right_cmd
                        else:
                            continue
                        axis_idx, base_speed = -1, 0.0
                        direction = 1.0 if '_pos' in action else -1.0 if '_neg' in action else 1.0
                        if self.control_mode == config.MODE_XYZ:
                            if '_x' in action:
                                axis_idx, base_speed = 0, self.current_speed_xy
                            elif '_y' in action:
                                axis_idx, base_speed = 1, self.current_speed_xy
                            elif '_z' in action:
                                axis_idx, base_speed = 2, self.current_speed_z
                        elif self.control_mode == config.MODE_RPY:  # RPY Jogging
                            if '_roll' in action:
                                axis_idx, base_speed = 3, self.rpy_speed
                            elif '_pitch' in action:
                                axis_idx, base_speed = 4, self.rpy_speed
                            elif '_yaw' in action:
                                axis_idx, base_speed = 5, self.rpy_speed
                        if axis_idx != -1:
                            if control_cfg.get('type') == 'axis':
                                try:
                                    axis_val = self.ui_manager.joystick.get_axis(control_cfg.get('index', -1))
                                    target_array[axis_idx] = base_speed * abs(axis_val) * direction
                                except (pygame.error, IndexError):
                                    target_array[axis_idx] = 0
                            else:
                                target_array[axis_idx] = base_speed * direction
        return speed_left_cmd, speed_right_cmd

    def _apply_transformations(self, speed_left_cmd: np.ndarray, speed_right_cmd: np.ndarray) -> Tuple[
        np.ndarray, np.ndarray]:
        if R is None: return speed_left_cmd, speed_right_cmd
        speed_left_final, speed_right_final = np.zeros(6), np.zeros(6)
        if self.control_mode == config.MODE_XYZ:
            speed_left_final, speed_right_final = speed_left_cmd.copy(), speed_right_cmd.copy()
            try:
                rot_left = R.from_euler('xyz', [65, 0, 10], degrees=True).as_matrix(); speed_left_final[
                                                                                       :3] = speed_left_cmd[
                                                                                             :3] @ rot_left
            except Exception as e:
                print(f"L Transform Err: {e}"); speed_left_final[:3] = 0
            try:
                rot_right = R.from_euler('xyz', [65.334, -4.208, -9.079], degrees=True).as_matrix(); speed_right_final[
                                                                                                     :3] = speed_right_cmd[
                                                                                                           :3] @ rot_right
            except Exception as e:
                print(f"R Transform Err: {e}"); speed_right_final[:3] = 0
            speed_left_final[3:], speed_right_final[3:] = 0.0, 0.0
        elif self.control_mode == config.MODE_RPY:  # RPY Jogging
            speed_left_final[3:], speed_right_final[3:] = speed_left_cmd[3:], speed_right_cmd[3:]
        return speed_left_final, speed_right_final

    def _send_robot_commands(self, speed_left_final: np.ndarray, speed_right_final: np.ndarray):
        if self.control_mode == config.MODE_XYZ:
            if self.left_init_ok and self.controller_left:
                try:
                    self.controller_left.moveBySpeedl(list(speed_left_final), self.acc, self.arot, self.t_interval)
                except Exception as e:
                    print(f"L Speedl Err: {e}")
            if self.right_init_ok and self.controller_right:
                try:
                    self.controller_right.moveBySpeedl(list(speed_right_final), self.acc, self.arot, self.t_interval)
                except Exception as e:
                    print(f"R Speedl Err: {e}")
        elif self.control_mode == config.MODE_RPY:  # RPY Jogging
            if self.left_init_ok and self.controller_left: send_jog_command(self.controller_left, speed_left_final,
                                                                            self.min_speed, self.max_speed)
            if self.right_init_ok and self.controller_right: send_jog_command(self.controller_right, speed_right_final,
                                                                              self.min_speed, self.max_speed)

    def run_main_loop(self):
        if not self.running: print("错误: 控制器未成功设置，无法启动主循环."); return
        print("\n--- 控制循环开始 (按 ESC 退出) ---")
        while self.running:
            self.ui_manager.handle_events()
            if not self.running: break
            speed_left_cmd, speed_right_cmd = self._calculate_speed_commands()
            speed_left_final, speed_right_final = self._apply_transformations(speed_left_cmd, speed_right_cmd)
            if self.control_mode in [config.MODE_XYZ, config.MODE_RPY]:
                if self.left_init_ok or self.right_init_ok:
                    self._send_robot_commands(speed_left_final, speed_right_final)
            self.ui_manager.status_message = self.status_message  # Ensure UI has the latest controller status
            self.ui_manager.draw_display(speed_left_final, speed_right_final)
            self.ui_manager.clock.tick(30)
        print("--- 控制循环已终止 ---")

    def cleanup(self):
        print("\n" + "=" * 10 + " 开始清理和退出 " + "=" * 10)
        self.running = False
        print("  [Cleanup 1/4] 发送停止运动指令...")
        self.stop_all_movement()
        print("  [Cleanup 2/4] 关闭相机...")
        if not self.cameras:
            print("    无需关闭相机 (未初始化).")
        else:
            for cam_name, cam_client in self.cameras.items():
                if cam_client and hasattr(cam_client, 'close'):
                    try:
                        print(f"    关闭相机 '{cam_name}'..."); cam_client.close()
                    except Exception as e_cam_close:
                        print(f"    关闭相机 '{cam_name}' 时出错: {e_cam_close}")
            self.cameras = {}
        print("  [Cleanup 3/4] 断开机器人连接...")
        controllers_to_disconnect = [("左臂", self.controller_left), ("右臂", self.controller_right)]
        for name, controller_obj in controllers_to_disconnect:
            if controller_obj:
                try:
                    print(f"    断开 {name} 连接..."); controller_obj.disconnect(); print(f"    {name} 已断开。")
                except Exception as disconn_e:
                    print(f"    断开 {name} 连接时出错: {disconn_e}")
            if name == "左臂":
                self.controller_left = None
            else:
                self.controller_right = None
        self.left_init_ok = False;
        self.right_init_ok = False
        print("  [Cleanup 4/4] 关闭 Pygame...")
        self.ui_manager.quit()
        print("\n==== 清理完成，程序退出 ====")