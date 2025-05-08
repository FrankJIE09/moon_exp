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
import pygame  # Needed for pygame.error check at least
from typing import Dict, Optional, Any, List, Tuple  # Type hinting

# --- Vision/Camera/Math Imports ---
# (尝试导入并设置可用性标志)
try:
    # !! 更新为你的实际相机初始化函数路径 !!
    from camera.orbbec_camera import initialize_connected_cameras

    CAMERA_AVAILABLE = True
    # !! 定义你的相机客户端的类型别名 !!
    CameraClientType = Any  # Replace 'Any' with your actual camera client class type if available
except ImportError:
    print("警告: 未找到相机初始化函数 'initialize_connected_cameras'. 视觉功能将受限。")
    CAMERA_AVAILABLE = False
    CameraClientType = None


    def initialize_connected_cameras(serial):
        return None

try:
    from ultralytics import YOLO

    YOLO_AVAILABLE = True
    YoloModelType = YOLO  # Type alias for YOLO model
except ImportError:
    print("警告: 未找到 'ultralytics' (YOLO). 视觉检测功能将受限。")
    YOLO_AVAILABLE = False
    YoloModelType = None


    class YOLO:  # Dummy class
        def __init__(self, *args, **kwargs): pass

        def predict(self, *args, **kwargs): return []

try:
    from scipy.spatial.transform import Rotation as R

    SCIPY_AVAILABLE = True
except ImportError:
    print("警告: 未找到 'scipy'. 坐标变换功能将受限。")
    SCIPY_AVAILABLE = False
    R = None  # Define as None for checks

# --- Local Module Imports ---
import config  # Constants and config loader
from robot_control import (initialize_robot, connect_arm_gripper, format_speed,
                           attempt_reset_arm, send_jog_command)
from ui import UIManager
from CPS import CPSClient, desire_right_pose, desire_left_pose  # Robot control client API

# Import the vision interaction logic module
import vision_interaction

# Type Alias for Pose/Joint data
PoseType = List[float]  # Example: [x, y, z, r, p, y]
JointType = List[float]
ControllerDict = Dict[str, Optional[CPSClient]]
CameraDict = Dict[str, Optional[CameraClientType]]
ModelDict = Dict[str, Optional[YoloModelType]]
CalibrationDict = Dict[str, Optional[np.ndarray]]


class DualArmController:
    """
    主控制器类，管理双臂机器人、UI、视觉组件和状态。

    Attributes:
        config_path (str): 主配置文件路径。
        config (Optional[Dict]): 加载的配置字典。
        ui_manager (UIManager): UI 管理器实例。
        controller_left (Optional[CPSClient]): 左臂控制器实例。
        controller_right (Optional[CPSClient]): 右臂控制器实例。
        cameras (CameraDict): 初始化后的相机客户端字典。
        models (ModelDict): 加载后的YOLO模型字典。
        calibration (CalibrationDict): 加载后的标定矩阵字典。
        # ... 其他状态和参数属性 ...
    """

    def __init__(self, config_path: str = config.CONFIG_FILE):
        """初始化控制器实例和默认状态。"""
        self.config_path: str = config_path
        self.config: Optional[Dict] = None
        self.ui_manager: UIManager = UIManager(self)

        # Robot Controllers & Status
        self.controller_left: Optional[CPSClient] = None
        self.controller_right: Optional[CPSClient] = None
        self.left_init_ok: bool = False
        self.right_init_ok: bool = False
        self.left_gripper_active: bool = False
        self.right_gripper_active: bool = False
        self.left_gripper_open: bool = True
        self.right_gripper_open: bool = True

        # Vision Components
        self.cameras: CameraDict = {}
        self.models: ModelDict = {}
        self.calibration: CalibrationDict = {}

        # State Variables
        self.running: bool = False
        self.control_modes: List[str] = [config.MODE_XYZ, config.MODE_RPY, config.MODE_RESET, config.MODE_VISION]
        self.current_mode_index: int = 0
        self.control_mode: str = self.control_modes[0]
        self.mode_button_press_time: Optional[float] = None
        self.status_message: str = "初始化中..."

        # Parameters (will be overridden by config)
        self.controls_map: Dict = {}
        self.audio_files_config: Dict = {}
        self._load_default_parameters()  # Load defaults first
        self._update_control_attributes()  # Initialize specific controls with defaults/empty

    def _load_default_parameters(self):
        """设置参数的默认值。"""
        self.current_speed_xy: float = config.DEFAULT_XY_SPEED
        self.current_speed_z: float = config.DEFAULT_Z_SPEED
        self.rpy_speed: float = config.DEFAULT_RPY_SPEED
        self.reset_speed: float = config.DEFAULT_RESET_SPEED
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
        self.font_path: Optional[str] = config.DEFAULT_FONT_PATH
        self.window_width: int = config.DEFAULT_WINDOW_WIDTH
        self.window_height: int = config.DEFAULT_WINDOW_HEIGHT
        self.font_size: int = config.DEFAULT_FONT_SIZE
        self.left_robot_ip: str = config.DEFAULT_IP
        self.right_robot_ip: str = config.DEFAULT_IP
        self.left_gripper_id: int = config.DEFAULT_GRIPPER_ID
        self.right_gripper_id: int = config.DEFAULT_GRIPPER_ID

    def _update_control_attributes(self):
        """从加载的 controls_map 更新特定的控制属性, 若无则使用默认。"""

        def get_control(key: str, default_index: int) -> dict:
            default_cfg = {'type': 'button', 'index': default_index}
            # Ensure controls_map is a dict before using get
            return self.controls_map.get(key, default_cfg) if isinstance(self.controls_map, dict) else default_cfg

        self.mode_switch_control = get_control('mode_switch_button', config.DEFAULT_MODE_SWITCH_BUTTON)
        self.speed_inc_control = get_control('speed_increase_alt', config.DEFAULT_SPEED_INC_BUTTON)
        self.speed_dec_control = get_control('speed_decrease', config.DEFAULT_SPEED_DEC_BUTTON)
        self.gripper_toggle_left_ctrl = get_control('gripper_toggle_left', config.DEFAULT_GRIPPER_L_BUTTON)
        self.gripper_toggle_right_ctrl = get_control('gripper_toggle_right', config.DEFAULT_GRIPPER_R_BUTTON)
        self.reset_left_arm_ctrl = self.controls_map.get('reset_left_arm')  # Can be None
        self.reset_right_arm_ctrl = self.controls_map.get('reset_right_arm')  # Can be None

    def setup(self) -> bool:
        """
        执行所有初始化步骤: 配置, Pygame, 视觉组件, 机器人硬件。
        返回 True 表示设置基本成功（允许进入主循环），False 表示关键失败。
        """
        print("=" * 10 + " 开始设置双臂控制器 " + "=" * 10)
        overall_success = True  # Track overall setup status

        # 1. 加载主配置文件
        print("\n[Setup 1/5] 加载主配置文件...")
        try:
            if not config.load_and_set_config_variables(self, self.config_path):
                self.status_message = "错误: 加载主配置失败"
                print(f"  {self.status_message}")
                return False  # Config loading is critical
            if self.config is None:
                self.status_message = "错误: 主配置加载后仍为空"
                print(f"  {self.status_message}")
                return False
            print("  主配置文件加载成功.")
            self._update_control_attributes()  # Update controls from loaded map
        except Exception as e_cfg:
            self.status_message = f"错误: 加载配置时异常: {e_cfg}"
            print(f"  {self.status_message}")
            traceback.print_exc()
            return False

        # 2. 初始化 Pygame 和 UI
        print("\n[Setup 2/5] 初始化 Pygame 和 UI...")
        try:
            if not self.ui_manager.init_pygame():
                self.status_message = self.ui_manager.status_message  # Get error
                print(f"  错误: Pygame 初始化失败 - {self.status_message}")
                return False  # Pygame/UI is critical
            self.status_message = self.ui_manager.status_message  # Update status
            print("  Pygame 和 UI 初始化成功.")
        except Exception as e_pygame:
            self.status_message = f"错误: Pygame 初始化异常: {e_pygame}"
            print(f"  {self.status_message}")
            traceback.print_exc()
            return False

        # 3. 初始化视觉组件
        print("\n[Setup 3/5] 初始化视觉组件...")
        vision_ok = self._initialize_vision_components()  # Returns True if all OK
        print(f"  视觉组件初始化完成 (状态: {'OK' if vision_ok else '存在问题'}).")
        # Don't necessarily fail setup if vision fails, but log it

        # 4. 初始化机器人硬件
        print("\n[Setup 4/5] 初始化机器人硬件...")
        robots_ok = self._init_robots()
        if not robots_ok:
            print("  警告: 机器人硬件初始化失败!")
            # Might still allow running if only one arm failed, TBD
        else:
            print("  机器人硬件初始化成功.")

        # 5. Final Setup State & Sound
        print("\n[Setup 5/5] 完成设置.")
        if robots_ok and vision_ok:
            self.status_message = "系统就绪"
            self.ui_manager.play_sound('system_ready')
        elif robots_ok and not vision_ok:
            self.status_message = self._append_status("机器人OK, 但视觉组件失败")
            self.ui_manager.play_sound('system_ready_partial')  # Consider adding this sound
        else:  # Robots failed
            self.status_message = self._append_status("错误: 机器人初始化失败")
            self.ui_manager.play_sound('system_init_error')

        self.ui_manager.update_status_message(self.status_message)
        self.running = True  # Allow main loop unless critical failure earlier
        print(f"\n==== 控制器设置完成 (最终状态: {self.status_message}) ====")
        return True  # Indicate setup completed, allowing run loop

    def _initialize_vision_components(self) -> bool:
        """初始化相机、模型和标定数据。 返回 True 表示所有组件成功。"""
        all_vision_ok = True
        setup_cfg = self.config.get('setup', {})
        if not isinstance(setup_cfg, dict):
            print("  错误: 'setup' 配置部分无效或缺失。")
            return False

        # --- 相机 ---
        print("  初始化相机...")
        if CAMERA_AVAILABLE:
            camera_serials = setup_cfg.get('camera_serials', {})
            if not camera_serials or not isinstance(camera_serials, dict):
                print("    警告: 未配置 'camera_serials' 或配置无效。")
                all_vision_ok = False  # Consider cameras essential?
            else:
                failed_cams_details = []
                for cam_name, serial in camera_serials.items():
                    print(f"    尝试 '{cam_name}' (SN: {serial})...")
                    cam_client = None
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
                                    try: cam_client.close()
                                    except: pass
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

        # --- YOLO 模型 ---
        print("\n  加载YOLO模型...")
        if YOLO_AVAILABLE:
            yolo_models_cfg = setup_cfg.get('yolo_models', {})
            if not yolo_models_cfg:
                print("    警告: 未配置 'yolo_models'。")
                # Allow continuing without models? Or set all_vision_ok = False?
                # Depends if vision mode *requires* models. Let's assume it does.
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
                if loaded_model_count == 0 and yolo_models_cfg:  # Configured but none loaded
                    print("    警告: 未能加载任何配置的YOLO模型。")
                    self.status_message = self._append_status("警告: 模型加载失败")
                    all_vision_ok = False
        else:
            print("    跳过YOLO模型加载 (库不可用).")
            self.status_message = self._append_status("警告: YOLO库不可用")
            all_vision_ok = False

        # --- 标定数据 ---
        print("\n  加载标定数据...")
        if not SCIPY_AVAILABLE:  # Need scipy for using the matrices
            print("    跳过标定加载 (scipy库不可用).")
            self.status_message = self._append_status("警告: scipy库不可用")
            all_vision_ok = False
        else:
            calibration_files = setup_cfg.get('calibration_files', {})
            if not calibration_files or len(calibration_files) < 2:  # Expect 'left' and 'right'
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
                            # Validate matrix structure
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
                if loaded_calib_count < len(['left', 'right']):  # Check if both loaded
                    print("    警告: 未能加载所有臂的标定矩阵。")
                    self.status_message = self._append_status("警告: 标定加载不完整")
                    all_vision_ok = False

        return all_vision_ok

    def _init_robots(self) -> bool:
        """连接并初始化机器人硬件。"""
        # ...(Implementation remains the same as previous response)...
        print("\n[Robot Init] 开始连接和初始化机器人...")
        self.left_init_ok, self.right_init_ok = False, False
        self.left_gripper_active, self.right_gripper_active = False, False
        all_ok = True
        # Left Arm
        try:
            print(f"  连接左臂 ({self.left_robot_ip})...")
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
        # Right Arm
        try:
            print(f"  连接右臂 ({self.right_robot_ip})...")
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
        """附加状态信息，用于累积警告/错误。"""
        if self.status_message and ("警告" in self.status_message or "错误" in self.status_message):
            if new_status_part not in self.status_message:
                return f"{self.status_message} | {new_status_part}"
            else:
                return self.status_message
        else:
            return new_status_part

    # --- Background Thread for Vision Processing ---
    def _threaded_process_vision_audio(self, audio_filepath: str):
        """
        后台线程处理语音: 发送->接收->处理指令(含视觉抓取)。
        明确传递区分左右臂的资源。
        """
        current_thread = threading.current_thread()
        thread_name = current_thread.name
        print(f"\n[{thread_name}] 开始处理语音: {audio_filepath}")
        self.ui_manager.update_status_message("正在发送语音指令...")

        # 1. Network Communication
        received_audio_path, command_json = vision_interaction.send_audio_and_receive_response(audio_filepath)

        # 2. Cleanup Sent Audio
        if os.path.exists(audio_filepath):
            try:
                os.remove(audio_filepath)
            except OSError as e:
                print(f"[{thread_name}] 清理发送文件时出错: {e}")

        # 3. Play Received Audio
        if received_audio_path:
            print(f"[{thread_name}] 收到音频回复, 准备播放...")
            self.ui_manager.update_status_message("正在播放回复...")
            self.ui_manager.play_sound('server_response_received')
            vision_interaction.play_audio_file(received_audio_path)  # Blocking playback
            if os.path.exists(received_audio_path):
                try:
                    os.remove(received_audio_path)
                except OSError as e:
                    print(f"[{thread_name}] 清理接收文件时出错: {e}")
        else:
            print(f"[{thread_name}] 未收到音频回复。")

        # 4. Process Received JSON Command
        final_status = "操作完成"
        action_success = False  # Track if the action was successful

        if command_json and isinstance(command_json, dict) and "error" not in command_json:
            print(f"[{thread_name}] 收到有效JSON指令: {command_json}")
            action = command_json.get('action', '未知')
            self.ui_manager.update_status_message(f"处理指令: {action}...")

            # --- Prepare Resources for vision_interaction ---
            # Pass only initialized controllers
            robot_controllers_dict: ControllerDict = {
                'left': self.controller_left if self.left_init_ok else None,
                'right': self.controller_right if self.right_init_ok else None
            }
            robot_controllers_dict = {k: v for k, v in robot_controllers_dict.items() if v}

            # Pass initialized vision components
            camera_clients_dict: CameraDict = self.cameras
            models_dict: ModelDict = self.models
            calibration_dict: CalibrationDict = self.calibration

            # --- Call Handler (with pre-checks if action is grasp) ---
            if action == "grasp":
                target_details = command_json.get("target", {})
                arm_choice = target_details.get("arm_choice")
                object_id = target_details.get("id")
                required_cam_key = f"{arm_choice}_hand" if arm_choice else None
                required_model_key = object_id  # Or 'default' if specific not found

                # Perform rigorous checks BEFORE calling the potentially long grasp sequence
                checks_passed = True
                if not robot_controllers_dict.get(arm_choice):
                    final_status = f"错误: {arm_choice}臂未就绪";
                    checks_passed = False
                elif not camera_clients_dict.get(required_cam_key):
                    final_status = f"错误: {required_cam_key}相机未就绪";
                    checks_passed = False
                elif not models_dict.get(required_model_key) and not models_dict.get('default'):
                    final_status = f"错误: 无'{required_model_key}'或默认模型";
                    checks_passed = False
                elif  calibration_dict.get(arm_choice) is None:
                    final_status = f"错误: {arm_choice}臂标定缺失";
                    checks_passed = False

                if checks_passed:
                    try:
                        print(f"[{thread_name}] 调用 vision_interaction.handle_command_json (grasp)...")
                        # handle_command_json calls initiate_grasp_from_command which returns True/False
                        action_success = vision_interaction.handle_command_json(
                            command_json, robot_controllers_dict, camera_clients_dict, models_dict, calibration_dict
                        )
                        final_status = f"抓取指令 " + ("成功" if action_success else "失败")
                        self.ui_manager.play_sound(
                            'action_success_general' if action_success else 'action_fail_general')
                    except Exception as e_handle:
                        print(f"[{thread_name}] 调用 handle_command_json (grasp) 时发生错误: {e_handle}")
                        traceback.print_exc()
                        final_status = f"错误: 处理抓取指令失败"
                        self.ui_manager.play_sound('action_fail_general')
                else:
                    # If checks failed, final_status already contains the error message
                    print(f"[{thread_name}] 抓取指令前置检查失败: {final_status}")
                    self.ui_manager.play_sound('action_fail_general')
            else:
                # Handle other non-grasp actions
                try:
                    print(f"[{thread_name}] 调用 vision_interaction.handle_command_json ({action})...")
                    # Assuming handle_command_json returns boolean for success on other actions too? Or void?
                    # Let's assume void for now for non-grasp.
                    vision_interaction.handle_command_json(
                        command_json, robot_controllers_dict, camera_clients_dict, models_dict, calibration_dict
                    )
                    final_status = f"指令 '{action}' 已处理"
                    action_success = True  # Assume success for non-grasp if no exception
                    self.ui_manager.play_sound('action_success_general')
                except Exception as e_handle_other:
                    print(f"[{thread_name}] 处理指令 '{action}' 时发生错误: {e_handle_other}")
                    traceback.print_exc()
                    final_status = f"错误: 处理 '{action}' 指令失败"
                    self.ui_manager.play_sound('action_fail_general')

        elif command_json and "error" in command_json:
            final_status = f"错误: 服务器返回JSON错误 ({command_json['error']})"
            self.ui_manager.play_sound('action_fail_general')
        else:
            final_status = "未收到有效指令或JSON无效"

        # 5. Update UI and Prompt for Next Action
        start_rec_cfg = self.controls_map.get('vision_start_record')
        start_rec_btn_idx = start_rec_cfg.get('index', '?') if start_rec_cfg else '?'
        self.ui_manager.update_status_message(f"{final_status}. 按 B{start_rec_btn_idx} 开始新指令.")
        print(f"[{thread_name}] 视觉交互处理完毕。")

    # --- Other Core Methods ---
    # (Ensure type hints and docstrings are added/updated for clarity)

    def switch_control_mode(self):
        """切换到下一个控制模式，停止当前运动并更新UI。"""
        self.stop_all_movement()
        old_mode = self.control_mode
        self.current_mode_index = (self.current_mode_index + 1) % len(self.control_modes)
        self.control_mode = self.control_modes[self.current_mode_index]
        print(f"切换模式: {old_mode} -> {self.control_mode}")

        sound_event = None
        status_msg = f"模式: {self.control_mode}"

        # Mode specific actions on switch
        if self.control_mode == config.MODE_XYZ:
            sound_event = 'xyz_mode'
        elif self.control_mode == config.MODE_RPY:
            sound_event = 'rpy_mode'
        elif self.control_mode == config.MODE_RESET:
            sound_event = 'reset_mode'
        elif self.control_mode == config.MODE_VISION:
            sound_event = 'vision_enter'
            # Update status message with prompt for vision mode
            start_rec_cfg = self.controls_map.get('vision_start_record')
            start_rec_btn_idx = start_rec_cfg.get('index', '?') if start_rec_cfg else '?'
            status_msg = f"视觉模式: 按 B{start_rec_btn_idx} 开始语音"

        if sound_event: self.ui_manager.play_sound(sound_event)
        self.ui_manager.update_status_message(status_msg)

        # Update window title
        if self.ui_manager.screen:
            pygame.display.set_caption(f"双臂控制 (模式: {self.control_mode})")

        # Cancel recording if switching out of vision mode
        if old_mode == config.MODE_VISION and vision_interaction.is_recording:
            print("切换模式时取消进行中的录音...")
            if vision_interaction.cancel_recording():
                self.ui_manager.play_sound('vision_record_cancel')

    def stop_all_movement(self):
        """向所有已初始化的机械臂发送停止指令。"""
        print("发送停止所有运动指令...")
        stop_payload = [0.0] * 6
        # Use a high acceleration for stopping quickly
        stop_acc, stop_arot, stop_t = 200, 20, 0.05
        try:
            if self.left_init_ok and self.controller_left:
                # Prioritize dedicated stop commands if they exist and work reliably
                # if hasattr(self.controller_left, 'stopl'): self.controller_left.stopl() else:
                self.controller_left.moveBySpeedl(stop_payload, stop_acc, stop_arot, stop_t)
            if self.right_init_ok and self.controller_right:
                # if hasattr(self.controller_right, 'stopl'): self.controller_right.stopl() else:
                self.controller_right.moveBySpeedl(stop_payload, stop_acc, stop_arot, stop_t)
            time.sleep(0.1)  # Allow time for commands to process
        except Exception as e:
            print(f"发送停止指令时出错: {e}")

    def toggle_gripper(self, side: str):
        """切换指定侧 ('left' or 'right') 夹爪的状态。"""
        controller = self.controller_left if side == 'left' else self.controller_right
        is_open = self.left_gripper_open if side == 'left' else self.right_gripper_open
        is_active = self.left_gripper_active if side == 'left' else self.right_gripper_active

        if not controller or not is_active:
            self.ui_manager.play_sound('gripper_inactive')
            msg = f"{side} 夹爪无效或未初始化"
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
            # Using wait=False for potentially faster UI response
            success = action(speed=self.gripper_speed, force=self.gripper_force, wait=False)
            # Assume state change on successful command send
            if side == 'left':
                self.left_gripper_open = not is_open
            else:
                self.right_gripper_open = not is_open
            self.ui_manager.update_status_message(f"{side} 夹爪: {new_state_str}")
            # TODO: Potentially add a check later to confirm gripper state if wait=False
        except Exception as e:
            print(f"切换 {side} 夹爪出错: {e}")
            self.ui_manager.play_sound('action_fail_general')
            self.ui_manager.update_status_message(f"{side} 夹爪切换失败")

    def attempt_reset_left_arm(self):
        """尝试左臂回正。"""
        if not self.left_init_ok:
            msg = "左臂回正失败: 未初始化";
            print(msg)
            self.ui_manager.play_sound('left_reset_fail');
            self.ui_manager.update_status_message(msg)
            return
        self.ui_manager.update_status_message("左臂回正中...")
        # Check if attempt_reset_arm uses config constants internally or needs them passed
        success = attempt_reset_arm(self.controller_left, "左臂", config.TARGET_RESET_RPY_LEFT,
                                    self.reset_speed, desire_left_pose, 'move_robot',  # Verify move_func name
                                    self.ui_manager.play_sound, 'left_reset_success', 'left_reset_fail')
        self.ui_manager.update_status_message("左臂回正成功" if success else "左臂回正失败")

    def attempt_reset_right_arm(self):
        """尝试右臂回正。"""
        if not self.right_init_ok:
            msg = "右臂回正失败: 未初始化";
            print(msg)
            self.ui_manager.play_sound('right_reset_fail');
            self.ui_manager.update_status_message(msg)
            return
        self.ui_manager.update_status_message("右臂回正中...")
        success = attempt_reset_arm(self.controller_right, "右臂", config.TARGET_RESET_RPY_RIGHT,
                                    self.reset_speed, desire_right_pose, 'move_right_robot',  # Verify move_func name
                                    self.ui_manager.play_sound, 'right_reset_success', 'right_reset_fail')
        self.ui_manager.update_status_message("右臂回正成功" if success else "右臂回正失败")

    def _calculate_speed_commands(self) -> Tuple[np.ndarray, np.ndarray]:
        """根据手柄输入和当前模式计算目标速度向量。"""
        # ...(Implementation from previous response, using config.MODE_... constants)...
        speed_left_cmd, speed_right_cmd = np.zeros(6), np.zeros(6)
        if not self.ui_manager.joystick: return speed_left_cmd, speed_right_cmd  # No joystick, no speed

        current_mode_prefix = self.control_mode.lower() + "_"
        if self.control_mode in [config.MODE_XYZ, config.MODE_RPY]:
            # Iterate through controls defined in the config file
            for action, control_cfg in self.controls_map.items():
                if not isinstance(control_cfg, dict): continue  # Skip invalid control entries

                # Check if the action matches the current mode and specifies an arm
                if action.startswith(current_mode_prefix) and ('_arm' in action):
                    # Check if the corresponding joystick input is active
                    if self.ui_manager.get_joystick_input_state(control_cfg):
                        target_array = None
                        if 'left_arm' in action:
                            target_array = speed_left_cmd
                        elif 'right_arm' in action:
                            target_array = speed_right_cmd
                        else:
                            continue  # Skip if arm not identified

                        axis_idx, base_speed = -1, 0.0
                        direction = 1.0 if '_pos' in action else -1.0 if '_neg' in action else 1.0  # Determine direction

                        # Determine which axis and base speed to use based on mode
                        if self.control_mode == config.MODE_XYZ:
                            if '_x' in action:
                                axis_idx, base_speed = 0, self.current_speed_xy
                            elif '_y' in action:
                                axis_idx, base_speed = 1, self.current_speed_xy
                            elif '_z' in action:
                                axis_idx, base_speed = 2, self.current_speed_z
                        elif self.control_mode == config.MODE_RPY:
                            if '_roll' in action:
                                axis_idx, base_speed = 3, self.rpy_speed
                            elif '_pitch' in action:
                                axis_idx, base_speed = 4, self.rpy_speed
                            elif '_yaw' in action:
                                axis_idx, base_speed = 5, self.rpy_speed

                        # If a valid axis was identified, calculate and set the speed
                        if axis_idx != -1:
                            if control_cfg.get('type') == 'axis':  # Scale speed based on analog input value
                                try:
                                    axis_val = self.ui_manager.joystick.get_axis(control_cfg.get('index', -1))
                                    target_array[axis_idx] = base_speed * abs(axis_val) * direction
                                except (pygame.error, IndexError):  # Catch potential joystick errors
                                    target_array[axis_idx] = 0
                            else:  # Button input - use full base speed
                                target_array[axis_idx] = base_speed * direction
        # For VISION or RESET mode, speed commands remain zero as actions are event-driven
        return speed_left_cmd, speed_right_cmd

    def _apply_transformations(self, speed_left_cmd: np.ndarray, speed_right_cmd: np.ndarray) -> Tuple[
        np.ndarray, np.ndarray]:
        """对速度指令应用坐标变换 (示例)。"""
        # ...(Implementation from previous response)...
        if R is None:  # Check if SciPy/Rotation is available
            print("警告: Scipy未找到，跳过坐标变换。")
            return speed_left_cmd, speed_right_cmd

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
        elif self.control_mode == config.MODE_RPY:
            speed_left_final[3:], speed_right_final[3:] = speed_left_cmd[3:], speed_right_cmd[3:]
        # VISION/RESET modes already have zero speed from _calculate_speed_commands
        return speed_left_final, speed_right_final

    def _send_robot_commands(self, speed_left_final: np.ndarray, speed_right_final: np.ndarray):
        """发送持续运动指令 (SpeedL 或 Jog)。"""
        # ...(Implementation from previous response)...
        # Only send if the corresponding arm is initialized
        if self.control_mode == config.MODE_XYZ:
            if self.left_init_ok:
                try: self.controller_left.moveBySpeedl(list(speed_left_final), self.acc, self.arot,
                                                                         self.t_interval)
                except Exception as e: print(
                f"L Speedl Err: {e}")
            if self.right_init_ok:
                try: self.controller_right.moveBySpeedl(list(speed_right_final), self.acc, self.arot,
                                                                           self.t_interval)
                except Exception as e: print(
                f"R Speedl Err: {e}")
        elif self.control_mode == config.MODE_RPY:
            if self.left_init_ok: send_jog_command(self.controller_left, speed_left_final, self.min_speed,
                                                   self.max_speed)
            if self.right_init_ok: send_jog_command(self.controller_right, speed_right_final, self.min_speed,
                                                    self.max_speed)

    def run_main_loop(self):
        """主运行循环，处理事件、计算、发送指令和绘制UI。"""
        if not self.running:
            print("错误: 控制器未成功设置，无法启动主循环。")
            # Maybe show error on screen if pygame initialized?
            # if self.ui_manager.screen: ... display error ... pygame.time.wait(5000)
            return

        print("\n--- 控制循环开始 (按 ESC 退出) ---")
        while self.running:
            # Optional: Get status updates from background threads via a queue
            # try:
            #    new_status = self.status_queue.get_nowait()
            #    self.status_message = new_status
            # except queue.Empty:
            #    pass # No new status

            # 1. Handle Events (Input, Mode Changes, Vision Triggers)
            self.ui_manager.handle_events()  # This now handles vision button presses
            if not self.running: break  # Check if quit event occurred

            # 2. Calculate Speeds (Zero for Vision/Reset)
            speed_left_cmd, speed_right_cmd = self._calculate_speed_commands()

            # 3. Apply Transformations (Only relevant for XYZ/RPY)
            speed_left_final, speed_right_final = self._apply_transformations(speed_left_cmd, speed_right_cmd)

            # 4. Send Continuous Commands (Only for XYZ/RPY)
            if self.control_mode in [config.MODE_XYZ, config.MODE_RPY]:
                # Check init status before sending
                if self.left_init_ok or self.right_init_ok:
                    self._send_robot_commands(speed_left_final, speed_right_final)

            # 5. Update Display
            self.ui_manager.status_message = self.status_message  # Ensure UI has latest status
            self.ui_manager.draw_display(speed_left_final, speed_right_final)

            # 6. Tick Clock
            self.ui_manager.clock.tick(30)  # Maintain target FPS

        print("--- 控制循环已终止 ---")

    def cleanup(self):
        """停止机器人、关闭相机并清理所有资源。"""
        print("\n" + "=" * 10 + " 开始清理和退出 " + "=" * 10)
        self.running = False  # Ensure loop flag is false

        # 1. Stop Robot Movement
        print("  [Cleanup 1/4] 发送停止运动指令...")
        self.stop_all_movement()

        # 2. Close Cameras
        print("  [Cleanup 2/4] 关闭相机...")
        if not self.cameras:
            print("    无需关闭相机 (未初始化).")
        else:
            for cam_name, cam_client in self.cameras.items():
                if cam_client and hasattr(cam_client, 'close'):
                    try:
                        print(f"    关闭相机 '{cam_name}'...")
                        cam_client.close()  # Call the camera's close method
                    except Exception as e_cam_close:
                        print(f"    关闭相机 '{cam_name}' 时出错: {e_cam_close}")
            self.cameras = {}  # Clear the dictionary

        # 3. Disconnect Robots
        print("  [Cleanup 3/4] 断开机器人连接...")
        controllers_to_disconnect = [
            ("左臂", self.controller_left),
            ("右臂", self.controller_right)
        ]
        for name, controller in controllers_to_disconnect:
            if controller:
                try:
                    # Optional: Check if connected before disconnecting
                    # if hasattr(controller,'is_connected') and controller.is_connected():
                    print(f"    断开 {name} 连接...")
                    controller.disconnect()
                    print(f"    {name} 已断开。")
                    # else: print(f"    {name} 已断开或未连接。")
                except Exception as disconn_e:
                    print(f"    断开 {name} 连接时出错: {disconn_e}")
            # Clear reference regardless of disconnect success
            if name == "左臂":
                self.controller_left = None
            else:
                self.controller_right = None
        self.left_init_ok = False;
        self.right_init_ok = False  # Update status

        # 4. Quit Pygame (via UI Manager)
        print("  [Cleanup 4/4] 关闭 Pygame...")
        self.ui_manager.quit()

        print("\n==== 清理完成，程序退出 ====")