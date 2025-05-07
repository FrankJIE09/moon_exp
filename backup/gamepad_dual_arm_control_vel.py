import pygame
from CPS import *
import time
import numpy as np
from scipy.spatial.transform import Rotation as R
import traceback
import sys
import os
import yaml

# -- 配置文件路径 --
CONFIG_FILE = '../config.yaml'

# -- 全局变量 (将被配置文件覆盖或使用默认值) --
FONT_PATH = None; LEFT_ROBOT_IP = "127.0.0.1"; RIGHT_ROBOT_IP = "127.0.0.1"
WINDOW_WIDTH = 750; WINDOW_HEIGHT = 380; FONT_SIZE = 18
LEFT_GRIPPER_ID = 9; RIGHT_GRIPPER_ID = 9 # 默认夹爪 ID
# (速度/ACC/AROT/T/阈值 将从配置加载)

# --- Pygame 颜色常量 ---
WHITE = (255, 255, 255); BLACK = (0, 0, 0); GREEN = (0, 255, 0); RED = (255, 0, 0); BLUE = (100, 100, 255)
INFO_X_MARGIN = 20; INFO_Y_START = 20; LINE_SPACING = 25

# --- 手柄按钮索引 (仅作参考/默认值) ---
# (不再需要在这里定义 A_BUTTON 等，因为它们来自配置)

# -- 功能函数 --
def load_config(filepath):
    # (load_config 函数与上个版本基本相同，确保返回 config 字典)
    try:
        with open(filepath, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)
        if config is None: print(f"错误: 配置文件 {filepath} 为空或格式错误。"); return None
        print(f"成功加载配置文件: {filepath}")
        return config
    except FileNotFoundError: print(f"错误: 配置文件未找到: {filepath}"); return None
    except yaml.YAMLError as e: print(f"错误: 解析配置文件 {filepath} 失败: {e}"); return None
    except Exception as e: print(f"加载配置文件时发生未知错误: {e}"); return None

def format_speed(speed_array):
    return f"[{speed_array[0]:>6.1f}, {speed_array[1]:>6.1f}, {speed_array[2]:>6.1f}, {speed_array[3]:>6.1f}, {speed_array[4]:>6.1f}, {speed_array[5]:>6.1f}]"

def initialize_robot(controller, arm_name):
    # (与之前相同)
    print(f"\n--- 初始化 {arm_name} ---")
    # ... (省略内部代码，与之前版本相同) ...
    try:
        print(f"[{arm_name}] 1/4: 上电...");
        if not controller.power_on(): raise RuntimeError(f"{arm_name} 上电失败")
        print(f"[{arm_name}] 上电成功."); time.sleep(0.5)
        print(f"[{arm_name}] 2/4: 清除报警..."); controller.clearAlarm()
        print(f"[{arm_name}] 清除报警完成."); time.sleep(0.5)
        print(f"[{arm_name}] 3/4: 同步电机...");
        if not controller.setMotorStatus(): print(f"警告: {arm_name} 同步电机失败.")
        else: print(f"[{arm_name}] 同步电机成功."); time.sleep(0.5)
        print(f"[{arm_name}] 4/4: 伺服使能...");
        if not controller.set_servo(1): raise RuntimeError(f"{arm_name} 伺服使能失败")
        print(f"[{arm_name}] 伺服使能成功.")
        print(f"--- {arm_name} 初始化完成 ---")
        return True
    except Exception as e: print(f"错误: 初始化 {arm_name} 时: {e}"); traceback.print_exc(); return False


def connect_arm_gripper(controller, arm_name):
    """尝试连接并激活夹爪"""
    print(f"--- 尝试连接 {arm_name} 夹爪 ---")
    try:
        if controller.connect_gripper(): # connect_gripper 应该返回 True/False
            print(f"{arm_name} 夹爪连接并激活成功。")
            return True
        else:
            print(f"错误: {arm_name} 夹爪连接或激活失败 (检查 CPS.py 日志)。")
            return False
    except AttributeError:
        print(f"错误: 控制器对象没有 'connect_gripper' 方法。请检查 CPS.py 版本。")
        return False
    except Exception as e:
        print(f"连接 {arm_name} 夹爪时发生异常: {e}")
        traceback.print_exc()
        return False

# -- 加载配置 --
config = load_config(CONFIG_FILE)
if config is None or 'controls' not in config or 'settings' not in config or 'setup' not in config:
    print("错误：配置文件加载失败或缺少部分 (setup, controls, settings)。程序退出。")
    exit()

# -- 从配置中提取参数 --
setup_cfg = config.get('setup', {})
FONT_PATH = setup_cfg.get('font_path')
LEFT_ROBOT_IP = setup_cfg.get('left_robot_ip', '127.0.0.1')
RIGHT_ROBOT_IP = setup_cfg.get('right_robot_ip', '127.0.0.1')
LEFT_GRIPPER_ID = setup_cfg.get('left_gripper_id', 9) # 从配置读取或用默认值
RIGHT_GRIPPER_ID = setup_cfg.get('right_gripper_id', 9)
WINDOW_WIDTH = setup_cfg.get('window_width', 750)
WINDOW_HEIGHT = setup_cfg.get('window_height', 380) # 确认足够高
FONT_SIZE = setup_cfg.get('font_size', 18)

settings_cfg = config.get('settings', {})
initial_xy_speed = settings_cfg.get('initial_xy_speed', 40.0)
initial_z_speed = settings_cfg.get('initial_z_speed', 30.0)
speed_increment = settings_cfg.get('speed_increment', 5.0)
min_speed = settings_cfg.get('min_speed', 5.0)
max_speed = settings_cfg.get('max_speed', 100.0)
ACC = settings_cfg.get('acc', 100); AROT = settings_cfg.get('arot', 10); T = settings_cfg.get('t', 0.1)
TRIGGER_AXIS_THRESHOLD = settings_cfg.get('trigger_threshold', 0.1)
GRIPPER_SPEED = settings_cfg.get('gripper_speed', 150) # 夹爪速度
GRIPPER_FORCE = settings_cfg.get('gripper_force', 100) # 夹爪力度

controls_map = config.get('controls', {})
# (退出按钮现在由 Esc/关闭窗口处理, quit_control 不再需要)
speed_inc_control = controls_map.get('speed_increase', {'type': 'button', 'index': 7}) # 默认 Start
speed_dec_control = controls_map.get('speed_decrease', {'type': 'button', 'index': 6}) # 默认 Back
gripper_toggle_left_ctrl = controls_map.get('gripper_toggle_left', {'type': 'button', 'index': 9}) # 默认 R3
gripper_toggle_right_ctrl = controls_map.get('gripper_toggle_right', {'type': 'button', 'index': 10})# 默认 Guide

# -- 当前状态变量 --
current_speed_xy = initial_xy_speed
current_speed_z = initial_z_speed
left_gripper_open = True # 假设初始状态为打开
right_gripper_open = True
left_gripper_active = False # 标记夹爪是否成功初始化
right_gripper_active = False

# -- 初始化 Pygame、字体和手柄 --
pygame.init(); pygame.font.init()
screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
pygame.display.set_caption("双臂手柄控制 (XY+Z+夹爪+调速)")
info_font = None
# (字体加载逻辑与之前相同)
if FONT_PATH and os.path.exists(FONT_PATH):
    try: info_font = pygame.font.Font(FONT_PATH, FONT_SIZE); print(f"字体加载成功: {FONT_PATH}")
    except Exception as e: print(f"字体加载失败: {e}")
elif FONT_PATH: print(f"字体未找到: {FONT_PATH}")
else: print("警告: 未配置字体路径.")
if info_font is None: print(f"警告: 将使用默认字体."); info_font = pygame.font.Font(None, FONT_SIZE)

clock = pygame.time.Clock()
joystick_count = pygame.joystick.get_count()
if joystick_count == 0: print("错误：未检测到手柄！"); pygame.quit(); exit()
joystick = pygame.joystick.Joystick(0); joystick.init()
print(f"已初始化手柄: {joystick.get_name()}")
num_hats = joystick.get_numhats(); num_axes = joystick.get_numaxes(); num_buttons = joystick.get_numbuttons()

print("机器人初始化信息将打印到控制台...")
time.sleep(1)

# -- 连接和初始化机器人与夹爪 --
controller_left = None; controller_right = None
left_initialized = False; right_initialized = False
try:
    # 注意：将夹爪 ID 传递给 CPSClient 构造函数
    print(f"连接左臂 ({LEFT_ROBOT_IP})...");
    controller_left = CPSClient(LEFT_ROBOT_IP, gripper_slave_id=LEFT_GRIPPER_ID)
    if not controller_left.connect(): raise ConnectionError("左臂连接失败")
    print("左臂连接成功.")
    print(f"连接右臂 ({RIGHT_ROBOT_IP})...");
    controller_right = CPSClient(RIGHT_ROBOT_IP, gripper_slave_id=RIGHT_GRIPPER_ID)
    if not controller_right.connect(): raise ConnectionError("右臂连接失败")
    print("右臂连接成功.")

    left_initialized = initialize_robot(controller_left, "左臂")
    right_initialized = initialize_robot(controller_right, "右臂")
    if not (left_initialized and right_initialized): raise RuntimeError("机器人初始化失败")

    # 连接夹爪
    if left_initialized: left_gripper_active = connect_arm_gripper(controller_left, "左臂")
    if right_initialized: right_gripper_active = connect_arm_gripper(controller_right, "右臂")

    print("\n初始化完成，开始控制...")

    # -- 主循环 --
    running = True
    while running:
        # --- 事件处理 ---
        for event in pygame.event.get():
            if event.type == pygame.QUIT: running = False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE: running = False

            if event.type == pygame.JOYBUTTONDOWN:
                button_index = event.button
                # 速度调节
                if speed_inc_control.get('type') == 'button' and button_index == speed_inc_control.get('index', -1):
                    current_speed_xy = min(max_speed, current_speed_xy + speed_increment)
                    current_speed_z = min(max_speed, current_speed_z + speed_increment)
                    # print(f"速度增加: XY={current_speed_xy:.1f}, Z={current_speed_z:.1f}")
                elif speed_dec_control.get('type') == 'button' and button_index == speed_dec_control.get('index', -1):
                    current_speed_xy = max(min_speed, current_speed_xy - speed_increment)
                    current_speed_z = max(min_speed, current_speed_z - speed_increment)
                    # print(f"速度减少: XY={current_speed_xy:.1f}, Z={current_speed_z:.1f}")
                # 左夹爪切换
                elif gripper_toggle_left_ctrl.get('type') == 'button' and button_index == gripper_toggle_left_ctrl.get('index', -1):
                    if left_gripper_active:
                        if left_gripper_open:
                            print("指令: 关闭左夹爪")
                            controller_left.close_gripper(speed=GRIPPER_SPEED, force=GRIPPER_FORCE, wait=False)
                        else:
                            print("指令: 打开左夹爪")
                            controller_left.open_gripper(speed=GRIPPER_SPEED, force=GRIPPER_FORCE, wait=False)
                        left_gripper_open = not left_gripper_open
                    else: print("左夹爪未激活")
                # 右夹爪切换
                elif gripper_toggle_right_ctrl.get('type') == 'button' and button_index == gripper_toggle_right_ctrl.get('index', -1):
                    if right_gripper_active:
                        if right_gripper_open:
                            print("指令: 关闭右夹爪")
                            controller_right.close_gripper(speed=GRIPPER_SPEED, force=GRIPPER_FORCE, wait=False)
                        else:
                            print("指令: 打开右夹爪")
                            controller_right.open_gripper(speed=GRIPPER_SPEED, force=GRIPPER_FORCE, wait=False)
                        right_gripper_open = not right_gripper_open
                    else: print("右夹爪未激活")


        # --- 计算移动速度 (根据配置文件和当前速度) ---
        speed_left_cmd = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        speed_right_cmd = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        # (移动速度计算逻辑与上个版本相同，使用 current_speed_xy/z)
        for action, control in controls_map.items():
            if action in ['speed_increase', 'speed_decrease', 'gripper_toggle_left', 'gripper_toggle_right']: continue
            active = False; ctrl_type = control.get('type'); ctrl_index = control.get('index', -1)
            if ctrl_type == 'button' and not (0 <= ctrl_index < num_buttons): continue
            if ctrl_type == 'axis' and not (0 <= ctrl_index < num_axes): continue
            if ctrl_type == 'hat' and not (0 <= ctrl_index < num_hats): continue

            if ctrl_type == 'button': active = joystick.get_button(ctrl_index) == 1
            elif ctrl_type == 'axis':
                axis_val = joystick.get_axis(ctrl_index); threshold = control.get('threshold', TRIGGER_AXIS_THRESHOLD); direction = control.get('direction', 1)
                if direction == 1 and axis_val > (-1.0 + threshold * 2.0): active = True
                elif direction == -1 and axis_val < (1.0 - threshold * 2.0): active = True
            elif ctrl_type == 'hat':
                hat_val = joystick.get_hat(ctrl_index); hat_axis = control.get('axis', 'x'); direction = control.get('direction', 1)
                if hat_axis == 'x' and hat_val[0] == direction: active = True
                elif hat_axis == 'y' and hat_val[1] == direction: active = True

            if active:
                target_speed_array = None; axis_index = -1; base_speed = 0.0; direction_sign = 1.0
                if 'left_arm' in action: target_speed_array = speed_left_cmd
                elif 'right_arm' in action: target_speed_array = speed_right_cmd
                if target_speed_array is not None:
                    if '_x' in action: axis_index = 0; base_speed = current_speed_xy
                    elif '_y' in action: axis_index = 1; base_speed = current_speed_xy
                    elif '_z' in action: axis_index = 2; base_speed = current_speed_z
                    if '_pos' in action: direction_sign = 1.0
                    elif '_neg' in action: direction_sign = -1.0
                    if axis_index != -1: target_speed_array[axis_index] = base_speed * direction_sign


        # --- 应用坐标系旋转 ---
        # (与之前相同)
        speed_left_final = speed_left_cmd.copy(); speed_right_final = speed_right_cmd.copy()
        try: rot_left = R.from_euler('xyz', [65, 0, 10], degrees=True).as_matrix(); speed_left_final[0:3] = speed_left_cmd[0:3] @ rot_left
        except Exception: speed_left_final[0:3] = [0,0,0]
        try: rot_right = R.from_euler('xyz', [65.33430565, -4.20854252,-9.07946747], degrees=True).as_matrix(); speed_right_final[0:3] = speed_right_cmd[0:3] @ rot_right
        except Exception: speed_right_final[0:3] = [0,0,0]
        speed_left_final[3:] = [0.0, 0.0, 0.0]; speed_right_final[3:] = [0.0, 0.0, 0.0]

        # --- 绘制图形界面 ---
        screen.fill(BLACK)
        lines_to_draw = []
        lines_to_draw.append(f"--- 双臂手柄控制 (配置: {CONFIG_FILE}) --- (Esc 退出)")
        lines_to_draw.append(f"左臂({LEFT_ROBOT_IP}): {'OK' if left_initialized else 'ERR'} | 右臂({RIGHT_ROBOT_IP}): {'OK' if right_initialized else 'ERR'}")
        lines_to_draw.append(f"左夹爪: {'打开' if left_gripper_open else '关闭'} ({'活动' if left_gripper_active else '无效'}) | 右夹爪: {'打开' if right_gripper_open else '关闭'} ({'活动' if right_gripper_active else '无效'})")
        lines_to_draw.append(f"速度 XY: {current_speed_xy:.1f} | Z: {current_speed_z:.1f} (B{speed_dec_control['index']}/B{speed_inc_control['index']} 调节)")
        lines_to_draw.append("-" * 80)
        lines_to_draw.append("[当前速度指令 (已发送)]")
        lines_to_draw.append(f"  左臂速度: {format_speed(speed_left_final)}")
        lines_to_draw.append(f"  右臂速度: {format_speed(speed_right_final)}")
        lines_to_draw.append("-" * 80)
        lines_to_draw.append(f"提示: 左夹爪(B{gripper_toggle_left_ctrl['index']}), 右夹爪(B{gripper_toggle_right_ctrl['index']}), 速度(B{speed_dec_control['index']}/B{speed_inc_control['index']})")


        y_pos = INFO_Y_START
        for i, line in enumerate(lines_to_draw):
            try:
                text_surface = info_font.render(line, True, WHITE)
                # 添加更多颜色逻辑
                color = WHITE
                if "状态:" in line or "ERR" in line or "无效" in line : color = RED if ("ERR" in line or "无效" in line) else GREEN
                elif "速度 XY:" in line : color = BLUE
                elif "夹爪:" in line: color = (200, 200, 0) # 黄色

                text_surface = info_font.render(line, True, color)
                screen.blit(text_surface, (INFO_X_MARGIN, y_pos))
            except Exception as render_e:
                if i==0: print(f"渲染文本时出错: {render_e}")
                try: error_font = pygame.font.Font(None, FONT_SIZE); error_surface = error_font.render("! RENDER ERR !", True, RED); screen.blit(error_surface, (INFO_X_MARGIN, y_pos))
                except: pass
            y_pos += LINE_SPACING

        pygame.display.flip()

        # --- 发送速度命令 (已启用) ---
        # (移动指令现在包含Z轴速度)
        if left_initialized: suc_l, res_l, _ = controller_left.moveBySpeedl(list(speed_left_final), ACC, AROT, T)
        if right_initialized: suc_r, res_r, _ = controller_right.moveBySpeedl(list(speed_right_final), ACC, AROT, T)

        # --- 控制帧率 ---
        clock.tick(30)

except KeyboardInterrupt: print("\n接收到 Ctrl+C，正在停止...")
except (ConnectionError, RuntimeError, Exception) as e: print(f"\n发生错误: {e}"); traceback.print_exc()
finally:
    # -- 清理 --
    print("\n正在停止机器人并执行清理操作...")
    stop_speed = [0.0] * 6
    try:
        # 尝试停止运动
        if left_initialized and controller_left and controller_left.sock:
             controller_left.moveBySpeedl(stop_speed, ACC, AROT, T)
        if right_initialized and controller_right and controller_right.sock:
             controller_right.moveBySpeedl(stop_speed, ACC, AROT, T)

        # (可选: 下使能/下电)
        # ...

    except Exception as stop_e: print(f"发送停止指令时出错: {stop_e}")

    # 断开连接 (会尝试关闭 TCI)
    if controller_left and controller_left.sock:
        try: controller_left.disconnect(); print("左臂已断开。")
        except Exception as dis_e: print(f"断开左臂时出错: {dis_e}")
    if controller_right and controller_right.sock:
        try: controller_right.disconnect(); print("右臂已断开。")
        except Exception as dis_e: print(f"断开右臂时出错: {dis_e}")

    pygame.quit()
    print("程序退出。")