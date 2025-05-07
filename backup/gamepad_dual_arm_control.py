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
FONT_PATH = None  # 将从 config 读取
LEFT_ROBOT_IP = "127.0.0.1"  # 默认值
RIGHT_ROBOT_IP = "127.0.0.1"  # 默认值
WINDOW_WIDTH = 750
WINDOW_HEIGHT = 300
FONT_SIZE = 18
GAMEPAD_SPEED_XY = 40.0
GAMEPAD_SPEED_Z = 30.0
ACC = 100
AROT = 10
T = 0.1
TRIGGER_AXIS_THRESHOLD = 0.1  # 将从 config 读取或使用默认

# --- Xbox 手柄按钮和轴的索引 (作为参考，但实际映射来自 config) ---
A_BUTTON = 0
B_BUTTON = 1
X_BUTTON = 2
Y_BUTTON = 3
LEFT_BUMPER = 4
RIGHT_BUMPER = 5
START_BUTTON = 7  # 默认退出键
LEFT_TRIGGER_AXIS = 2
RIGHT_TRIGGER_AXIS = 5
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
INFO_X_MARGIN = 20
INFO_Y_START = 20
LINE_SPACING = 25
# -- 功能函数 --
def load_config(filepath):
    """加载 YAML 配置文件并设置全局变量"""
    try:
        with open(filepath, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)
            if config is None:
                print(f"错误: 配置文件 {filepath} 为空或格式错误。")
                return None
            print(f"成功加载配置文件: {filepath}")

            # --- 应用 Setup 配置 ---
            global FONT_PATH, LEFT_ROBOT_IP, RIGHT_ROBOT_IP
            global WINDOW_WIDTH, WINDOW_HEIGHT, FONT_SIZE
            if 'setup' in config:
                setup = config['setup']
                FONT_PATH = setup.get('font_path')  # 可能为 None
                LEFT_ROBOT_IP = setup.get('left_robot_ip', LEFT_ROBOT_IP)
                RIGHT_ROBOT_IP = setup.get('right_robot_ip', RIGHT_ROBOT_IP)
                WINDOW_WIDTH = setup.get('window_width', WINDOW_WIDTH)
                WINDOW_HEIGHT = setup.get('window_height', WINDOW_HEIGHT)
                FONT_SIZE = setup.get('font_size', FONT_SIZE)
                print(f"  字体路径: {FONT_PATH}")
                print(f"  左臂 IP: {LEFT_ROBOT_IP}")
                print(f"  右臂 IP: {RIGHT_ROBOT_IP}")
            else:
                print("警告: 配置文件中缺少 'setup' 部分，将使用默认值。")

            # --- 应用 Settings 配置 ---
            global GAMEPAD_SPEED_XY, GAMEPAD_SPEED_Z, ACC, AROT, T, TRIGGER_AXIS_THRESHOLD
            if 'settings' in config:
                settings = config['settings']
                GAMEPAD_SPEED_XY = settings.get('xy_speed', GAMEPAD_SPEED_XY)
                GAMEPAD_SPEED_Z = settings.get('z_speed', GAMEPAD_SPEED_Z)
                ACC = settings.get('acc', ACC)
                AROT = settings.get('arot', AROT)
                T = settings.get('t', T)
                TRIGGER_AXIS_THRESHOLD = settings.get('trigger_threshold', TRIGGER_AXIS_THRESHOLD)
                # 更新 controls 中 axis 的默认阈值
                if 'controls' in config:
                    for action, control in config['controls'].items():
                        if control.get('type') == 'axis' and 'threshold' not in control:
                            control['threshold'] = TRIGGER_AXIS_THRESHOLD

            return config
    except FileNotFoundError:
        print(f"错误: 配置文件未找到: {filepath}")
        return None
    except yaml.YAMLError as e:
        print(f"错误: 解析配置文件 {filepath} 失败: {e}")
        return None
    except Exception as e:
        print(f"加载配置文件时发生未知错误: {e}")
        return None


def format_speed(speed_array):
    # (与之前相同)
    return f"[{speed_array[0]:>6.1f}, {speed_array[1]:>6.1f}, {speed_array[2]:>6.1f}, {speed_array[3]:>6.1f}, {speed_array[4]:>6.1f}, {speed_array[5]:>6.1f}]"


def initialize_robot(controller, arm_name):
    # (与之前相同)
    print(f"\n--- 初始化 {arm_name} ---")
    try:
        print(f"[{arm_name}] 步骤 1/4: 上电...");
        if not controller.power_on(): raise RuntimeError(f"{arm_name} 上电失败")
        print(f"[{arm_name}] 上电成功.");
        time.sleep(1)
        print(f"[{arm_name}] 步骤 2/4: 清除报警...");
        controller.clearAlarm()
        print(f"[{arm_name}] 清除报警完成.");
        time.sleep(1)
        print(f"[{arm_name}] 步骤 3/4: 同步电机状态...");
        if not controller.setMotorStatus():
            print(f"警告: {arm_name} 同步电机状态失败.")
        else:
            print(f"[{arm_name}] 同步电机状态成功."); time.sleep(1)
        print(f"[{arm_name}] 步骤 4/4: 伺服使能...");
        if not controller.set_servo(1): raise RuntimeError(f"{arm_name} 伺服使能失败")
        print(f"[{arm_name}] 伺服使能成功.")
        print(f"--- {arm_name} 初始化完成 ---")
        return True
    except Exception as e:
        print(f"错误: 初始化 {arm_name} 时发生异常:"); traceback.print_exc(); return False


# -- 加载配置 --
config = load_config(CONFIG_FILE)
if config is None or 'controls' not in config:
    print("错误：配置文件加载失败或格式不正确。程序退出。")
    exit()
controls_map = config.get('controls', {})
quit_control = config.get('quit_button', {'type': 'button', 'index': START_BUTTON})  # 默认退出键

# -- 初始化 Pygame、字体和手柄 --
pygame.init()
pygame.font.init()
# 使用从配置加载的窗口尺寸
screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
pygame.display.set_caption("双臂手柄控制 (配置文件)")

# --- 加载字体 (使用从配置加载的路径和大小) ---
info_font = None
if FONT_PATH and os.path.exists(FONT_PATH):
    try:
        info_font = pygame.font.Font(FONT_PATH, FONT_SIZE)
        print(f"成功加载字体: {FONT_PATH} (大小: {FONT_SIZE})")
    except Exception as e:
        print(f"加载字体 {FONT_PATH} (大小: {FONT_SIZE}) 失败: {e}")
elif FONT_PATH:
    print(f"错误: 字体文件未找到: {FONT_PATH}")
else:
    print("警告: 配置文件中未指定 'font_path'。")

if info_font is None:
    print(f"警告: 无法加载字体，将使用 Pygame 默认字体 (大小: {FONT_SIZE})。可能无法显示中文。")
    try:
        info_font = pygame.font.Font(None, FONT_SIZE)
    except Exception as e:
        print(f"加载 Pygame 默认字体失败: {e}")
        # 最终回退：如果连默认字体都失败
        print("错误：无法加载任何字体，文本将无法显示。")
        pygame.quit()
        exit()

clock = pygame.time.Clock()

# --- 初始化手柄 ---
joystick_count = pygame.joystick.get_count()
if joystick_count == 0: print("错误：未检测到手柄！"); pygame.quit(); exit()
joystick = pygame.joystick.Joystick(0);
joystick.init()
print(f"已初始化手柄: {joystick.get_name()}")
num_hats = joystick.get_numhats();
num_axes = joystick.get_numaxes();
num_buttons = joystick.get_numbuttons()

print("机器人初始化信息将打印到控制台。")
print(f"使用 '{CONFIG_FILE}' 配置控制。按 Esc 或配置文件中定义的退出键退出。")
time.sleep(1)

# -- 连接和初始化机器人 (使用从配置加载的 IP) --
controller_left = None;
controller_right = None
left_initialized = False;
right_initialized = False
try:
    print(f"正在连接左臂 ({LEFT_ROBOT_IP})...");
    controller_left = CPSClient(LEFT_ROBOT_IP)
    if not controller_left.connect(): raise ConnectionError("左臂连接失败")
    print("左臂连接成功.")
    print(f"正在连接右臂 ({RIGHT_ROBOT_IP})...");
    controller_right = CPSClient(RIGHT_ROBOT_IP)
    if not controller_right.connect(): raise ConnectionError("右臂连接失败")
    print("右臂连接成功.")

    left_initialized = initialize_robot(controller_left, "左臂")
    right_initialized = initialize_robot(controller_right, "右臂")
    if not (left_initialized and right_initialized): raise RuntimeError("机器人初始化失败")
    print("\n双臂初始化完成，根据配置文件控制...")

    # -- 主循环 --
    running = True
    while running:
        # --- 事件处理 ---
        quit_triggered = False
        for event in pygame.event.get():
            if event.type == pygame.QUIT: running = False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE: running = False
            # 检查配置文件中定义的退出按钮
            if event.type == pygame.JOYBUTTONDOWN:
                if quit_control.get('type') == 'button' and event.button == quit_control.get('index', -1):
                    running = False

        # --- 计算速度 (根据配置文件) ---
        speed_left_cmd = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        speed_right_cmd = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        for action, control in controls_map.items():
            active = False
            # 检查索引是否有效
            ctrl_type = control.get('type')
            ctrl_index = control.get('index', -1)
            if ctrl_type == 'button' and (ctrl_index < 0 or ctrl_index >= num_buttons): continue
            if ctrl_type == 'axis' and (ctrl_index < 0 or ctrl_index >= num_axes): continue
            if ctrl_type == 'hat' and (ctrl_index < 0 or ctrl_index >= num_hats): continue

            # 获取状态
            if ctrl_type == 'button':
                active = joystick.get_button(ctrl_index) == 1
            elif ctrl_type == 'axis':
                axis_val = joystick.get_axis(ctrl_index)
                threshold = control.get('threshold', TRIGGER_AXIS_THRESHOLD)
                direction = control.get('direction', 1)
                # 修正扳机键判断逻辑：静止是-1，按下是向+1移动
                if direction == 1 and axis_val > (-1.0 + threshold * 2.0):  # 值大于 (-1 + 2*阈值) 才算按下
                    active = True
                elif direction == -1 and axis_val < (1.0 - threshold * 2.0):  # 假设负向是值小于 (1 - 2*阈值)
                    active = True

            elif ctrl_type == 'hat':
                hat_val = joystick.get_hat(ctrl_index)
                hat_axis = control.get('axis', 'x')
                direction = control.get('direction', 1)
                if hat_axis == 'x' and hat_val[0] == direction:
                    active = True
                elif hat_axis == 'y' and hat_val[1] == direction:
                    active = True

            # 应用速度
            if active:
                speed = GAMEPAD_SPEED_XY
                target_speed_array = None
                axis_index = -1
                if 'left_arm' in action:
                    target_speed_array = speed_left_cmd
                elif 'right_arm' in action:
                    target_speed_array = speed_right_cmd

                if target_speed_array is not None:
                    if '_x_pos' in action:
                        axis_index = 0; speed = GAMEPAD_SPEED_XY
                    elif '_x_neg' in action:
                        axis_index = 0; speed = -GAMEPAD_SPEED_XY
                    elif '_y_pos' in action:
                        axis_index = 1; speed = GAMEPAD_SPEED_XY
                    elif '_y_neg' in action:
                        axis_index = 1; speed = -GAMEPAD_SPEED_XY
                    elif '_z_pos' in action:
                        axis_index = 2; speed = GAMEPAD_SPEED_Z
                    elif '_z_neg' in action:
                        axis_index = 2; speed = -GAMEPAD_SPEED_Z

                    if axis_index != -1: target_speed_array[axis_index] = speed

        # --- 应用坐标系旋转 ---
        speed_left_final = speed_left_cmd.copy();
        speed_right_final = speed_right_cmd.copy()
        try:
            rot_left = R.from_euler('xyz', [65, 0, 10], degrees=True).as_matrix(); speed_left_final[
                                                                                   0:3] = speed_left_cmd[0:3] @ rot_left
        except Exception:
            speed_left_final[0:3] = [0, 0, 0]
        try:
            rot_right = R.from_euler('xyz', [65.33430565, -4.20854252, -9.07946747],
                                     degrees=True).as_matrix(); speed_right_final[0:3] = speed_right_cmd[
                                                                                         0:3] @ rot_right
        except Exception:
            speed_right_final[0:3] = [0, 0, 0]
        speed_left_final[3:] = [0.0, 0.0, 0.0];
        speed_right_final[3:] = [0.0, 0.0, 0.0]

        # --- 绘制图形界面 ---
        screen.fill(BLACK)
        lines_to_draw = []
        lines_to_draw.append(
            f"--- 双臂手柄控制 (配置文件: {CONFIG_FILE}) --- (Esc/{quit_control['type'].upper()} {quit_control['index']} 退出)")
        lines_to_draw.append(
            f"左臂({LEFT_ROBOT_IP}): {'已初始化' if left_initialized else '错误'} | 右臂({RIGHT_ROBOT_IP}): {'已初始化' if right_initialized else '错误'}")
        lines_to_draw.append("-" * 80)
        lines_to_draw.append("[当前速度指令 (已发送)]")
        lines_to_draw.append(f"  左臂速度: {format_speed(speed_left_final)}")
        lines_to_draw.append(f"  右臂速度: {format_speed(speed_right_final)}")
        lines_to_draw.append("-" * 80)
        lines_to_draw.append("提示: 控制方式已在 config.yaml 中定义")

        y_pos = INFO_Y_START
        for i, line in enumerate(lines_to_draw):
            try:
                text_surface = info_font.render(line, True, WHITE)
                if "状态:" in line or "错误" in line:  # 简化状态行颜色判断
                    if "错误" in line:
                        text_surface = info_font.render(line, True, RED)
                    else:
                        text_surface = info_font.render(line, True, GREEN)
                screen.blit(text_surface, (INFO_X_MARGIN, y_pos))
            except Exception as render_e:
                if i == 0: print(f"渲染文本时出错: {render_e}")
                try:
                    error_font = pygame.font.Font(None, FONT_SIZE); error_surface = error_font.render(
                        "! FONT RENDER ERROR !", True, RED); screen.blit(error_surface, (INFO_X_MARGIN, y_pos))
                except:
                    pass
            y_pos += LINE_SPACING

        pygame.display.flip()

        # --- 发送速度命令 (已启用) ---
        suc_l, res_l, _ = controller_left.moveBySpeedl(list(speed_left_final), ACC, AROT, T)
        suc_r, res_r, _ = controller_right.moveBySpeedl(list(speed_right_final), ACC, AROT, T)

        # --- 控制帧率 ---
        clock.tick(30)

except KeyboardInterrupt:
    print("\n接收到 Ctrl+C，正在停止...")
except (ConnectionError, RuntimeError, Exception) as e:
    print(f"\n发生错误: {e}"); traceback.print_exc()
finally:
    # -- 清理 --
    # (与之前相同)
    print("\n正在停止机器人并执行清理操作...")
    stop_speed = [0.0] * 6
    try:
        if left_initialized and controller_left and controller_left.sock:
            controller_left.moveBySpeedl(stop_speed, ACC, AROT, T)
            # print("尝试伺服下使能 左臂..."); controller_left.set_servo(0); time.sleep(0.5)
            # print("尝试下电 左臂..."); controller_left.power_off(); time.sleep(0.5)
        if right_initialized and controller_right and controller_right.sock:
            controller_right.moveBySpeedl(stop_speed, ACC, AROT, T)
            # print("尝试伺服下使能 右臂..."); controller_right.set_servo(0); time.sleep(0.5)
            # print("尝试下电 右臂..."); controller_right.power_off(); time.sleep(0.5)
    except Exception as stop_e:
        print(f"发送停止指令或下电时出错: {stop_e}")
    if controller_left and controller_left.sock:
        try:
            controller_left.disconnect(); print("左臂已断开。")
        except Exception as dis_e:
            print(f"断开左臂时出错: {dis_e}")
    if controller_right and controller_right.sock:
        try:
            controller_right.disconnect(); print("右臂已断开。")
        except Exception as dis_e:
            print(f"断开右臂时出错: {dis_e}")
    pygame.quit()
    print("程序退出。")