import socket
import json
import time
import numpy as np
import ast
# 假设新的 Gripper 类在这个路径下 (与 CPSClient 在同一目录或已正确安装)
from elibot.Jodell_gripper import Gripper  # <<< 确保这里的 Gripper 是你修改后的版本
from scipy.spatial.transform import Rotation as R
import traceback  # 导入 traceback 模块


class CPSClient:
    """
    此类用于通过TCP/IP控制Elibot机器人，并通过TCI接口与Jodell夹爪通信。
    适配了新的 Gripper 类，该类负责生成Modbus命令的十六进制字符串。
    """

    def __init__(self, ip, port=8055, gripper_slave_id=0x09):
        """
        初始化CPS客户端。
        Args:
            ip (str): 机器人控制器的IP地址。
            port (int): 机器人控制器的端口号 (默认为8055)。
            gripper_slave_id (int): Jodell夹爪的Modbus从站ID (默认为9)。
        """
        self.ip = ip
        self.port = port
        self.sock = None
        self.tci_opened = False  # 用于跟踪TCI接口状态
        # 确保 Gripper 类被正确实例化，并传入slave_id
        try:
            self.gripper = Gripper(slave_id=gripper_slave_id)  # <<< 使用新的 Gripper 类
            print(f"夹爪命令生成器已初始化 (从站ID: {gripper_slave_id})。")
        except Exception as e:
            print(f"初始化夹爪类时出错: {e}")
            self.gripper = None  # 设置为 None 以便后续检查

    # ============================================
    # == 机器人本体控制方法 (基本保持不变) ==
    # ============================================
    def connect(self):
        print(f"尝试连接到机器人 {self.ip}:{self.port}...")
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            # 设置一个超时时间，例如 5 秒
            self.sock.settimeout(5.0)
            self.sock.connect((self.ip, self.port))
            print("机器人连接成功！")
            return True
        except socket.timeout:
            print("连接机器人超时！")
            if self.sock:
                self.sock.close()
            self.sock = None
            return False
        except Exception as e:
            print(f"连接机器人时发生错误: {e}")
            if self.sock:
                self.sock.close()
            self.sock = None
            return False

    def send_power_on_cmd(self):
        print("发送机器人上电指令...")
        method = "set_robot_power_status"
        params = {"status": 1}
        ret, result, id = self.sendCMD(method, params)
        print(f"上电指令回复: ret={ret}, result={result}")
        return result

    def send_power_off_cmd(self):
        print("发送机器人下电指令...")
        method = "set_robot_power_status"
        params = {"status": 0}
        ret, result, id = self.sendCMD(method, params)
        print(f"下电指令回复: ret={ret}, result={result}")
        return result

    def get_power_status(self):
        # print("查询机器人电源状态...") # 查询频繁，可注释掉
        method = "get_robot_power_status"
        ret, result, id = self.sendCMD(method)
        if ret and result == "2":  # 假设 '2' 代表已上电
            # print("电源状态: 已上电")
            return True
        else:
            # print(f"电源状态: 未上电或查询失败 (ret={ret}, result={result})")
            return False

    def power_on(self):
        print("--- 开始执行上电流程 ---")
        if not self.get_power_status():
            self.send_power_on_cmd()
            start_time = time.time()
            while True:
                result = self.get_power_status()
                if result:
                    print("机器人上电成功！")
                    break
                elif time.time() - start_time >= 30:
                    print("尝试上电30秒失败！")
                    break
                else:
                    print("上电检测中...")
                    time.sleep(1)  # 等待1秒再查询
            return result
        else:
            print("机器人已处于上电状态。")
            return True

    def power_off(self):
        print("--- 开始执行下电流程 ---")
        if self.get_power_status():
            self.send_power_off_cmd()
            start_time = time.time()
            while True:
                result = self.get_power_status()
                if not result:
                    print("机器人下电成功！")
                    break
                elif time.time() - start_time >= 30:
                    print("尝试下电30秒失败！")
                    break
                else:
                    print("下电检测中...")
                    time.sleep(1)  # 等待1秒再查询
            return not result
        else:
            print("机器人已处于下电状态。")
            return True

    def clearAlarm(self):
        print("--- 开始清除报警 ---")
        ret, result, id = self.sendCMD("clearAlarm")
        print(f"清除报警指令回复: ret={ret}, result={result}")
        if (ret == True):
            print("清除报警指令发送成功，等待抱闸释放...")
            brakeopen = []
            b_sum = 0
            start_time = time.time()
            # brakeopen 为6个轴抱闸打开情况，打开为 1，不然为 0，全部打开为[1, 1, 1, 1, 1, 1]
            while b_sum != 6:
                if time.time() - start_time > 10:  # 增加超时检查
                    print("等待抱闸释放超时(10秒)!")
                    break
                # 获取伺服抱闸打开情况
                suc, result_brake, id_brake = self.sendCMD("get_servo_brake_off_status")
                if suc:
                    try:
                        brakeopen = json.loads(result_brake)
                        b_sum = sum(brakeopen)  # 使用 sum() 简化求和
                        # print(f"当前抱闸状态: {brakeopen}, 已打开 {b_sum} 个") # 减少打印
                    except json.JSONDecodeError:
                        print(f"解析抱闸状态失败: {result_brake}")
                        brakeopen = []
                        b_sum = 0
                    except TypeError:
                        print(f"抱闸状态返回值类型错误: {result_brake} (类型: {type(result_brake)})")
                        brakeopen = []
                        b_sum = 0
                else:
                    print(f"获取抱闸状态失败: {result_brake}")
                time.sleep(0.2)  # 稍微增加检查间隔
                # 等待 6 个轴抱闸全部打开
            if b_sum == 6:
                print(f"所有机械臂抱闸已释放: {brakeopen}")
            else:
                print(f"抱闸未能全部释放 (状态: {brakeopen})")
            time.sleep(0.1)
        else:
            # result 可能已经是 error 字典
            err_msg = result.get('message', str(result)) if isinstance(result, dict) else str(result)
            print(f"清除报警指令失败: {err_msg}")

    def get_servo_status(self):
        # print("查询伺服状态...") # 查询频繁，可注释掉
        method = "getServoStatus"
        ret, result, id = self.sendCMD(method)
        # Elibot 返回的是字符串 "true" 或 "false"
        if ret and result == "true":
            # print("伺服状态: 已使能")
            return True
        else:
            # print(f"伺服状态: 未使能或查询失败 (ret={ret}, result={result})")
            return False

    def send_set_servo_cmd(self, status=1):  # 0为关，1为开
        cmd_text = "使能" if status == 1 else "下使能"
        print(f"发送伺服{cmd_text}指令 (status={status})...")
        method = "set_servo_status"
        params = {"status": status}
        ret, result, id = self.sendCMD(method, params)
        print(f"伺服{cmd_text}指令回复: ret={ret}, result={result}")
        # Elibot 返回的是字符串 "true" 或 "false"
        if ret and result == "true":
            return True
        else:
            return False

    def setMotorStatus(self):
        print("--- 开始同步电机状态 ---")
        ret, result, id = self.sendCMD("getMotorStatus")
        print(f"获取当前电机同步状态: ret={ret}, result={result}")
        # Elibot 返回 "true" 或 "false" 字符串
        if ret and result == "false":
            print("电机状态未同步，尝试同步...")
            ret1, result1, id1 = self.sendCMD("syncMotorStatus")
            print(f"同步电机状态指令回复: ret={ret1}, result={result1}")
            if (ret1 == True):
                print("同步电机状态成功！")
                return True
            else:
                err_msg = result1.get('message', str(result1)) if isinstance(result1, dict) else str(result1)
                print(f"同步电机状态失败: {err_msg}")
                return False
        elif ret and result == "true":
            print("电机状态已同步。")
            return True
        else:
            err_msg = result.get('message', str(result)) if isinstance(result, dict) else str(result)
            print(f"获取电机同步状态失败: {err_msg}")
            return False

    def set_servo(self, status=1):
        cmd_text = "使能" if status == 1 else "下使能"
        print(f"--- 开始执行伺服{cmd_text}流程 ---")
        current_servo_status = self.get_servo_status()
        target_status_bool = (status == 1)

        if current_servo_status == target_status_bool:
            print(f"伺服已经是目标状态 ({cmd_text}状态)。")
            return True

        if not self.send_set_servo_cmd(status):
            print(f"发送伺服{cmd_text}指令失败。")
            return False

        # 等待状态改变
        start_time = time.time()
        while True:
            result = self.get_servo_status()
            if result == target_status_bool:
                print(f"伺服{cmd_text}成功！")
                break
            elif time.time() - start_time >= 30:
                print(f"已尝试30秒，伺服{cmd_text}失败！")
                break
            else:
                print(f"正在等待伺服{cmd_text}状态确认...")
                time.sleep(1)
        return self.get_servo_status() == target_status_bool

    def disconnect(self):
        print("--- 开始断开连接 ---")
        if self.sock:
            # 关闭 TCI 接口（如果打开了）
            if hasattr(self, 'tci_opened') and self.tci_opened:
                print("正在关闭 TCI 接口...")
                self.close_tci()  # 调用关闭 TCI 的方法
            print("正在关闭socket连接...")
            try:
                # 尝试优雅关闭
                self.sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass  # 可能已经关闭了
            finally:
                self.sock.close()
                print("Socket连接已关闭。")
                self.sock = None
        else:
            print("连接已经断开或未建立。")
        print("断开连接流程结束。")

    def sendCMD(self, cmd, params=None, id=1):
        """向机器人控制器发送JSON-RPC命令。"""
        if not self.sock:
            print(f"错误: Socket未连接，无法发送命令 '{cmd}'")
            return False, "Socket not connected", None
        if not params:
            params = {}
        sendStr = json.dumps({
            "jsonrpc": "2.0",
            "method": cmd,
            "params": params,
            "id": id
        }) + "\n"
        # print(f"发送指令: {sendStr.strip()}") # 调试: 打印发送的 JSON
        try:
            self.sock.sendall(sendStr.encode('utf-8'))
            # 增加接收缓冲区大小，以防回复过长
            ret = self.sock.recv(4096)
            ret_decoded = ret.decode('utf-8')
            # print(f"收到原始回复: {ret_decoded}") # 调试: 打印原始回复
            jdata = json.loads(ret_decoded)
            # print(f"解析后JSON: {jdata}") # 调试: 打印解析后的 JSON
            if "result" in jdata:
                return True, jdata["result"], jdata["id"]
            elif "error" in jdata:
                print(f"指令 '{cmd}' 返回错误: {jdata['error']}")  # 打印具体错误
                return False, jdata["error"], jdata["id"]
            else:
                print(f"警告: 指令 '{cmd}' 的回复格式异常: {jdata}")
                return False, "Unexpected response format", None
        except socket.timeout:
            print(f"错误: Socket接收指令 '{cmd}' 的回复超时")
            # self.disconnect() # 可以考虑在这里断开
            return False, "Socket recv timed out", None
        except socket.error as e:
            print(f"错误: Socket在发送/接收指令 '{cmd}' 时出错: {e}")
            # self.disconnect() # 可以考虑在这里断开
            return False, str(e), None
        except json.JSONDecodeError as e:
            print(f"错误: 解析指令 '{cmd}' 的JSON回复失败: {ret_decoded}")
            return False, f"JSON Decode Error: {e}", None
        except Exception as e:
            print(f"错误: 执行 sendCMD 处理指令 '{cmd}' 时发生意外错误:")
            traceback.print_exc()
            return False, str(e), None

    # --- 机器人运动控制方法 (保持不变) ---
    def getJointPos(self):
        # ... (代码保持不变) ...
        # print("获取当前关节角度...") # 按需取消注释
        suc, joint_pose, id = self.sendCMD("get_joint_pos")
        if suc:
            try:
                # Elibot 返回的是 JSON 字符串 '[...]'
                joint_pose_list = json.loads(joint_pose)
                # print(f"当前关节角度: {joint_pose_list}")
                return joint_pose_list
            except json.JSONDecodeError:
                print(f"解析关节角度失败: {joint_pose}")
                return None
            except TypeError:
                print(f"获取关节角度返回值类型错误: {joint_pose} (类型: {type(joint_pose)})")
                return None
        else:
            print(f"获取关节角度失败: {joint_pose}")  # joint_pose 此时是错误信息
            return None

    def getTCPPose(self, unit_type=0):
        # ... (代码保持不变) ...
        # unit_type: 0 for mm/degree, 1 for m/radian
        unit_str = "毫米/度" if unit_type == 0 else "米/弧度"
        # print(f"获取当前TCP位姿 ({unit_str})...") # 按需取消注释
        params = {"unit_type": unit_type}
        suc, result_pose, _ = self.sendCMD("getTcpPose", params=params)
        if suc:
            try:
                # Elibot 返回的是 JSON 字符串 '[x,y,z,rx,ry,rz]'
                pose_list = json.loads(result_pose)
                # print(f"当前TCP位姿: {pose_list}")
                return pose_list
            except json.JSONDecodeError:
                print(f"解析TCP位姿失败: {result_pose}")
                return None
            except TypeError:
                print(f"获取TCP位姿返回值类型错误: {result_pose} (类型: {type(result_pose)})")
                return None
        else:
            print(f"获取TCP位姿失败: {result_pose}")  # result_pose 此时是错误信息
            return None

    def moveByJoint(self, target_joint, speed=10, block=True):
        # ... (代码保持不变) ...
        print(f"--- 开始关节运动 MoveByJoint ---")
        print(f"目标关节角度: {target_joint}, 速度: {speed}, 是否阻塞: {block}")
        # J1 轴限位检查 (左手机器人)
        if target_joint[0] > 260 or target_joint[0] < 80:  # J1 范围 ±120
            print(f"警告: J1 ({target_joint[0]}) 超出常见范围，请确认。")
            raise Exception("Joint1 over limit!", target_joint[0])

        print("发送 MoveByJoint 指令...")
        suc, result, _ = self.sendCMD("moveByJoint", {"targetPos": target_joint, "speed": speed})
        if suc:
            print(f"MoveByJoint 指令发送成功。")
            if block:
                print("等待机器人运动停止...")
                start_time = time.time()
                while True:
                    if time.time() - start_time > 180:  # 增加超时(3分钟)
                        print("错误: 等待机器人停止超时 (180秒)!")
                        return False  # 表示运动未确认完成
                    suc_state, state, _ = self.sendCMD("getRobotState")
                    if suc_state and state == '0':
                        print("机器人已停止，运动完成。")
                        current_pos = self.getJointPos()
                        # print(f"当前关节位置: {current_pos}") # 减少打印
                        return True  # 表示运动完成
                    elif not suc_state:
                        print("获取机器人状态失败，无法确认运动是否完成。")
                        time.sleep(1)  # 获取失败稍等重试
                    else:
                        # print(f"机器人运动中 (状态: {state})...") # 打印状态信息
                        time.sleep(0.2)  # 运动中检查间隔可以短一些
            else:
                print("非阻塞模式，指令已发送。")
                return True  # 非阻塞模式，发送成功即返回 True
        else:
            err_msg = result.get('message', str(result)) if isinstance(result, dict) else str(result)
            print(f"发送 MoveByJoint 指令失败: {err_msg}")
            return False

    def moveByJoint_right(self, target_joint, speed=10, block=True):
        # ... (代码保持不变) ...
        print(f"--- 开始关节运动 MoveByJoint (右手机器人逻辑) ---")
        print(f"目标关节角度: {target_joint}, 速度: {speed}, 是否阻塞: {block}")
        if target_joint[0] > -60 or target_joint[0] < -260:  # J1 范围 ±175
            print(f"警告: J1 ({target_joint[0]}) 超出常见范围，请确认。")
            raise Exception("Joint1 over limit!", target_joint[0])

        print("发送 MoveByJoint 指令...")
        suc, result, _ = self.sendCMD("moveByJoint", {"targetPos": target_joint, "speed": speed})
        if suc:
            print(f"MoveByJoint 指令发送成功。")
            if block:
                print("等待机器人运动停止...")
                start_time = time.time()
                while True:
                    if time.time() - start_time > 180:  # 增加超时(3分钟)
                        print("错误: 等待机器人停止超时 (180秒)!")
                        return False
                    suc_state, state, _ = self.sendCMD("getRobotState")
                    if suc_state and state == '0':
                        print("机器人已停止，运动完成。")
                        current_pos = self.getJointPos()
                        # print(f"当前关节位置: {current_pos}") # 减少打印
                        return True
                    elif not suc_state:
                        print("获取机器人状态失败，无法确认运动是否完成。")
                        time.sleep(1)
                    else:
                        # print(f"机器人运动中 (状态: {state})...")
                        time.sleep(0.2)
            else:
                print("非阻塞模式，指令已发送。")
                return True
        else:
            err_msg = result.get('message', str(result)) if isinstance(result, dict) else str(result)
            print(f"发送 MoveByJoint 指令失败: {err_msg}")
            return False

    def moveBySpeedl(self, speed_l, acc, arot, t, id=1):
        # ... (代码保持不变) ...
        print(f"--- 开始速度控制运动 MoveBySpeedl ---")
        print(f"速度向量: {speed_l}, 线性加速度: {acc}, 旋转加速度: {arot}, 持续时间: {t}")
        params = {"v": speed_l, "acc": acc, "arot": arot, "t": t}
        print("发送 MoveBySpeedl 指令...")
        ret, result, ret_id = self.sendCMD("moveBySpeedl", params, id)
        print(f"MoveBySpeedl 指令回复: ret={ret}, result={result}, id={ret_id}")
        if not ret:
            err_msg = result.get('message', str(result)) if isinstance(result, dict) else str(result)
            print(f"发送 MoveBySpeedl 指令失败: {err_msg}")
        return ret, result, ret_id

    def inverseKinematic(self, targetPose, unit_type=0):
        # ... (代码保持不变) ...
        unit_str = "毫米/度" if unit_type == 0 else "米/弧度"
        print(f"--- 开始逆运动学计算 IK ---")
        print(f"目标位姿: {targetPose} ({unit_str})")
        referencePos = self.getJointPos()
        if referencePos is None:
            print("错误：无法获取当前关节位置作为IK参考！")
            return None
        print(f"参考关节位置: {referencePos}")
        targetPose_list = targetPose.tolist() if isinstance(targetPose, np.ndarray) else list(targetPose)
        params = {"targetPose": targetPose_list, "referencePos": referencePos, "unit_type": unit_type}
        print("发送 inverseKinematic 指令...")
        suc, iK_joint_str, _ = self.sendCMD("inverseKinematic", params)
        if suc:
            try:
                iK_joint = json.loads(iK_joint_str)
                print(f"IK 计算成功，结果关节角度: {iK_joint}")
                return iK_joint
            except json.JSONDecodeError:
                print(f"IK 计算失败或返回非JSON格式: {iK_joint_str}")
                return None
            except TypeError:
                print(f"IK 计算返回值类型错误: {iK_joint_str} (类型: {type(iK_joint_str)})")
                return None
        else:
            print(f"发送 inverseKinematic 指令失败: {iK_joint_str}")
            return None

    def move_robot(self, target_pose, speed=10, block=True):
        # ... (代码保持不变) ...
        print(f"--- 开始 Move Robot (IK + MoveByJoint) ---")
        print(f"目标位姿: {target_pose}, 速度: {speed}, 是否阻塞: {block}")
        iK_joint = self.inverseKinematic(target_pose)
        if iK_joint is None:
            print("错误：逆运动学计算失败，无法移动。")
            return False
        current_pos = self.getJointPos()
        if current_pos is None:
            print("警告：无法获取当前关节位置，跳过角度差值检查。")
        else:
            for idx, (ik_val, pos_val) in enumerate(zip(iK_joint, current_pos)):
                diff = abs(ik_val - pos_val)
                if diff > 90:
                    raise Exception(f"警告：第 {idx + 1} 个关节角度差值 {diff:.2f} 度超过 90 度！")
        print(f"开始使用计算出的关节角度移动: {iK_joint}")
        return self.moveByJoint(iK_joint, speed=speed, block=block)

    def move_right_robot(self, target_pose, speed=10, block=True):
        # ... (代码保持不变) ...
        print(f"--- 开始 Move Right Robot (IK + MoveByJoint_right) ---")
        print(f"目标位姿: {target_pose}, 速度: {speed}, 是否阻塞: {block}")
        iK_joint = self.inverseKinematic(target_pose)
        if iK_joint is None:
            print("错误：逆运动学计算失败，无法移动。")
            return False
        current_pos = self.getJointPos()
        if current_pos is None:
            print("警告：无法获取当前关节位置，跳过角度差值检查。")
        else:
            try:
                for idx, (ik_val, pos_val) in enumerate(zip(iK_joint, current_pos)):
                    diff = abs(ik_val - pos_val)
                    if diff > 180: print(f"警告：第 {idx + 1} 个关节角度差值 {diff:.2f} 度超过 180 度！")
            except Exception as e:
                print(f"检查关节角度差值时出错: {e}")
        print(f"开始使用计算出的关节角度移动 (右手机器人): {iK_joint}")
        return self.moveByJoint_right(iK_joint, speed=speed, block=block)

    def alignZAxis(self):
        # ... (代码保持不变) ...
        print("--- 开始执行 Align Z Axis 操作 ---")
        current_pose = self.getTCPPose()
        if current_pose is None:
            print("错误：无法获取当前 TCP 位姿，无法对齐 Z 轴。")
            return False
        print(f"当前位姿: {current_pose}")
        target_pose = current_pose[:3] + [180.0, 0.0, 0.0]
        print(f"目标位姿 (工具Z轴朝下): {target_pose}")
        return self.move_robot(target_pose, speed=10, block=True)

    # ============================================
    # == TCI 通信层方法 (保持不变) ==
    # ============================================
    # 这些方法负责与机器人控制器的 TCI 接口通信
    def open_tci(self):
        """打开机器人控制器上的 TCI 串口转发功能。"""
        print("尝试打开 TCI 串口...")
        suc, result, _ = self.sendCMD("open_tci")
        print(f"打开 TCI 串口结果: suc={suc}, result={result}")
        if suc:
            self.tci_opened = True  # 标记 TCI 已打开
        else:
            err_msg = result.get('message', str(result)) if isinstance(result, dict) else str(result)
            print(f"打开 TCI 失败: {err_msg}")
            self.tci_opened = False
        return suc, result

    def set_tci(self, baud_rate=115200, bits=8, event="N", stop=1):
        """设置 TCI 串口参数。"""
        print(f"尝试设置 TCI 选项: 波特率={baud_rate}, 数据位={bits}, 校验位={event}, 停止位={stop}...")
        # 注意: Elibot setopt_tci 的参数名可能不同，这里假设与你的原始代码一致
        suc, result, _ = self.sendCMD("setopt_tci",
                                      {"baud_rate": baud_rate, "bits": bits, "event": event, "stop": stop})
        print(f"设置 TCI 选项结果: suc={suc}, result={result}")
        if not suc:
            err_msg = result.get('message', str(result)) if isinstance(result, dict) else str(result)
            print(f"设置 TCI 选项失败: {err_msg}")
        return suc, result

    def recv_tci(self, count=100, hex_format=1, timeout=1000):
        """
        通过 TCI 接收数据。
        Args:
            count (int): 期望接收的最大字节数。
            hex_format (int): 接收格式 (1: 十六进制字符串, 0: ASCII)。
            timeout (int): 接收超时时间 (毫秒)。
        Returns:
            tuple: (success, result_code, size, buffer_string) 或 (False, error_msg, None, None)
                   buffer_string 是接收到的数据（根据 hex_format 决定格式）。
        """
        # print(f"尝试接收 TCI 数据: 数量={count}, 格式={'HEX' if hex_format==1 else 'ASCII'}, 超时={timeout}ms...")
        suc, result, _ = self.sendCMD("recv_tci", {"count": count, "hex": hex_format, "timeout": timeout})
        # print(f"接收 TCI 原始结果: suc={suc}, result='{result}' (类型: {type(result)})")

        if not suc:
            print(f"接收 TCI 数据失败: {result}")
            return False, None, None, None

        if isinstance(result, dict) and 'message' in result:
            print(f"接收 TCI 时控制器返回错误: {result}")
            return False, result['message'], None, None

        try:
            # Elibot 标准回复是一个 JSON 字符串
            data = json.loads(result)
            if "result" in data and "size" in data and "buf" in data:
                recv_result = data["result"]  # true/false
                recv_size = data["size"]  # 实际字节数
                recv_buf = data['buf']  # 数据 (十六进制字符串或ASCII)
                # print(f"接收 TCI 解析成功: result={recv_result}, size={recv_size}, buf='{recv_buf}'")
                return True, recv_result, recv_size, recv_buf
            else:
                print(f"接收 TCI 的 JSON 结构异常: {data}")
                return False, "Unexpected JSON structure", None, None
        except json.JSONDecodeError:
            print(f"接收 TCI 的结果不是有效的 JSON: '{result}'")
            return False, "Non-JSON response", None, None
        except Exception as e:
            print(f"处理接收 TCI 结果时发生错误: {e}")
            traceback.print_exc()
            return False, str(e), None, None

    def send_tci(self, send_buf_hex: str, hex_format=1):
        """
        通过 TCI 发送数据 (通常是十六进制字符串)。
        Args:
            send_buf_hex (str): 要发送的十六进制字符串。
            hex_format (int): 发送格式 (1: 十六进制字符串, 0: ASCII)。
        Returns:
            tuple: (success, result)
        """
        # print(f"尝试发送 TCI 数据: buf='{send_buf_hex}', 格式={'HEX' if hex_format==1 else 'ASCII'}")
        suc, result, _ = self.sendCMD("send_tci", {"send_buf": send_buf_hex, "hex": hex_format})
        # print(f"发送 TCI 数据结果: suc={suc}, result={result}")
        if not suc:
            print(f"发送 TCI 数据失败! buf='{send_buf_hex}', 错误: {result}")
        return suc, result

    def flush_tci(self):
        """清空 TCI 接收缓冲区。"""
        print("尝试清空 TCI 接收缓冲区...")
        suc, result, _ = self.sendCMD("flush_tci")
        print(f"清空 TCI 缓冲区结果: suc={suc}, result={result}")
        if not suc:
            err_msg = result.get('message', str(result)) if isinstance(result, dict) else str(result)
            print(f"清空 TCI 缓冲区失败: {err_msg}")
        return suc, result

    def close_tci(self):
        """关闭机器人控制器上的 TCI 串口转发功能。"""
        print("尝试关闭 TCI 串口...")
        suc, result, _ = self.sendCMD("close_tci")
        print(f"关闭 TCI 串口结果: suc={suc}, result={result}")
        if suc:
            self.tci_opened = False  # 标记 TCI 已关闭
        else:
            err_msg = result.get('message', str(result)) if isinstance(result, dict) else str(result)
            print(f"关闭 TCI 失败: {err_msg}")
        return suc, result

    # ============================================
    # == 夹爪控制方法 (适配新的 Gripper 类) ==
    # ============================================

    def connect_gripper(self) -> bool:
        """
        连接并激活夹爪。
        使用 self.gripper 生成命令，通过 TCI 发送/接收。
        Returns:
            bool: True 如果连接并激活成功，否则 False。
        """
        if not self.gripper:
            print("夹爪错误: Gripper 类未初始化!")
            return False

        print("--- 开始连接并激活夹爪 ---")
        # 1. 打开 TCI 接口
        suc_open, _ = self.open_tci()
        if not suc_open:
            print("连接夹爪失败：无法打开 TCI。")
            return False

        # 2. 设置 TCI 串口参数 (使用夹爪的默认参数)
        serial_params = self.gripper.serial_params
        suc_set, _ = self.set_tci(
            baud_rate=serial_params.get("baud_rate", 115200),
            bits=serial_params.get("data_bits", 8),
            event=serial_params.get("event", "N"),
            stop=serial_params.get("stop_bits", 1)
        )
        if not suc_set:
            print("连接夹爪失败：无法设置 TCI 参数。")
            self.close_tci()
            return False

        # 3. 清空 TCI 缓冲区
        self.flush_tci()
        time.sleep(0.1)  # 短暂等待

        try:
            # --- 4. 发送激活序列 ---
            # 4.1 发送复位指令 (rACT=0)
            reset_cmd_hex = self.gripper.activate_request()  # 获取复位命令的 hex 字符串
            print(f"发送夹爪复位指令 (Hex): {reset_cmd_hex}")
            suc_send_reset, _ = self.send_tci(reset_cmd_hex, hex_format=1)
            if not suc_send_reset:
                print("激活失败：发送复位指令时出错。")
                self.close_tci()
                return False
            time.sleep(0.5)  # 等待夹爪处理复位

            # 4.2 发送使能指令 (rACT=1)
            enable_cmd_hex = self.gripper.enable_gripper()  # 获取使能命令的 hex 字符串
            print(f"发送夹爪使能指令 (Hex): {enable_cmd_hex}")
            suc_send_enable, _ = self.send_tci(enable_cmd_hex, hex_format=1)
            if not suc_send_enable:
                print("激活失败：发送使能指令时出错。")
                self.close_tci()
                return False
            time.sleep(0.5)  # 等待夹爪处理使能
            # (可选) 接收并解码使能响应

            # --- 5. 轮询状态直到激活完成 ---
            print("等待夹爪激活完成...")
            start_time = time.time()
            activation_success = False
            while time.time() - start_time < 15:  # 15秒超时
                # read_gripper_state 内部处理了命令生成、发送、接收、解码
                state_tuple = self._read_and_decode_gripper_state()
                if state_tuple:
                    (activate_state, move_state, hand_state, current_position,
                     error_code, current_speed, current_force) = state_tuple

                    # 检查错误 (忽略激活过程中可能出现的 '需要激活' 错误 0x01 或 0x05?)
                    # Jodell 手册中 0x01 是需要激活位，0x05 也是类似含义
                    fault_code_to_check = error_code & ~0x01 & ~0x05  # 忽略01和05错误
                    if fault_code_to_check != 0:
                        print(f"激活失败：夹爪报错！错误码: 0x{error_code:02X}")
                        self.gripper._print_fault_description_cn(error_code)
                        return False  # 出现严重错误，激活失败

                    # 检查激活状态 (gSTA = 3 表示激活完成)
                    if activate_state == 3:
                        print("夹爪激活成功!")
                        activation_success = True
                        break
                    else:
                        print(f"夹爪仍在激活中 (激活状态码: {activate_state})...")
                else:
                    print("读取夹爪状态失败，重试...")
                    # 可以增加连续失败计数器来提前退出

                time.sleep(0.5)  # 轮询间隔

            if not activation_success:
                print("夹爪激活超时或失败!")
                # 尝试读取最后状态
                final_state = self._read_and_decode_gripper_state()
                if final_state: print(f"超时前最后读取的状态: {final_state}")
                self.close_tci()
                return False

            print("--- 夹爪连接并激活成功 ---")
            return True

        except Exception as e:
            print(f"连接或激活夹爪过程中发生意外错误:")
            traceback.print_exc()
            self.close_tci()  # 确保关闭 TCI
            return False

    def _read_and_decode_gripper_state(self, register_count=3) -> tuple | None:
        """
        内部方法：生成读取命令，发送，接收，解码，并返回状态元组。
        Args:
            register_count (int): 要读取的寄存器数量。
        Returns:
            tuple | None: 解析后的状态元组，或在失败时返回 None。
        """
        if not self.gripper or not self.tci_opened:
            # print("读取状态错误：夹爪未初始化或TCI未打开。") # 减少打印
            return None

        # 1. 生成读取命令的 Hex 字符串
        read_cmd_hex = self.gripper.read_gripper_state(register_count)

        # 2. 清空可能存在的旧数据 (可选，但读取前通常不清空)
        # self.flush_tci()

        # 3. 发送读取命令
        suc_send, _ = self.send_tci(read_cmd_hex, hex_format=1)
        if not suc_send:
            print("读取状态失败：发送读取命令时出错。")
            return None
        time.sleep(0.05)  # 短暂等待响应

        # 4. 接收响应 (需要估计合适的字节数)
        # 读取3个寄存器=6字节数据, 响应帧=Addr(1)+FC(1)+ByteCount(1)+Data(6)+CRC(2)=11字节
        # 设置 count 稍大一些，例如 30
        expected_len = 1 + 1 + 1 + (register_count * 2) + 2
        suc_recv, _, size, buf_hex = self.recv_tci(count=expected_len + 10, hex_format=1, timeout=500)

        if not suc_recv or not buf_hex:
            # print("读取状态失败：接收响应时出错或无数据。") # 减少打印
            return None

        # 5. 解码响应字符串
        fc, payload = self.gripper.decode_response(buf_hex)

        # 6. 如果解码成功且功能码正确，则解码状态数据
        if fc == 0x04 and payload:  # 检查是否为读输入寄存器成功响应
            state_tuple = self.gripper.decode_state_data(fc, payload)
            return state_tuple
        elif fc is not None:  # 收到了响应，但不是预期的 FC04
            print(f"读取状态错误：收到非预期的功能码 0x{fc:02X}")
            return None
        else:  # decode_response 失败 (例如 CRC 错误)
            print("读取状态错误：解码响应失败。")
            return None

    def run_gripper(self, target_position: int, force: int = 100, speed: int = 100, wait: bool = True,
                    timeout: int = 20) -> bool:
        """
        控制夹爪移动到指定位置。
        Args:
            target_position (int): 目标位置 (0=开, 255=关)。
            force (int): 目标力 (0-255)。
            speed (int): 目标速度 (0-255)。
            wait (bool): 是否阻塞等待运动完成 (默认 True)。
            timeout (int): 等待运动完成的超时时间 (秒, 默认 20)。
        Returns:
            bool: True 如果命令发送成功 (wait=False) 或运动成功完成 (wait=True)。
        """
        if not self.gripper:
            print("夹爪错误: Gripper 类未初始化!")
            return False
        if not self.tci_opened:
            print("夹爪错误: TCI 未打开，无法操作。")
            return False
        # if not self.is_gripper_activated(): # 需要一个检查激活状态的方法
        #     print("夹爪错误: 夹爪未激活。")
        #     return False

        action_desc = "打开" if target_position == 0 else (
            "关闭" if target_position == 255 else f"移动到 {target_position}")
        print(f"--- 开始执行夹爪操作: {action_desc} ---")

        try:
            # 1. 清空 TCI 缓冲区
            self.flush_tci()
            time.sleep(0.1)

            # 2. 生成并发送移动命令
            move_cmd_hex = self.gripper.run_gripper(target_position, force, speed)
            print(f"发送夹爪移动指令 (Hex): {move_cmd_hex}")
            suc_send, _ = self.send_tci(move_cmd_hex, hex_format=1)
            if not suc_send:
                print(f"发送夹爪移动指令失败。")
                return False
            time.sleep(0.1)  # 等待命令被接收

            # 3. (可选) 接收并解码对移动命令的响应 (通常只是确认)
            # suc_recv, _, _, buf = self.recv_tci(...)
            # if suc_recv and buf: self.gripper.decode_response(buf)

            # 4. 如果需要，等待运动完成
            if wait:
                print("等待夹爪运动完成...")
                start_time = time.time()
                movement_complete = False
                last_pos = -1
                while time.time() - start_time < timeout:
                    state_tuple = self._read_and_decode_gripper_state()
                    if state_tuple:
                        (activate_state, move_state, hand_state, current_position,
                         error_code, current_speed, current_force) = state_tuple

                        if current_position != last_pos:
                            # print(f"  夹爪移动中: 位置={current_position}, 运动状态(gGTO)={'运动' if move_state else '停止'}, 夹持状态(gOBJ)={hand_state}, 错误=0x{error_code:02X}")
                            last_pos = current_position

                        # 检查是否有错误
                        if error_code != 0:
                            print(f"夹爪运动中报错！错误码: 0x{error_code:02X}")
                            self.gripper._print_fault_description_cn(error_code)
                            return False  # 运动失败

                        # 检查是否停止 (gGTO=False)
                        if not move_state:
                            print(f"夹爪已停止。当前位置: {current_position}")
                            # 可以进一步根据 hand_state 判断停止原因
                            if hand_state == 3:
                                print("  原因: 到达目标位置 (未检测到物体)")
                            elif hand_state == 1:
                                print("  原因: 检测到物体 (内撑模式)")
                            elif hand_state == 2:
                                print("  原因: 检测到物体 (外夹模式)")
                            else:
                                print(f"  原因: 未知停止状态 (gOBJ={hand_state})")
                            movement_complete = True
                            break  # 退出等待循环
                    else:
                        print("等待运动完成时读取状态失败，重试...")
                        # 增加连续失败计数

                    time.sleep(0.3)  # 轮询间隔

                if not movement_complete:
                    print(f"夹爪运动等待超时 ({timeout}秒)!")
                    final_state = self._read_and_decode_gripper_state()
                    if final_state: print(f"超时前最后读取的状态: {final_state}")
                    return False  # 超时，运动失败

                print(f"--- 夹爪操作 ({action_desc}) 成功完成 ---")
                return True  # 运动成功完成

            else:  # 非阻塞模式
                print("移动指令已发送 (非阻塞)。")
                return True  # 命令发送成功

        except Exception as e:
            print(f"执行夹爪操作 ({action_desc}) 时发生意外错误:")
            traceback.print_exc()
            return False

    def open_gripper(self, speed=200, force=150, wait=False, timeout=10) -> bool:
        """完全打开夹爪。"""
        time.sleep(0.1)
        return self.run_gripper(target_position=0, speed=speed, force=force, wait=wait, timeout=timeout)

    def close_gripper(self, speed=200, force=150, wait=False, timeout=10) -> bool:
        """完全关闭夹爪。"""
        time.sleep(0.1)

        return self.run_gripper(target_position=255, speed=speed, force=force, wait=wait, timeout=timeout)

    def read_gripper_state_tuple(self) -> tuple | None:
        """读取并返回解析后的夹爪状态元组。"""
        return self._read_and_decode_gripper_state(register_count=3)


# --- 姿态计算函数 (保持不变) ---
def desire_left_pose(rpy_array=None):
    # ... (代码保持不变) ...
    if rpy_array is None: rpy_array = [180, 0, 180]
    base_to_flange_L = R.from_euler('xyz', [65, 4, 12], degrees=True)
    target_orientation = R.from_euler('xyz', rpy_array, degrees=True)
    flange_orientation = base_to_flange_L.inv() * target_orientation
    rpy_angles = flange_orientation.as_euler('xyz', degrees=True)
    return rpy_angles


def desire_right_pose(rpy_array=None):
    # ... (代码保持不变) ...
    if rpy_array is None: rpy_array = [180, 0, 180]
    base_to_flange_R = R.from_euler('xyz', [65.334, -4.208, -9.079], degrees=True)
    target_orientation = R.from_euler('xyz', rpy_array, degrees=True)
    flange_orientation = base_to_flange_R.inv() * target_orientation
    rpy_angles = flange_orientation.as_euler('xyz', degrees=True)
    return rpy_angles


# --- 主程序入口 ---
if __name__ == "__main__":
    robot_ip = "192.168.188.200"
    gripper_id = 9  # <<< 确认你的夹爪ID
    print(f"--- 脚本开始: 准备连接机器人 {robot_ip} ---")
    controller = CPSClient(robot_ip, gripper_slave_id=gripper_id)

    try:
        print("\n[步骤 1/6] 正在连接机器人...")
        if controller.connect():
            print("机器人连接成功。")

            print("\n[步骤 2/6] 正在初始化机器人...")
            # ... (机器人上电、清报警、电机同步、伺服使能等保持不变) ...
            if not controller.power_on(): exit()
            controller.clearAlarm()
            if not controller.setMotorStatus(): print("警告: 同步电机状态失败")
            if not controller.set_servo(1): exit()
            print("机器人初始化完成。")

            print("\n[步骤 3/6] 获取机器人初始状态...")
            # ... (获取位姿、关节角等保持不变) ...
            initial_pose = controller.getTCPPose()
            if initial_pose: print(f"初始 TCP 位姿: {[round(p, 2) for p in initial_pose]}")
            initial_joint = controller.getJointPos()
            if initial_joint: print(f"初始关节角度: {[round(j, 2) for j in initial_joint]}")

            # --- 夹爪测试 (使用新的适配方法) ---
            print("\n[步骤 4/6] 开始测试夹爪...")
            print("尝试连接并激活夹爪...")
            if controller.connect_gripper():  # connect_gripper 内部处理了打开TCI和激活
                print("\n夹爪连接并激活成功。现在执行动作测试...")
                time.sleep(1)

                # 测试案例 1: 关闭夹爪
                print("\n测试案例 1: 关闭夹爪 (位置 255)")
                if controller.close_gripper(wait=True):  # close_gripper 调用 run_gripper
                    print("关闭指令执行流程完毕。")
                    state = controller.read_gripper_state_tuple()  # 读取最终状态
                    if state: print(f"关闭后状态元组: {state}")
                else:
                    print("关闭指令执行流程失败。")
                time.sleep(2)

                # 测试案例 2: 打开夹爪
                print("\n测试案例 2: 打开夹爪 (位置 0)")
                if controller.open_gripper(wait=True):  # open_gripper 调用 run_gripper
                    print("打开指令执行流程完毕。")
                    state = controller.read_gripper_state_tuple()
                    if state: print(f"打开后状态元组: {state}")
                else:
                    print("打开指令执行流程失败。")
                time.sleep(2)

                # 测试案例 3: 移动到中间位置 (例如 128)
                print("\n测试案例 3: 移动到中间位置 (128)")
                if controller.run_gripper(target_position=128, wait=True):
                    print("移动到中间位置指令执行流程完毕。")
                    state = controller.read_gripper_state_tuple()
                    if state: print(f"移动到 128 后状态元组: {state}")
                else:
                    print("移动到中间位置指令执行流程失败。")
                time.sleep(2)

                print("\n夹爪测试结束。")
            else:
                print("错误: 连接或激活夹爪失败，跳过夹爪测试。")

            print("\n[步骤 5/6] 执行其他机器人操作 (无)...")
            # ...

            print("\n[步骤 6/6] 执行清理工作...")
            # ... (下使能, 下电等) ...
            # print("尝试伺服下使能...")
            # controller.set_servo(0)
            # time.sleep(1)
            # print("尝试机器人下电...")
            # controller.power_off()
            print("清理工作完成 (已注释掉实际操作)。")

        else:
            print(f"错误: 无法连接到机器人 {robot_ip}，脚本终止。")

    except KeyboardInterrupt:
        print("\n警告: 检测到手动中断 (Ctrl+C)。")
        # ... (中断后的清理) ...
        if controller and controller.sock:
            print("尝试在中断后进行伺服下使能和下电...")
            # controller.set_servo(0)
            # time.sleep(0.5)
            # controller.power_off()
    except Exception as e:
        print("\n错误: 主程序执行过程中发生未捕获的异常:")
        traceback.print_exc()
        # ... (异常后的清理) ...
        if controller and controller.sock:
            print("尝试在异常后进行伺服下使能和下电...")
            try:
                # controller.set_servo(0)
                # time.sleep(0.5)
                # controller.power_off()
                pass
            except Exception as cleanup_e:
                print(f"异常后的清理操作也发生错误: {cleanup_e}")
    finally:
        # --- 断开连接 ---
        print("\n--- 执行最终的断开连接操作 ---")
        if controller:
            controller.disconnect()  # disconnect 内部会尝试 close_tci
        print("--- 脚本结束 ---")
