import struct # 导入struct模块 (虽然当前代码未直接使用，但处理字节时可能有用)

class Gripper:
    """
    JODELL RG系列夹爪 Modbus RTU 命令生成器。

    此类用于生成控制夹爪所需的Modbus RTU命令帧（十六进制字符串格式），
    并提供了用于解析响应帧的方法。
    基于 JODELL RG 系列说明书 V1.3。
    """
    def __init__(self, slave_id=0x09):
        """
        初始化夹爪命令生成器。
        Args:
            slave_id (int): 夹爪的Modbus从站ID (默认为9)。
        """
        # 串口参数 (仅供参考，此类不执行实际通讯)
        self.serial_params = {
            "interface": "RS-485",      # 物理接口
            "baud_rate": 115200,        # 波特率 (默认)
            "event": "N",               # 校验位 (N=None)
            "data_bits": 8,             # 数据位
            "stop_bits": 1,             # 停止位
            "parity": "None",           # 奇偶校验
            "flow_control": "None",     # 流控
        }
        self.slave_id = slave_id        # 从站地址 (设备ID)

        # 主要寄存器地址 (根据PDF Table 6.1)
        self.REG_ACTION_CONTROL = 0x03E8  # 控制寄存器 (低字节控制，高字节预设命令)
        self.REG_POSITION_SET = 0x03E9    # 位置设置 (高字节，MODE=0时)
        self.REG_SPEED_FORCE_SET = 0x03EA # 速度(低字节)/力(高字节)设置 (MODE=0时)
        self.REG_BRAKE_CONTROL = 0x03FC   # 抱闸控制 (低字节)

        self.REG_GRIPPER_STATUS = 0x07D0  # 状态反馈寄存器起始地址
        # 其他状态寄存器... (0x07D1, 0x07D2 等)

        print(f"夹爪命令生成器已初始化。从站ID: {self.slave_id}")

    @staticmethod
    def _calculate_crc(data: list[int]) -> list[int]:
        """
        计算 Modbus RTU CRC16 校验码。
        Args:
            data: 包含从站地址、功能码和数据负载的字节列表 (整数形式)。
        Returns:
            list: 包含两个元素的列表 [CRC低字节, CRC高字节]。
        """
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc >>= 1
                    crc ^= 0xA001 # Modbus CRC多项式 A001
                else:
                    crc >>= 1
        # 返回 CRC 低字节在前，高字节在后
        return [crc & 0xFF, (crc >> 8) & 0xFF]

    def _generate_command_frame(self, function_code: int, payload: list[int]) -> list[int]:
        """
        构建完整的 Modbus RTU 命令帧 (字节列表形式)。
        自动添加从站ID和CRC校验码。
        Args:
            function_code: Modbus 功能码 (例如 0x10, 0x04)。
            payload: 功能码特定的数据负载字节列表 (整数形式)。
                     例如写命令: [寄存器地址高, 地址低, 寄存器数量高, 数量低, 字节数, 数据...]
                     例如读命令: [寄存器地址高, 地址低, 寄存器数量高, 数量低]
        Returns:
            list: 代表完整Modbus命令帧的字节列表 (整数形式)。
        """
        # 帧起始：从站ID + 功能码
        command_no_crc = [self.slave_id, function_code] + payload
        # 计算 CRC
        crc_bytes = self._calculate_crc(command_no_crc)
        # 组合完整帧：命令 + CRC (低字节在前)
        full_frame = command_no_crc + crc_bytes
        return full_frame

    def _format_hex_string(self, command_frame: list[int]) -> str:
        """将字节列表格式化为无空格的十六进制字符串。"""
        hex_string = "".join(f"{byte:02X}" for byte in command_frame)
        # 打印生成的命令帧用于调试 (中文)
        print(f"生成的命令帧 (Hex): {hex_string}")
        return hex_string

    def _check_value(self, value: int, name: str) -> int:
        """检查输入值是否在0-255之间，超出则限制并打印中文警告。"""
        if not isinstance(value, int):
             value = 0 # 如果不是整数则默认为0
             print(f"警告：参数 '{name}' 不是有效的整数，已强制设置为 0。")
        if value < 0x00:
            print(f"警告：参数 '{name}' 的值 ({value}) 过小，已设置为 0x00。")
            value = 0x00
        elif value > 0xFF:
            print(f"警告：参数 '{name}' 的值 ({value}) 过大，已设置为 0xFF。")
            value = 0xFF
        return value

    # --- 命令生成方法 (返回十六进制字符串) ---

    def force_open_brake(self, open_brake: bool = True) -> str:
        """
        生成强制打开抱闸或释放的命令帧 (FC10)。
        Args:
            open_brake (bool): True 表示强制打开 (写入0x0001)，False 表示恢复正常 (写入0x0000)。
        Returns:
            str: 代表Modbus命令的十六进制字符串。
        """
        function_code = 0x10          # 功能码: 写多个寄存器 (虽然只写一个，但示例和结构常用FC10)
        register_address = self.REG_BRAKE_CONTROL # 寄存器地址: 0x03FC
        register_count = 0x0001       # 写入寄存器数量: 1
        byte_count = 0x02             # 数据字节数: 1 * 2 = 2
        register_data = 0x0001 if open_brake else 0x0000 # 写入的数据

        payload = [
            (register_address >> 8) & 0xFF, register_address & 0xFF,    # 地址高, 地址低
            (register_count >> 8) & 0xFF, register_count & 0xFF,      # 数量高, 数量低
            byte_count,                                               # 字节数
            (register_data >> 8) & 0xFF, register_data & 0xFF         # 数据高, 数据低
        ]
        command_frame_list = self._generate_command_frame(function_code, payload)
        return self._format_hex_string(command_frame_list)

    def activate_request(self) -> str:
        """
        生成夹爪复位/取消激活请求的命令帧 (rACT=0) (FC10)。
        目标寄存器 0x03E8，写入值 0x0000。
        示例帧: 09 10 03 E8 00 01 02 00 00 E5 B8
        Returns:
            str: 代表Modbus命令的十六进制字符串。
        """
        function_code = 0x10
        register_address = self.REG_ACTION_CONTROL
        register_count = 0x0001
        byte_count = 0x02
        register_data = 0x0000 # rACT = 0 (复位)

        payload = [
            (register_address >> 8) & 0xFF, register_address & 0xFF,
            (register_count >> 8) & 0xFF, register_count & 0xFF,
            byte_count,
            (register_data >> 8) & 0xFF, register_data & 0xFF
        ]
        command_frame_list = self._generate_command_frame(function_code, payload)
        return self._format_hex_string(command_frame_list)

    def enable_gripper(self) -> str:
        """
        生成夹爪使能/激活请求的命令帧 (rACT=1) (FC10)。
        目标寄存器 0x03E8，写入值 0x0001。
        示例帧: 09 10 03 E8 00 01 02 00 01 24 78
        Returns:
            str: 代表Modbus命令的十六进制字符串。
        """
        function_code = 0x10
        register_address = self.REG_ACTION_CONTROL
        register_count = 0x0001
        byte_count = 0x02
        register_data = 0x0001 # rACT = 1 (使能)

        payload = [
            (register_address >> 8) & 0xFF, register_address & 0xFF,
            (register_count >> 8) & 0xFF, register_count & 0xFF,
            byte_count,
            (register_data >> 8) & 0xFF, register_data & 0xFF
        ]
        command_frame_list = self._generate_command_frame(function_code, payload)
        return self._format_hex_string(command_frame_list)

    def run_gripper(self, target_position: int, force: int = 100, speed: int = 100) -> str:
        """
        生成参数化移动模式 (Mode 0) 的命令帧 (FC10)。
        一次写入3个寄存器: 0x03E8, 0x03E9, 0x03EA。
        Args:
            target_position (int): 目标位置 (0=完全张开, 255=完全闭合)。
            force (int): 目标力 (0-255)。
            speed (int): 目标速度 (0-255)。
        Returns:
            str: 代表Modbus命令的十六进制字符串。
        """
        pos = self._check_value(target_position, "目标位置")
        frc = self._check_value(force, "目标力")
        spd = self._check_value(speed, "目标速度")

        function_code = 0x10                # 功能码: 写多个寄存器
        start_register_address = self.REG_ACTION_CONTROL # 起始地址: 0x03E8
        register_count = 0x0003             # 写入寄存器数量: 3
        byte_count = 0x06                   # 数据字节数: 3 * 2 = 6

        # --- 准备三个寄存器的数据 ---
        # 寄存器 0x03E8 的值: 控制字 (rACT=1, rGTO=1, MODE=0) => 0x0009
        reg_data1 = 0x0009
        # 寄存器 0x03E9 的值: 位置在高字节
        reg_data2 = (pos << 8) | 0x00
        # 寄存器 0x03EA 的值: 力在高字节, 速度在低字节
        reg_data3 = (frc << 8) | spd

        # --- 构建数据负载 ---
        payload = [
            (start_register_address >> 8) & 0xFF, start_register_address & 0xFF, # 起始地址高, 低
            (register_count >> 8) & 0xFF, register_count & 0xFF,                 # 寄存器数量高, 低
            byte_count,                                                          # 数据字节数
            # 数据 (高字节在前)
            (reg_data1 >> 8) & 0xFF, reg_data1 & 0xFF,                           # 0x03E8 的数据
            (reg_data2 >> 8) & 0xFF, reg_data2 & 0xFF,                           # 0x03E9 的数据
            (reg_data3 >> 8) & 0xFF, reg_data3 & 0xFF                            # 0x03EA 的数据
        ]
        command_frame_list = self._generate_command_frame(function_code, payload)
        return self._format_hex_string(command_frame_list)

    def read_gripper_state(self, register_count: int = 3) -> str:
        """
        生成读取夹爪状态寄存器的命令帧 (FC04)。
        Args:
            register_count (int): 要读取的寄存器数量 (从0x07D0开始)，通常为1或3。
        Returns:
            str: 代表Modbus命令的十六进制字符串。
        """
        if register_count <= 0:
            print("警告: 读取的寄存器数量应大于0，已强制设为1。")
            register_count = 1
        if register_count > 10: # 限制一次读取过多寄存器
             print(f"警告: 请求读取 {register_count} 个寄存器过多，已限制为10个。")
             register_count = 10

        function_code = 0x04                  # 功能码: 读输入寄存器
        start_register_address = self.REG_GRIPPER_STATUS # 起始地址: 0x07D0

        payload = [
            (start_register_address >> 8) & 0xFF, start_register_address & 0xFF, # 起始地址高, 低
            (register_count >> 8) & 0xFF, register_count & 0xFF                # 寄存器数量高, 低
        ]
        command_frame_list = self._generate_command_frame(function_code, payload)
        return self._format_hex_string(command_frame_list)

    # --- 响应解码方法 ---
    # 注意：这些方法假设输入数据已经过处理（例如，从串口读取并可能已转换为特定格式）

    def decode_response(self, response_hex_string: str) -> tuple[int | None, list[int] | None]:
        """
        解码Modbus RTU响应帧 (以十六进制字符串形式提供)。
        执行基本长度和CRC校验。
        Args:
            response_hex_string (str): 收到的响应帧的十六进制字符串表示
                                       (例如 "091003E8000180F1")。
        Returns:
            tuple: (功能码, 数据负载) 或 (None, None) 如果校验失败或格式错误。
                   数据负载是字节列表(整数形式)。
                   对于写响应(FC06, FC10): 负载通常是地址和数量的回显。
                   对于读响应(FC03, FC04): 负载是[字节数, 数据1, 数据2...].
                   对于异常响应: 功能码最高位置1，负载是[异常码].
        """
        print(f"开始解码响应 (Hex): {response_hex_string}")
        if not response_hex_string or len(response_hex_string) < 8 or len(response_hex_string) % 2 != 0:
            print("解码错误：无效的十六进制字符串长度。")
            return None, None

        try:
            # 将十六进制字符串转换为字节值列表
            response_bytes_list = [int(response_hex_string[i:i+2], 16) for i in range(0, len(response_hex_string), 2)]
        except ValueError:
            print("解码错误：十六进制字符串中包含无效字符。")
            return None, None

        if len(response_bytes_list) < 4: # 最小长度: 从站地址, 功能码, CRC低, CRC高
             print("解码错误：响应帧太短。")
             return None, None

        # 提取帧的各个部分
        received_address = response_bytes_list[0]
        received_fc = response_bytes_list[1]
        payload_and_crc = response_bytes_list[2:]
        received_crc = payload_and_crc[-2:] # CRC [低字节, 高字节]
        payload = payload_and_crc[:-2]      # 数据负载部分

        # 校验从站地址 (可选，但推荐)
        if received_address != self.slave_id:
            print(f"解码警告：收到的从站地址 0x{received_address:02X} 与期望的 0x{self.slave_id:02X} 不匹配。")
            # 可以选择在这里返回错误，或者继续处理

        # 校验 CRC
        expected_crc = self._calculate_crc(response_bytes_list[:-2]) # 对地址、功能码、数据负载计算CRC
        if received_crc != expected_crc:
            # 注意: _calculate_crc 返回 [低, 高], received_crc 也是 [低, 高] (假设)
            print(f"解码错误：CRC校验失败! 收到=[0x{received_crc[0]:02X}, 0x{received_crc[1]:02X}], "
                  f"计算=[0x{expected_crc[0]:02X}, 0x{expected_crc[1]:02X}]")
            return None, None
        print("CRC校验通过。")

        # 检查是否为Modbus异常响应 (功能码最高位为1)
        if received_fc >= 0x80:
            exception_code = payload[0] if payload else -1 # 异常码在数据负载的第一个字节
            print(f"解码错误：收到Modbus异常响应! 功能码=0x{received_fc:02X}, 异常码=0x{exception_code:02X}")
            return received_fc, payload # 返回异常功能码和包含异常码的负载

        # 解码成功
        payload_hex_str = ' '.join([f'0x{b:02X}' for b in payload]) if payload else "无"
        print(f"解码成功: 功能码=0x{received_fc:02X}, 数据负载={payload_hex_str}")
        return received_fc, payload

    def decode_state_data(self, read_fc: int, data_payload: list[int]) -> tuple | None:
        """
        解码从 read_gripper_state (FC04) 收到的数据负载。
        Args:
            read_fc (int): 来自 decode_response 的功能码 (应为 0x04)。
            data_payload (list[int]): 来自 decode_response 的数据负载列表
                                      (格式: [字节数, 寄存器0高, 寄存器0低, ...])。
        Returns:
            tuple: (激活状态, 移动状态, 夹持状态, 当前位置, 错误代码, 当前速度, 当前力/电流)
                   或 None 如果解码失败。其中某些值可能为 None 如果未读取足够的寄存器。
        """
        print(f"开始解码状态数据: 功能码=0x{read_fc:02X}, 负载={data_payload}")
        if read_fc != 0x04 or not data_payload:
            print("状态解码错误：无效的功能码或空的数据负载。")
            return None

        byte_count = data_payload[0]           # 负载的第一个字节是数据字节数
        register_data_bytes = data_payload[1:] # 剩余的是寄存器数据

        # 校验字节数是否匹配
        if len(register_data_bytes) != byte_count:
            print(f"状态解码错误：字节数不匹配。期望 {byte_count} 字节数据, 实际收到 {len(register_data_bytes)} 字节。")
            return None

        num_registers_read = byte_count // 2
        if byte_count % 2 != 0:
             print("状态解码警告：收到奇数个数据字节。")
             # 也许仍然可以处理部分数据？

        print(f"读取了 {num_registers_read} 个寄存器 ({byte_count} 字节)。")

        # 初始化状态变量
        activate_state = None # 激活状态 (0x0=复位, 0x1=激活中, 0x3=完成) - 来自 gSTA
        move_state = None     # 移动状态 (False=停止, True=移动) - 来自 gGTO
        hand_state = None     # 夹持/物体检测状态 (0=移动中, 1=内撑接触, 2=外夹接触, 3=到达无物体) - 来自 gOBJ
        current_position = None # 当前位置回显 (0-255)
        error_code = None     # 故障代码 (0=无故障)
        current_speed = None  # 当前速度回显 (0-255)
        current_force = None  # 当前实际力/电流 (0-255) - 手册称 gCU (电流)

        # --- 根据读取的寄存器数量提取数据 ---
        # 假设寄存器顺序: 0x07D0, 0x07D1, 0x07D2 ...
        if num_registers_read >= 1:
            # 处理寄存器 0x07D0 (状态)
            # 数据字节索引: 0 (高字节), 1 (低字节)
            if len(register_data_bytes) >= 2:
                # reg0_hi = register_data_bytes[0] # 0x07D0 高字节未使用
                reg0_lo = register_data_bytes[1] # 0x07D0 低字节包含状态位 [Source 77]
                status_byte = reg0_lo
                print(f"  状态寄存器 (0x07D0) 低字节: 0x{status_byte:02X}")

                # 解析状态位
                # gSTA: 激活状态 (位 4, 5) -> 0, 1, 3
                activate_state = (status_byte & 0b00110000) >> 4
                # gGTO: 运动状态 (位 3) -> True/False
                move_state = bool((status_byte & 0b00001000) >> 3)
                # gOBJ: 物体检测状态 (位 6, 7) -> 0, 1, 2, 3
                hand_state = (status_byte & 0b11000000) >> 6

                print(f"    -> 激活状态(gSTA): {activate_state}")
                print(f"    -> 移动状态(gGTO): {'移动中' if move_state else '已停止'}")
                print(f"    -> 夹持状态(gOBJ): {hand_state}")
            else: print("状态解码警告: 数据不足，无法解析寄存器 0x07D0。")


        if num_registers_read >= 2:
            # 处理寄存器 0x07D1 (故障/位置)
            # 数据字节索引: 2 (高字节 - 位置), 3 (低字节 - 故障)
            if len(register_data_bytes) >= 4:
                reg1_hi = register_data_bytes[2] # 位置回显 [Source 83]
                reg1_lo = register_data_bytes[3] # 故障状态 [Source 80]
                current_position = reg1_hi
                error_code = reg1_lo
                print(f"  故障/位置寄存器 (0x07D1): 位置回显={current_position}, 故障码=0x{error_code:02X}")
                if error_code != 0: self._print_fault_description_cn(error_code) # 打印故障描述
            else: print("状态解码警告: 数据不足，无法解析寄存器 0x07D1。")


        if num_registers_read >= 3:
             # 处理寄存器 0x07D2 (速度/电流)
             # 数据字节索引: 4 (高字节 - 电流), 5 (低字节 - 速度)
             if len(register_data_bytes) >= 6:
                 reg2_hi = register_data_bytes[4] # 电流 [Source 87]
                 reg2_lo = register_data_bytes[5] # 速度回显 [Source 85]
                 current_speed = reg2_lo
                 current_force = reg2_hi # 使用手册中的 gCU (电流)
                 print(f"  速度/电流寄存器 (0x07D2): 速度回显={current_speed}, 电流={current_force}")
             else: print("状态解码警告: 数据不足，无法解析寄存器 0x07D2。")

        # 返回解析出的状态元组
        return (activate_state, move_state, hand_state, current_position,
                error_code, current_speed, current_force)

    def _print_fault_description_cn(self, fault_code):
        """打印中文的故障代码描述。"""
        if fault_code == 0x00: return # 无故障不打印
        print("    故障详情:")
        if fault_code & 0x01: print("      - 需要激活 (0x01)")
        if fault_code & 0x02: print("      - 控制指令错误 (0x02)")
        if fault_code & 0x04: print("      - 通讯丢失 (0x04)")
        if fault_code & 0x08: print("      - 过流 (0x08)")
        if fault_code & 0x10: print("      - 电压异常 (<20V 或 >30V) (0x10)")
        if fault_code & 0x20: print("      - 使能故障 (可能被阻挡) (0x20)")
        if fault_code & 0x40: print("      - 过温 (>85℃) (0x40)")
        if fault_code & 0x80: print("      - 产品自身故障 (0x80)")


# --- 示例用法 ---
def main():
    """主函数，演示如何生成命令。"""
    gripper = Gripper(slave_id=0x09) # 确保使用正确的从站ID

    print("\n--- 生成命令示例 ---")

    # 获取各种命令的十六进制字符串
    activate_hex = gripper.activate_request()      # 复位命令
    enable_hex = gripper.enable_gripper()          # 使能命令
    close_hex = gripper.run_gripper(target_position=255, force=150, speed=200) # 关闭
    open_hex = gripper.run_gripper(target_position=0, force=150, speed=200)   # 打开
    move_mid_hex = gripper.run_gripper(target_position=128, force=80, speed=50) # 移到中间
    read_status_hex = gripper.read_gripper_state(register_count=3) # 读取3个状态寄存器
    brake_open_hex = gripper.force_open_brake(True) # 强制打开抱闸

    print("\n--- 生成的命令汇总 (Hex) ---")
    print(f"夹爪复位命令:           {activate_hex}")
    print(f"夹爪使能命令:           {enable_hex}")
    print(f"关闭夹爪命令:           {close_hex}")
    print(f"打开夹爪命令:           {open_hex}")
    print(f"移动到位置128命令:      {move_mid_hex}")
    print(f"读取状态命令 (3寄存器): {read_status_hex}")
    print(f"强制打开抱闸命令:       {brake_open_hex}")

    print("\n--- 响应解码示例 ---")
    # 模拟一个收到的响应十六进制字符串 (例如，对使能命令的响应)
    # 实际夹爪响应: 09 10 03 E8 00 01 80 F1
    simulated_response_hex = "091003E8000180F1"
    fc, payload = gripper.decode_response(simulated_response_hex)
    if fc is not None:
        print(f"成功解码响应 -> 功能码: 0x{fc:02X}, 数据负载: {payload}")
        # 如果是写响应(FC10)，payload 通常是 [起始地址高, 低, 写入数量高, 低]

    print("-" * 20)

    # 模拟一个读取状态的响应 (读取3个寄存器)
    # 假设: 已激活(gSTA=3, gACT=1 -> 0x31), 已停止(gGTO=0), 到达闭合位置(Pos=FF), 无物体(gOBJ=3 -> 0xC0), 无故障(Fault=00), 速度=0, 电流=20(0x14)
    # 状态字节 (0x07D0 Low) = gOBJ | gSTA | gGTO | gMOD | gACT = 0xC0 | 0x30 | 0x00 | 0x00 | 0x01 = 0xF1 (假设gMOD=0)
    # 故障/位置 (0x07D1) = Pos=FF (Hi), Fault=00 (Lo) -> 0xFF00
    # 速度/电流 (0x07D2) = Current=14 (Hi), Speed=00 (Lo) -> 0x1400
    # 响应数据负载 = [ByteCount=6, Reg0Hi=0x00, Reg0Lo=0xF1, Reg1Hi=0xFF, Reg1Lo=0x00, Reg2Hi=0x14, Reg2Lo=0x00]
    simulated_payload = [0x06, 0x00, 0xF1, 0xFF, 0x00, 0x14, 0x00]
    # 完整帧 = [Addr=09, FC=04] + Payload + CRC
    frame_no_crc = [gripper.slave_id, 0x04] + simulated_payload
    crc = gripper._calculate_crc(frame_no_crc)
    simulated_status_frame_list = frame_no_crc + crc
    simulated_status_hex = "".join(f"{b:02X}" for b in simulated_status_frame_list)

    print(f"\n模拟读取状态响应 (Hex): {simulated_status_hex}")
    fc, payload = gripper.decode_response(simulated_status_hex)
    if fc == 0x04: # 确保是读响应
        state_tuple = gripper.decode_state_data(fc, payload)
        if state_tuple:
            (activate_state, move_state, hand_state, current_position,
             error_code, current_speed, current_force) = state_tuple
            print("\n成功解码状态数据:")
            print(f"  激活状态 (gSTA): {activate_state}")
            print(f"  移动状态 (gGTO): {'移动中' if move_state else '已停止'}")
            print(f"  夹持状态 (gOBJ): {hand_state}")
            print(f"  当前位置回显: {current_position}")
            print(f"  错误代码: 0x{error_code:02X}")
            print(f"  当前速度回显: {current_speed}")
            print(f"  当前力/电流: {current_force}")

if __name__ == "__main__":
    main()