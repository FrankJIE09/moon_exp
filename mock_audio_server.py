# mock_audio_server.py (Rewritten with type hints and updated JSON)
# -*- coding: utf-8 -*-

"""
模拟音频服务器，用于测试 vision_interaction.py 客户端。
接收客户端发送的音频文件，并返回一个预设的默认音频文件和
一个预设的、符合抓取指令格式的 JSON 响应。
"""

import socket
import json
import os
import time
import traceback
from typing import Tuple, Optional, Dict, Any

# --- 配置常量 ---
SERVER_HOST: str = "0.0.0.0"  # 监听所有可用接口
SERVER_PORT: int = 12345  # 端口号
RECEIVED_AUDIO_DIR: str = "received_from_client"  # 保存接收音频的目录
DEFAULT_RESPONSE_AUDIO_PATH: str = "default_server_response.wav"  # 默认响应音频路径
BUFFER_SIZE: int = 4096  # Socket 缓冲区大小
MAX_METADATA_LENGTH: int = 1024  # 最大元数据长度 (防止攻击)
SOCKET_TIMEOUT: float = 60.0  # 客户端连接和数据传输的超时时间 (秒)


def create_default_response_json() -> Dict[str, Any]:
    """
    创建一个符合视觉抓取指令格式的默认JSON响应。
    """
    return {
        "action": "grasp",  # 指令类型为抓取
        "target": {
            "id": "default_mock_object",  # 目标物体ID (可修改)
            # !! 注意: 这里的坐标需要根据你的实际测试场景调整 !!
            # 例如，一个可能在机器人工作空间内的坐标 (单位: 毫米)
            "base_coordinates_mm": [300.0, 50.0, 100.0],
            # 可选: 物体姿态 (RPY, 度) - 如果不需要可以省略或设为 None
            "orientation_rpy_deg": [0.0, 0.0, 45.0],
            # 指定哪个手臂执行抓取
            "arm_choice": "left"  # 或 "right"
        },
        "status": "success",  # 指令状态
        "message": "Default grasp command from mock server.",  # 附加信息
        "confidence": 0.99,  # 模拟置信度
        "timestamp": time.time()  # 时间戳
    }


def handle_client(conn: socket.socket, addr: Tuple[str, int]):
    """处理单个客户端连接：接收音频，发送默认响应。"""
    print(f"\n[Server] Accepted connection from {addr}")
    conn.settimeout(SOCKET_TIMEOUT)  # 为此连接设置超时

    try:
        # 1. 接收音频元数据 (文件名:大小\n)
        metadata_buffer = b''
        metadata_str: Optional[str] = None
        print(f"[Server] Waiting for metadata from {addr} (Timeout: {SOCKET_TIMEOUT}s)...")

        while True:
            if b'\n' in metadata_buffer:
                metadata_part, rest_of_buffer = metadata_buffer.split(b'\n', 1)
                try:
                    metadata_str = metadata_part.decode()
                    metadata_buffer = rest_of_buffer  # Keep remainder for file content
                    break
                except UnicodeDecodeError:
                    print(f"[Server] Error: Could not decode metadata from {addr}. Assuming binary data.")
                    # Handle as if no valid metadata was received? Or close? Let's close.
                    metadata_str = None
                    break  # Exit loop, metadata_str will be None

            if len(metadata_buffer) >= MAX_METADATA_LENGTH:
                print(
                    f"[Server] Error: Metadata limit ({MAX_METADATA_LENGTH} bytes) exceeded or no newline from {addr}.")
                return  # Close connection

            chunk = conn.recv(BUFFER_SIZE)
            if not chunk:
                print(f"[Server] Client {addr} disconnected while sending metadata.")
                return
            metadata_buffer += chunk

        if metadata_str is None:
            print(f"[Server] Failed to receive valid metadata from {addr}. Closing connection.")
            return

        print(f"[Server] Received raw metadata string: '{metadata_str}'")

        # --- 解析元数据 ---
        try:
            client_audio_filename, client_audio_filesize_str = metadata_str.split(':')
            client_audio_filesize = int(client_audio_filesize_str)
            # Sanitize filename slightly
            client_audio_filename = os.path.basename(client_audio_filename.replace('../', ''))
            if not client_audio_filename: client_audio_filename = "unknown_client_audio.wav"
        except ValueError:
            print(f"[Server] Invalid metadata format '{metadata_str}' from {addr}. Closing connection.")
            # Optionally send error back: conn.sendall(b"ERROR: Invalid metadata.\n")
            return

        print(f"[Server] Expecting '{client_audio_filename}' ({client_audio_filesize} bytes) from {addr}")
        if client_audio_filesize < 0 or client_audio_filesize > 100 * 1024 * 1024:  # Sanity check size (e.g., < 100MB)
            print(f"[Server] Error: Unreasonable audio filesize ({client_audio_filesize}) from {addr}. Closing.")
            return

        # --- (可选) 保存接收到的音频 ---
        if not os.path.exists(RECEIVED_AUDIO_DIR):
            try:
                os.makedirs(RECEIVED_AUDIO_DIR)
            except OSError as e:
                print(f"[Server] Warning: Could not create directory '{RECEIVED_AUDIO_DIR}': {e}")

        # Create unique filename for saving
        received_filename = f"client_{addr[0]}_{addr[1]}_{time.strftime('%Y%m%d%H%M%S')}_{client_audio_filename}"
        received_filepath = os.path.join(RECEIVED_AUDIO_DIR, received_filename) if os.path.exists(
            RECEIVED_AUDIO_DIR) else None

        # --- 2. 接收音频文件内容 ---
        bytes_received_for_file = 0
        file_saved_successfully = False
        try:
            with open(received_filepath, 'wb') if received_filepath else open(os.devnull,
                                                                              'wb') as f:  # Write to devnull if dir creation failed
                # Write initial data from metadata buffer
                if metadata_buffer:
                    f.write(metadata_buffer)
                    bytes_received_for_file += len(metadata_buffer)

                # Receive remaining file content
                while bytes_received_for_file < client_audio_filesize:
                    remaining = client_audio_filesize - bytes_received_for_file
                    chunk = conn.recv(min(BUFFER_SIZE, remaining))
                    if not chunk:
                        print(f"[Server] Client {addr} disconnected during audio transfer.")
                        break
                    f.write(chunk)
                    bytes_received_for_file += len(chunk)

            if bytes_received_for_file == client_audio_filesize:
                if received_filepath:
                    print(f"[Server] Received {bytes_received_for_file} bytes. Saved to '{received_filepath}'.")
                else:
                    print(f"[Server] Received {bytes_received_for_file} bytes (not saved).")
                file_saved_successfully = True  # Mark as successful reception
            else:
                print(
                    f"[Server] Incomplete audio from {addr}. Expected {client_audio_filesize}, got {bytes_received_for_file}.")
                # Clean up incomplete file if it was created
                if received_filepath and os.path.exists(received_filepath):
                    try:
                        os.remove(received_filepath)
                    except OSError:
                        pass

        except socket.timeout:
            print(f"[Server] Timeout during audio transfer from {addr}.")
        except IOError as e:
            print(f"[Server] IOError saving received audio to '{received_filepath}': {e}")
            # Continue processing request, but log save error
        except Exception as e_recv:
            print(f"[Server] Error receiving audio data: {e_recv}")
            traceback.print_exc()
            # Decide whether to continue or close connection

        # --- 3. 准备并发送默认响应 (即使接收不完整也发送，用于测试客户端) ---
        print(f"[Server] Preparing default response for {addr}...")

        # a. Prepare JSON
        response_json_obj = create_default_response_json()
        # Modify JSON based on received filename if needed (example)
        # response_json_obj["received_file"] = client_audio_filename
        response_json_bytes = json.dumps(response_json_obj).encode()
        json_len = len(response_json_bytes)

        # b. Prepare Audio
        response_audio_filesize = 0
        audio_data_to_send = b''
        if not os.path.exists(DEFAULT_RESPONSE_AUDIO_PATH):
            print(f"[Server] Warning: Default audio response '{DEFAULT_RESPONSE_AUDIO_PATH}' not found!")
        else:
            try:
                response_audio_filesize = os.path.getsize(DEFAULT_RESPONSE_AUDIO_PATH)
                with open(DEFAULT_RESPONSE_AUDIO_PATH, 'rb') as f_audio:
                    audio_data_to_send = f_audio.read()
            except IOError as e:
                print(f"[Server] Error reading default response audio '{DEFAULT_RESPONSE_AUDIO_PATH}': {e}")
                response_audio_filesize = 0  # Send size 0 if file read fails

        # --- 4. 发送响应 ---
        try:
            # Send JSON length (4 bytes)
            conn.sendall(json_len.to_bytes(4, 'big'))
            # Send JSON data
            conn.sendall(response_json_bytes)
            print(f"[Server] Sent JSON (len {json_len}): {response_json_obj}")

            # Send Audio length (4 bytes)
            conn.sendall(response_audio_filesize.to_bytes(4, 'big'))
            # Send Audio data (if size > 0)
            if response_audio_filesize > 0:
                conn.sendall(audio_data_to_send)
                print(f"[Server] Sent Audio (len {response_audio_filesize}): '{DEFAULT_RESPONSE_AUDIO_PATH}'")
            else:
                print(f"[Server] Sent Audio length 0 (no audio data).")

            print(f"[Server] Full response sent to {addr}.")

        except socket.timeout:
            print(f"[Server] Timeout sending response to {addr}.")
        except socket.error as e_send:
            print(f"[Server] Socket error sending response to {addr}: {e_send}")

    except socket.timeout:
        print(f"[Server] Timeout during initial connection phase with {addr}.")
    except socket.error as e_sock:
        print(f"[Server] Socket error with {addr}: {e_sock}")
    except Exception as e_main:
        print(f"[Server] Unexpected error handling client {addr}: {e_main}")
        traceback.print_exc()
    finally:
        print(f"[Server] Closing connection with {addr}.")
        conn.close()


def main():
    """主服务器循环。"""
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    try:
        server_socket.bind((SERVER_HOST, SERVER_PORT))
        server_socket.listen(5)
        print(f"[Server] Mock Audio Server listening on {SERVER_HOST}:{SERVER_PORT}")
        if not os.path.exists(DEFAULT_RESPONSE_AUDIO_PATH):
            print(f"[Server] WARNING: Default audio '{DEFAULT_RESPONSE_AUDIO_PATH}' not found!")
        else:
            print(f"[Server] Default audio response: '{DEFAULT_RESPONSE_AUDIO_PATH}'")
        print(f"[Server] Default JSON response includes grasp action.")

        while True:
            try:
                print("\n[Server] Waiting for new connection...")
                conn, addr = server_socket.accept()
                # Handle client sequentially (for simplicity)
                # To handle multiple clients concurrently, use threading:
                # client_thread = threading.Thread(target=handle_client, args=(conn, addr))
                # client_thread.daemon = True
                # client_thread.start()
                handle_client(conn, addr)

            except KeyboardInterrupt:
                print("\n[Server] Keyboard interrupt received. Shutting down...")
                break
            except Exception as e_accept:
                print(f"[Server] Error accepting connection: {e_accept}")
                time.sleep(1)  # Prevent rapid spinning on accept errors

    except OSError as e_bind:
        print(f"[Server] CRITICAL ERROR binding to {SERVER_HOST}:{SERVER_PORT} - {e_bind}")
        print("  Check if the port is already in use or if you have permissions.")
    except Exception as e_main:
        print(f"[Server] Server critical error: {e_main}")
        traceback.print_exc()
    finally:
        print("[Server] Closing server socket.")
        server_socket.close()


if __name__ == "__main__":
    # Check for default audio file before starting
    if not os.path.exists(DEFAULT_RESPONSE_AUDIO_PATH):
        print("-" * 20)
        print(f"!! WARNING: Default response audio file '{DEFAULT_RESPONSE_AUDIO_PATH}' is missing.")
        print("   The server will run but will send 0-length audio responses.")
        print("   Consider creating a dummy WAV file or running 'create_default_wav.py'.")
        print("-" * 20)
        time.sleep(2)  # Pause to let user see the warning
    main()