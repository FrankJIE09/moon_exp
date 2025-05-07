# mock_audio_server.py
# -*- coding: utf-8 -*-

import socket
import json
import os
import time

SERVER_HOST = "127.0.0.1"  # 监听所有可用接口
SERVER_PORT = 12345  # 与 vision_interaction.py 中的端口匹配
RECEIVED_AUDIO_DIR = "received_from_client"  # 可选: 保存客户端音频的目录
DEFAULT_RESPONSE_AUDIO_PATH = "default_server_response.wav"  # 预设的响应音频
BUFFER_SIZE = 4096


def create_default_response_json():
    """创建一个默认的JSON响应"""
    return {
        "action": "grasp",
        "target": "default_object_id_123",
        "status": "success",
        "message": "This is a default response from the mock server.",
        "confidence": 0.95,
        "timestamp": time.time()
    }


def handle_client(conn, addr):
    print(f"[Server] Accepted connection from {addr}")
    try:
        # 1. 接收音频元数据 (文件名:大小\n)
        metadata_bytes = b''
        while not metadata_bytes.endswith(b'\n'):  # 读取直到换行符
            chunk = conn.recv(BUFFER_SIZE)
            if not chunk:
                print(f"[Server] Client {addr} disconnected before sending metadata.")
                return
            metadata_bytes += chunk

        metadata_str = metadata_bytes.decode().strip()
        print(f"[Server] Received metadata: '{metadata_str}'")

        try:
            client_audio_filename, client_audio_filesize_str = metadata_str.split(':')
            client_audio_filesize = int(client_audio_filesize_str)
        except ValueError:
            print(f"[Server] Invalid metadata format from {addr}. Closing connection.")
            conn.sendall(b"ERROR: Invalid metadata format.\n")  # 可选的错误回复
            return

        print(
            f"[Server] Expecting audio file '{client_audio_filename}' of size {client_audio_filesize} bytes from {addr}")

        # (可选) 创建目录保存接收到的音频
        if not os.path.exists(RECEIVED_AUDIO_DIR):
            os.makedirs(RECEIVED_AUDIO_DIR)

        # 为避免文件名冲突，可以加上时间戳或客户端地址
        # received_filepath = os.path.join(RECEIVED_AUDIO_DIR, f"{addr[0]}_{addr[1]}_{client_audio_filename}")
        received_filepath = os.path.join(RECEIVED_AUDIO_DIR,
                                         f"client_{time.strftime('%Y%m%d%H%M%S')}_{client_audio_filename}")

        # 2. 接收音频文件内容
        bytes_received = 0
        with open(received_filepath, 'wb') as f:  # 打开文件准备写入接收的音频
            while bytes_received < client_audio_filesize:
                remaining_bytes = client_audio_filesize - bytes_received
                chunk = conn.recv(min(BUFFER_SIZE, remaining_bytes))
                if not chunk:
                    print(f"[Server] Client {addr} disconnected during audio file transfer.")
                    break
                f.write(chunk)
                bytes_received += len(chunk)

        if bytes_received == client_audio_filesize:
            print(
                f"[Server] Successfully received '{client_audio_filename}' ({bytes_received} bytes) from {addr}. Saved to '{received_filepath}'.")
        else:
            print(
                f"[Server] Incomplete audio file from {addr}. Expected {client_audio_filesize}, received {bytes_received}.")
            # 可以选择不处理此不完整文件或发送错误

        # --- 准备并发送默认响应 ---

        # 3. 准备默认 JSON 响应
        response_json_obj = create_default_response_json()
        response_json_bytes = json.dumps(response_json_obj).encode()
        json_len = len(response_json_bytes)

        # 4. 准备默认音频文件响应
        if not os.path.exists(DEFAULT_RESPONSE_AUDIO_PATH):
            print(f"[Server] CRITICAL ERROR: Default response audio file '{DEFAULT_RESPONSE_AUDIO_PATH}' not found!")
            # 发送一个表示没有音频的响应 (size 0)
            response_audio_filesize = 0
            audio_data_to_send = b''
        else:
            response_audio_filesize = os.path.getsize(DEFAULT_RESPONSE_AUDIO_PATH)
            with open(DEFAULT_RESPONSE_AUDIO_PATH, 'rb') as f_audio:
                audio_data_to_send = f_audio.read()

        # 5. 发送响应给客户端 (与 vision_interaction.py 中客户端接收逻辑对应)
        #   a. JSON 长度 (4 bytes, big-endian)
        #   b. JSON 数据
        #   c. 音频文件大小 (4 bytes, big-endian)
        #   d. 音频文件数据

        # a. 发送 JSON 长度
        conn.sendall(json_len.to_bytes(4, 'big'))
        print(f"[Server] Sending JSON length: {json_len} bytes")

        # b. 发送 JSON 数据
        conn.sendall(response_json_bytes)
        print(f"[Server] Sent JSON response: {response_json_obj}")

        # c. 发送音频文件大小
        conn.sendall(response_audio_filesize.to_bytes(4, 'big'))
        print(f"[Server] Sending audio filesize: {response_audio_filesize} bytes")

        # d. 发送音频文件数据 (如果大小 > 0)
        if response_audio_filesize > 0:
            conn.sendall(audio_data_to_send)
            print(f"[Server] Sent default audio response: '{DEFAULT_RESPONSE_AUDIO_PATH}'")
        else:
            print(f"[Server] No audio response sent (due to missing file or size 0).")

        print(f"[Server] Response sent to {addr}. Closing connection.")

    except socket.error as e:
        print(f"[Server] Socket error with {addr}: {e}")
    except Exception as e:
        print(f"[Server] Unexpected error with {addr}: {e}")
    finally:
        conn.close()


def main():
    # 创建服务器套接字
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # 允许地址重用，避免 "Address already in use" 错误
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    try:
        server_socket.bind((SERVER_HOST, SERVER_PORT))
        server_socket.listen(5)  # 最多允许5个排队连接
        print(f"[Server] Mock Audio Server listening on {SERVER_HOST}:{SERVER_PORT}")
        print(f"[Server] Default audio response will be: '{DEFAULT_RESPONSE_AUDIO_PATH}'")
        if not os.path.exists(DEFAULT_RESPONSE_AUDIO_PATH):
            print(f"[Server] WARNING: Default response audio file '{DEFAULT_RESPONSE_AUDIO_PATH}' is missing!")
            print(f"[Server] Please create it or the server will send a 0-byte audio response.")

        while True:
            try:
                conn, addr = server_socket.accept()
                # 可以为每个客户端创建一个新线程来处理，但对于简单测试，串行处理也可以
                # import threading
                # client_thread = threading.Thread(target=handle_client, args=(conn, addr))
                # client_thread.daemon = True
                # client_thread.start()
                handle_client(conn, addr)  # 简单串行处理

            except socket.timeout:  # 如果设置了超时
                print("[Server] Socket timeout, continuing to listen...")
            except KeyboardInterrupt:
                print("[Server] Keyboard interrupt received. Shutting down...")
                break
            except Exception as e_accept:
                print(f"[Server] Error accepting connection: {e_accept}")
                time.sleep(1)  # 避免快速失败循环

    except Exception as e:
        print(f"[Server] Server critical error: {e}")
    finally:
        print("[Server] Closing server socket.")
        server_socket.close()


if __name__ == "__main__":
    # 确保默认响应音频存在 (或提示用户创建)
    if not os.path.exists(DEFAULT_RESPONSE_AUDIO_PATH):
        print(f"--- IMPORTANT ---")
        print(f"Default response audio file '{DEFAULT_RESPONSE_AUDIO_PATH}' not found.")
        print(f"Please create this WAV file, or run 'python create_default_wav.py' (if you have that script).")
        print(f"Without it, the server will indicate no audio response to the client.")
        print(f"-----------------")
        # choice = input("Continue without default audio? (y/N): ")
        # if choice.lower() != 'y':
        #     exit()
    main()