# vision_interaction.py
# -*- coding: utf-8 -*-

import socket
import json
import time
import sounddevice as sd
import soundfile as sf
import numpy as np
import threading  # 用于非阻塞录音和播放
import queue  # 用于线程间通信
import os

# --- 配置 ---
SERVER_IP = "127.0.0.1"  # 修改为您的 Socket 服务器 IP
SERVER_PORT = 12345  # 修改为您的 Socket 服务器端口
RECORD_DEVICE = None  # 使用默认录音设备，可以指定设备 ID 或名称
PLAY_DEVICE = None  # 使用默认播放设备
SAMPLE_RATE = 44100  # 录音采样率
CHANNELS = 1  # 录音通道数
TEMP_AUDIO_FILENAME_SEND = "temp_recorded_audio.wav"
TEMP_AUDIO_FILENAME_RECV = "temp_received_audio.wav"

# --- 状态变量 ---
is_recording = False
audio_frames = []
recording_thread = None
audio_processing_queue = queue.Queue()  # 用于将录音数据传递给发送线程


# --- 占位符函数 ---
def yolo_recognize_and_grasp(object_id, target_info):
    """
    占位符：使用 YOLO 识别物体并执行抓取。
    object_id: 从服务器JSON中获取的要抓取的物体标识。
    target_info: 可能包含物体位置等信息。
    """
    print(
        f"[VISION_INTERACTION] Placeholder: Attempting to recognize and grasp object_id: {object_id} with info: {target_info}")
    # 在这里集成您的 YOLO 模型和机器人抓取逻辑
    # 例如:
    # detected_object = yolo_detect(object_id)
    # if detected_object:
    #     success = robot_controller.grasp(detected_object.position, detected_object.orientation)
    #     return success
    # return False
    time.sleep(2)  # 模拟操作耗时
    print(f"[VISION_INTERACTION] Placeholder: Grasp action for {object_id} completed.")
    return True


def play_audio_file(filename):
    """播放指定的音频文件"""
    try:
        if not os.path.exists(filename):
            print(f"[VISION_INTERACTION] Error: Audio file not found for playback: {filename}")
            return

        data, fs = sf.read(filename, dtype='float32')
        print(f"[VISION_INTERACTION] Playing received audio: {filename}")
        sd.play(data, fs, device=PLAY_DEVICE)
        sd.wait()  # 等待播放完成
        print(f"[VISION_INTERACTION] Playback finished.")
    except Exception as e:
        print(f"[VISION_INTERACTION] Error playing audio file {filename}: {e}")


# --- 核心功能 ---

def record_audio_callback(indata, frames, time, status):
    """sounddevice 录音回调函数"""
    global audio_frames
    if status:
        print(f"[VISION_INTERACTION] Recording status: {status}")
    audio_frames.append(indata.copy())


def start_recording_thread():
    """启动一个新线程来录音"""
    global is_recording, audio_frames, recording_thread

    if is_recording:
        print("[VISION_INTERACTION] Already recording.")
        return False

    print("[VISION_INTERACTION] Starting recording... Press 'Y' to stop, 'A' to cancel.")
    audio_frames = []
    is_recording = True

    def _record():
        global is_recording, audio_frames
        try:
            with sd.InputStream(samplerate=SAMPLE_RATE, device=RECORD_DEVICE,
                                channels=CHANNELS, callback=record_audio_callback):
                while is_recording:
                    sd.sleep(100)  # 保持流打开直到 is_recording 变为 False
        except Exception as e:
            print(f"[VISION_INTERACTION] Error during recording stream: {e}")
        finally:
            # 确保即使发生错误，is_recording 也会被重置（如果需要）
            # is_recording = False # 或者由 stop_recording 控制
            pass

    recording_thread = threading.Thread(target=_record)
    recording_thread.daemon = True  # 允许主程序退出时线程也退出
    recording_thread.start()
    return True


def stop_recording_and_save(save_path=TEMP_AUDIO_FILENAME_SEND):
    """停止录音并保存到文件"""
    global is_recording, audio_frames, recording_thread

    if not is_recording and not audio_frames:  # 如果没有在录音且没有数据
        print("[VISION_INTERACTION] Not recording or no audio data to save.")
        return None

    if is_recording:  # 如果仍在录音（例如因为超时或其他原因调用）
        print("[VISION_INTERACTION] Stopping recording...")
        is_recording = False  # 通知录音线程停止
        if recording_thread and recording_thread.is_alive():
            recording_thread.join(timeout=1.0)  # 等待录音线程结束
        if recording_thread and recording_thread.is_alive():
            print("[VISION_INTERACTION] Warning: Recording thread did not terminate gracefully.")

    if not audio_frames:
        print("[VISION_INTERACTION] No audio frames captured.")
        return None

    print(f"[VISION_INTERACTION] Saving recorded audio to {save_path}")
    try:
        recording_data = np.concatenate(audio_frames, axis=0)
        sf.write(save_path, recording_data, SAMPLE_RATE)
        print(f"[VISION_INTERACTION] Audio saved successfully: {save_path}")
        audio_frames = []  # 清空缓存
        return save_path
    except Exception as e:
        print(f"[VISION_INTERACTION] Error saving audio: {e}")
        audio_frames = []  # 清空缓存
        return None


def cancel_recording():
    """取消当前录音"""
    global is_recording, audio_frames, recording_thread
    if is_recording:
        print("[VISION_INTERACTION] Cancelling recording...")
        is_recording = False
        if recording_thread and recording_thread.is_alive():
            recording_thread.join(timeout=1.0)
        audio_frames = []  # 清空已录制的帧
        print("[VISION_INTERACTION] Recording cancelled.")
        return True
    print("[VISION_INTERACTION] No active recording to cancel.")
    return False


def send_audio_and_receive_response(audio_filepath):
    """通过 Socket 发送音频文件，接收音频和 JSON 响应"""
    if not os.path.exists(audio_filepath):
        print(f"[VISION_INTERACTION] Audio file not found for sending: {audio_filepath}")
        return None, None

    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            print(f"[VISION_INTERACTION] Connecting to server {SERVER_IP}:{SERVER_PORT}...")
            s.connect((SERVER_IP, SERVER_PORT))
            print("[VISION_INTERACTION] Connected to server.")

            # 1. 发送音频文件大小和文件名 (元数据)
            filesize = os.path.getsize(audio_filepath)
            filename = os.path.basename(audio_filepath)
            metadata = f"{filename}:{filesize}\n"  # 使用换行符分隔元数据和文件内容
            time.sleep(0.1)
            s.sendall(metadata.encode())
            print(f"[VISION_INTERACTION] Sent metadata: {metadata.strip()}")

            # 2. 发送音频文件内容
            with open(audio_filepath, 'rb') as f:
                while True:
                    bytes_read = f.read(4096)
                    if not bytes_read:
                        break
                    s.sendall(bytes_read)
            print(f"[VISION_INTERACTION] Audio file '{audio_filepath}' sent.")
            # 可选：发送一个结束标记，如果服务器需要
            # s.sendall(b"EOF_AUDIO")

            # 3. 接收服务器响应 (假设先收到 JSON，后收到音频)
            #    或者服务器可以先发送一个头部，指明接下来是什么类型的数据和长度

            # 简化处理：假设服务器先发送JSON长度，然后JSON，然后音频文件大小，然后音频

            # 接收JSON长度 (假设4字节)
            json_len_bytes = s.recv(4)
            if not json_len_bytes:
                print("[VISION_INTERACTION] Did not receive JSON length from server.")
                return None, None
            json_len = int.from_bytes(json_len_bytes, 'big')
            print(f"[VISION_INTERACTION] Expecting JSON of length: {json_len}")

            # 接收JSON数据
            json_data_bytes = b''
            while len(json_data_bytes) < json_len:
                packet = s.recv(json_len - len(json_data_bytes))
                if not packet: break
                json_data_bytes += packet

            if len(json_data_bytes) != json_len:
                print("[VISION_INTERACTION] Did not receive complete JSON data.")
                return None, None

            received_json = json.loads(json_data_bytes.decode())
            print(f"[VISION_INTERACTION] Received JSON: {received_json}")

            # 接收返回的音频文件大小 (假设4字节)
            audio_filesize_bytes = s.recv(4)
            if not audio_filesize_bytes:
                print("[VISION_INTERACTION] Server did not send audio filesize.")
                # 可能没有返回音频，只有JSON
                if os.path.exists(TEMP_AUDIO_FILENAME_RECV):  # 删除旧的临时文件
                    os.remove(TEMP_AUDIO_FILENAME_RECV)
                return None, received_json  # 返回 None 表示没有音频文件

            audio_filesize = int.from_bytes(audio_filesize_bytes, 'big')
            print(f"[VISION_INTERACTION] Expecting audio file of size: {audio_filesize}")

            if audio_filesize == 0:
                print("[VISION_INTERACTION] Server indicated no audio response (size 0).")
                if os.path.exists(TEMP_AUDIO_FILENAME_RECV):
                    os.remove(TEMP_AUDIO_FILENAME_RECV)
                return None, received_json

            # 接收音频文件
            with open(TEMP_AUDIO_FILENAME_RECV, 'wb') as f:
                bytes_received = 0
                while bytes_received < audio_filesize:
                    chunk = s.recv(min(4096, audio_filesize - bytes_received))
                    if not chunk:
                        break
                    f.write(chunk)
                    bytes_received += len(chunk)

            if bytes_received == audio_filesize:
                print(f"[VISION_INTERACTION] Received audio saved to {TEMP_AUDIO_FILENAME_RECV}")
                return TEMP_AUDIO_FILENAME_RECV, received_json
            else:
                print(
                    f"[VISION_INTERACTION] Incomplete audio received. Expected {audio_filesize}, got {bytes_received}")
                if os.path.exists(TEMP_AUDIO_FILENAME_RECV):
                    os.remove(TEMP_AUDIO_FILENAME_RECV)
                return None, received_json  # JSON 可能仍然有效

    except socket.error as e:
        print(f"[VISION_INTERACTION] Socket error: {e}")
    except json.JSONDecodeError as e:
        print(f"[VISION_INTERACTION] JSON decode error: {e}")
    except Exception as e:
        print(f"[VISION_INTERACTION] Error in send_audio_and_receive_response: {e}")
    return None, None


def process_vision_interaction():
    """
    主函数，用于在视觉模式下处理语音交互。
    这个函数会被 run_vision_mode 调用。
    按键检测需要由外部 Pygame 事件循环处理并调用这里的函数。
    """
    print("[VISION_INTERACTION] Vision interaction mode active.")
    print("[VISION_INTERACTION] Press 'X' (controller button) to start recording voice command.")

    # 这个函数本身不处理按键，而是等待外部调用其子函数
    # 例如: handle_vision_keypress('x'), handle_vision_keypress('y'), etc.
    #
    # 模拟一次交互流程，实际调用会由外部事件驱动
    # if simulate_keypress_x():
    #     start_recording_thread()
    #     time.sleep(3) # 模拟录音时长
    #     if simulate_keypress_y():
    #         saved_audio_path = stop_recording_and_save()
    #         if saved_audio_path:
    #             received_audio_path, command_json = send_audio_and_receive_response(saved_audio_path)
    #             if received_audio_path:
    #                 play_audio_file(received_audio_path)
    #                 os.remove(received_audio_path) # 清理临时文件
    #             if command_json:
    #                 handle_command_json(command_json)
    #             if os.path.exists(saved_audio_path):
    #                 os.remove(saved_audio_path) # 清理临时文件
    pass  # 实际逻辑由外部事件调用


def handle_command_json(command_json):
    """处理从服务器接收到的JSON指令"""
    if not isinstance(command_json, dict):
        print(f"[VISION_INTERACTION] Invalid command JSON format: {command_json}")
        return

    print(f"[VISION_INTERACTION] Handling command JSON: {command_json}")
    action = command_json.get("action")  # 假设JSON中有 "action" 字段
    target = command_json.get("target")  # 假设有 "target" 字段
    # 例如: {"action": "grasp", "target": 1} or {"action": "moveTo", "target": [x,y,z]}

    if action == "grasp" and target is not None:
        print(f"[VISION_INTERACTION] Received grasp command for target: {target}")
        # 假设 target 是物体 ID 或者更详细的信息
        # object_id = target if isinstance(target, (int, str)) else target.get("id")
        object_id = target  # 简化，假设 target 就是 ID
        yolo_recognize_and_grasp(object_id, command_json)  # 将整个 JSON 传递过去，可能包含额外信息
    elif action == "play_message":
        message = command_json.get("message", "No message content.")
        print(f"[VISION_INTERACTION] Server message: {message}")
        # 如果需要，这里也可以触发TTS播放 message
    else:
        print(f"[VISION_INTERACTION] Unknown or unhandled action: {action}")


# --- 这个模块如何被主程序使用 ---
# 在你的主程序 (例如 main_controller.py) 的 run_vision_mode 中:
#
# from vision_interaction import (
#     start_recording_thread,
#     stop_recording_and_save,
#     cancel_recording,
#     send_audio_and_receive_response,
#     play_audio_file,
#     handle_command_json,
#     TEMP_AUDIO_FILENAME_SEND,
#     TEMP_AUDIO_FILENAME_RECV
# )
# import os
#
# # 假设在 DualArmController 类中或其 UI 管理器中
# # 你需要有变量来追踪视觉模式的状态，例如 `self.vision_mode_recording = False`
#
# def run_vision_mode(self): # 这里的 self 是 DualArmController 实例
#     print("== 进入视觉模式 (实际逻辑) ==")
#     self.ui_manager.play_sound('vision_enter') # 假设有进入音效
#     # 提示用户按键
#     # self.ui_manager.update_display_message("视觉模式: 按 X 键开始录音")
#
#     # 在主事件循环 (DualArmController._handle_events) 中检测视觉模式下的特定按键
#     # 例如，如果当前是 VISION_MODE 并且按下了 'X' 按钮 (根据你的 controls_map 配置)
#     # if self.control_mode == MODE_VISION and event.type == pygame.JOYBUTTONDOWN:
#     #     if event.button == self.config.get_vision_record_button_idx(): # 假设配置了按键
#     #         if not vision_interaction.is_recording: # 使用 is_recording 状态
#     #             vision_interaction.start_recording_thread()
#     #             # self.ui_manager.update_display_message("录音中... 按 Y 结束, A 取消")
#     #         else:
#     #             print("已经在录音了")
#     #
#     #     elif event.button == self.config.get_vision_stop_button_idx():
#     #         if vision_interaction.is_recording:
#     #             saved_path = vision_interaction.stop_recording_and_save()
#     #             if saved_path:
#     #                 # self.ui_manager.update_display_message("发送语音...")
#     #                 # 为了不阻塞主循环，发送和接收也应该在线程中
#     #                 threading.Thread(target=self._process_recorded_audio, args=(saved_path,)).start()
#     #
#     #     elif event.button == self.config.get_vision_cancel_button_idx():
#     #         if vision_interaction.is_recording:
#     #             vision_interaction.cancel_recording()
#     #             # self.ui_manager.update_display_message("录音已取消. 按 X 重新开始.")
#     pass # run_vision_mode 本身可能不需要做太多，主要依赖事件循环调用此模块的函数

# def _process_recorded_audio(self, saved_audio_path): # 线程中运行的方法
#     # from vision_interaction import send_audio_and_receive_response, play_audio_file, handle_command_json, TEMP_AUDIO_FILENAME_RECV
#     # import os
#     received_audio, command = send_audio_and_receive_response(saved_audio_path)
#     if os.path.exists(saved_audio_path):
#         os.remove(saved_audio_path)
#
#     if received_audio:
#         # self.ui_manager.update_display_message("播放服务器回复...")
#         play_audio_file(received_audio)
#         if os.path.exists(received_audio): # TEMP_AUDIO_FILENAME_RECV
#             os.remove(received_audio)
#
#     if command:
#         # self.ui_manager.update_display_message(f"收到指令: {command.get('action','N/A')}")
#         handle_command_json(command)
#     # self.ui_manager.update_display_message("视觉模式: 按 X 键开始录音")


if __name__ == '__main__':
    # --- 本地测试 (不依赖主程序 pygame) ---
    # 需要安装 keyboard 库: pip install keyboard
    import keyboard

    print("本地测试 vision_interaction.py (需要管理员权限运行 keyboard 库，或在 Linux 上作为 root)")
    print("按 'x' 开始录音, 'y' 停止并处理, 'a' 取消录音, 'q' 退出测试.")


    # 模拟服务器端 (简单的 echo 和固定 JSON)
    def mock_server(server_ip, server_port):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s_serv:
            s_serv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s_serv.bind((server_ip, server_port))
            s_serv.listen()
            print(f"[MockServer] Listening on {server_ip}:{server_port}")
            conn, addr = s_serv.accept()
            with conn:
                print(f"[MockServer] Connected by {addr}")
                # 接收元数据
                metadata_bytes = conn.recv(1024)  # 假设元数据不会太长
                metadata_str = metadata_bytes.decode().strip()
                print(f"[MockServer] Received metadata: {metadata_str}")

                # 提取文件名和大小 (简单示例，实际应更健壮)
                try:
                    filename_cli, filesize_cli_str = metadata_str.split(':')
                    filesize_cli = int(filesize_cli_str)
                except ValueError:
                    print("[MockServer] Invalid metadata format.")
                    return

                print(f"[MockServer] Expecting file '{filename_cli}' of size {filesize_cli}")

                # 接收文件
                received_payload = b""
                while len(received_payload) < filesize_cli:
                    chunk = conn.recv(4096)
                    if not chunk: break
                    received_payload += chunk

                print(f"[MockServer] Received {len(received_payload)} bytes for '{filename_cli}'.")
                # 不保存，直接回传一个假的回应

                # 准备JSON回应
                response_json = {"action": "grasp", "target": 1, "detail": "Mock server response"}
                response_json_bytes = json.dumps(response_json).encode()

                # 准备一个假的音频文件回应 (可以是静音或测试音)
                mock_response_audio_path = "mock_server_response.wav"
                sr_mock = 22050
                duration_mock = 1
                frequency_mock = 440
                t_mock = np.linspace(0, duration_mock, int(sr_mock * duration_mock), False)
                audio_data_mock = 0.5 * np.sin(2 * np.pi * frequency_mock * t_mock)
                sf.write(mock_response_audio_path, audio_data_mock, sr_mock)

                response_audio_filesize = os.path.getsize(mock_response_audio_path)

                # 发送JSON长度
                conn.sendall(len(response_json_bytes).to_bytes(4, 'big'))
                # 发送JSON
                conn.sendall(response_json_bytes)
                print(f"[MockServer] Sent JSON: {response_json}")

                # 发送音频大小
                conn.sendall(response_audio_filesize.to_bytes(4, 'big'))
                # 发送音频内容
                with open(mock_response_audio_path, 'rb') as f_audio_resp:
                    while True:
                        bytes_to_send = f_audio_resp.read(4096)
                        if not bytes_to_send: break
                        conn.sendall(bytes_to_send)
                print(f"[MockServer] Sent mock audio response '{mock_response_audio_path}'.")
                os.remove(mock_response_audio_path)  # 清理


    server_thread = threading.Thread(target=mock_server, args=(SERVER_IP, SERVER_PORT))
    server_thread.daemon = True
    server_thread.start()
    time.sleep(1)  # 等待服务器启动

    while True:
        try:
            if keyboard.is_pressed('x'):
                if not is_recording:
                    start_recording_thread()
                while keyboard.is_pressed('x'): pass  # 等待按键释放
            elif keyboard.is_pressed('y'):
                if is_recording:
                    saved_file = stop_recording_and_save()
                    if saved_file:
                        print(f"Audio saved: {saved_file}, now sending...")


                        # 将网络操作放入线程以避免阻塞键盘监听
                        def process_in_thread(filepath):
                            recv_audio, recv_json = send_audio_and_receive_response(filepath)
                            if os.path.exists(filepath): os.remove(filepath)  # 清理发送的临时文件
                            if recv_audio:
                                play_audio_file(recv_audio)
                                if os.path.exists(recv_audio): os.remove(recv_audio)  # 清理接收的临时文件
                            if recv_json:
                                handle_command_json(recv_json)


                        threading.Thread(target=process_in_thread, args=(saved_file,)).start()
                while keyboard.is_pressed('y'): pass
            elif keyboard.is_pressed('a'):
                cancel_recording()
                while keyboard.is_pressed('a'): pass
            elif keyboard.is_pressed('q'):
                print("Exiting test.")
                break
            time.sleep(0.05)
        except Exception as e:
            print(f"Error in test loop: {e}")
            # keyboard 库在某些环境下可能需要特殊权限，或者在非主线程中使用会有问题
            break

    if is_recording:  # 确保退出时停止录音
        stop_recording_and_save()

    print("Test finished.")