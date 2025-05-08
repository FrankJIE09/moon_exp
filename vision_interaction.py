# vision_interaction.py
# -*- coding: utf-8 -*-

"""
Handles vision mode interactions: voice command recording, communication with a server,
and execution of commands, including 3D vision-based grasping using Orbbec camera and YOLO.

This module assumes it is called by a main controller (like DualArmController)
which manages the overall state, event loop (Pygame), and provides necessary
instances for robot control, cameras, YOLO models, and calibration data.
"""

import socket
import json
import time
import os
import numpy as np
import traceback
import threading
import queue

# Audio Handling Dependencies (Choose one set or adapt)
# Option 1: sounddevice + soundfile (Recommended for cross-platform)
try:
    import sounddevice as sd
    import soundfile as sf

    AUDIO_LIB = "sounddevice"
except ImportError:
    print("Warning: sounddevice or soundfile not found. Audio recording/playback will fail.")
    AUDIO_LIB = None

# Option 2: PyAudio + wave (Alternative)
# try:
#     import pyaudio
#     import wave
#     AUDIO_LIB = "pyaudio"
# except ImportError:
#     # Fallback check if sounddevice also failed
#     if not AUDIO_LIB:
#         print("Warning: Neither sounddevice/soundfile nor PyAudio/wave found. Audio functions disabled.")
#         AUDIO_LIB = None

# Vision/Math Dependencies
try:
    from scipy.spatial.transform import Rotation as R
except ImportError:
    print("Warning: scipy not found. Coordinate transformations will fail.")
    R = None  # Define R as None to allow conditional checks
try:
    from ultralytics import YOLO
except ImportError:
    print("Warning: ultralytics (YOLO) not found. Object detection will fail.")
    YOLO = None  # Define YOLO as None

# Assume orbbec_camera module provides necessary camera client functionality
# Example: from software.vision.orbbec_camera import OrbbecCameraClient


# --- Configuration Constants ---
SERVER_IP = "127.0.0.1"  # IP of the backend processing server
SERVER_PORT = 12345  # Port of the backend processing server
CONNECTION_TIMEOUT = 10  # Socket connection timeout (seconds)
RECV_TIMEOUT = 15  # Socket receive timeout (seconds)
BUFFER_SIZE = 4096  # Socket buffer size

# Audio Recording Settings
RECORD_DEVICE = None  # Use system default device, can be specific index/name
PLAY_DEVICE = None  # Use system default device
SAMPLE_RATE = 44100  # Standard audio sample rate
CHANNELS = 1  # Mono audio recording
TEMP_AUDIO_FILENAME_SEND = "temp_recorded_audio.wav"  # Temp file for sending
TEMP_AUDIO_FILENAME_RECV = "temp_received_audio.wav"  # Temp file for received audio

# Vision/Grasp Settings
CONF_THRESHOLD = 0.5  # YOLO detection confidence threshold
OBSERVATION_HEIGHT_OFFSET_MM = 250.0  # Z offset for observation pose (mm)
MAX_OBSERVATION_RETRIES = 2  # Max attempts for vision sequence
DEFAULT_GRASP_SPEED = 30  # Speed for final grasp approach (mm/s or deg/s)
DEFAULT_MOVE_SPEED = 50  # Speed for general moves
DEFAULT_GRIPPER_SPEED = 150  # Gripper speed
DEFAULT_GRIPPER_FORCE = 100  # Gripper force
PRE_GRASP_OFFSET_Z_MM = 100.0  # Offset above grasp point (mm) - Assuming Base Z UP
POST_GRASP_LIFT_Z_MM = 50.0  # Lift distance after grasp (mm) - Assuming Base Z UP
DEFAULT_LEFT_GRASP_RPY = [180.0, 0.0, 180.0]  # Default grasp tool RPY for Left Arm (Base Frame)
DEFAULT_RIGHT_GRASP_RPY = [180.0, 0.0, 0.0]  # Default grasp tool RPY for Right Arm (Base Frame)

# --- Module State ---
is_recording = False
audio_frames = []
recording_thread = None


# --- Audio Handling Functions ---

def record_audio_callback(indata, frames, time, status):
    """Callback for sounddevice InputStream."""
    global audio_frames
    if status:
        # E.g., Input overflow, Underflow
        print(f"[Audio Callback] Status: {status}", flush=True)
    if is_recording:  # Only append if still in recording state
        audio_frames.append(indata.copy())


def play_audio_file(filename):
    """Plays an audio file using sounddevice."""
    if AUDIO_LIB != "sounddevice":
        print("[Play Audio] Error: sounddevice library not available.")
        return
    try:
        if not os.path.exists(filename):
            print(f"[Play Audio] Error: File not found: {filename}")
            return
        data, fs = sf.read(filename, dtype='float32')
        print(f"[Play Audio] Playing {filename} (Sample rate: {fs})...")
        sd.play(data, fs, device=PLAY_DEVICE, blocking=True)  # Use blocking=True
        # sd.wait() # No longer needed if blocking=True
        print(f"[Play Audio] Playback finished.")
    except Exception as e:
        print(f"[Play Audio] Error playing {filename}: {e}")
        traceback.print_exc()


def start_recording_thread():
    """Starts a background thread for audio recording using sounddevice."""
    global is_recording, audio_frames, recording_thread

    if AUDIO_LIB != "sounddevice":
        print("[Start Recording] Error: sounddevice library not available.")
        return False
    if is_recording:
        print("[Start Recording] Already recording.")
        return False

    print("[Start Recording] Starting recording... Trigger Stop/Cancel via main controller.")
    audio_frames = []  # Clear previous frames
    is_recording = True  # Set state flag

    def _record_task():
        """The actual recording task run in the thread."""
        global is_recording, audio_frames
        try:
            # Use sounddevice stream within the thread context
            with sd.InputStream(samplerate=SAMPLE_RATE, device=RECORD_DEVICE,
                                channels=CHANNELS, callback=record_audio_callback,
                                blocksize=1024):  # Adjust blocksize if needed
                while is_recording:
                    sd.sleep(50)  # Sleep briefly to allow state changes
            print("[Record Thread] Recording stream stopped.")
        except sd.PortAudioError as pae:
            print(f"[Record Thread] PortAudio Error during recording stream: {pae}")
            # Potentially indicate failure to the main thread
            is_recording = False  # Ensure state reflects stop
        except Exception as e:
            print(f"[Record Thread] Error during recording stream: {e}")
            traceback.print_exc()
            is_recording = False  # Ensure state reflects stop

    recording_thread = threading.Thread(target=_record_task, name="AudioRecordThread")
    recording_thread.daemon = True
    recording_thread.start()
    return True


def stop_recording_and_save(save_path=TEMP_AUDIO_FILENAME_SEND):
    """Stops the recording thread and saves the captured audio."""
    global is_recording, audio_frames, recording_thread

    if not is_recording and not audio_frames:
        print("[Stop Recording] Not recording or no audio data captured.")
        return None

    if is_recording:
        print("[Stop Recording] Signaling recording thread to stop...")
        is_recording = False  # Signal thread to exit loop

    if recording_thread and recording_thread.is_alive():
        print("[Stop Recording] Waiting for recording thread to finish...")
        recording_thread.join(timeout=2.0)  # Wait for thread (max 2 sec)
        if recording_thread.is_alive():
            print("[Stop Recording] Warning: Recording thread did not terminate.")
            # Forcefully losing audio frames might happen here if thread is stuck

    recording_thread = None  # Clear thread reference

    if not audio_frames:
        print("[Stop Recording] No audio frames were captured.")
        return None

    print(f"[Stop Recording] Saving {len(audio_frames)} frames to {save_path}")
    try:
        # Concatenate frames only if library is sounddevice
        if AUDIO_LIB == "sounddevice":
            if not audio_frames:  # Double check after join
                print("[Stop Recording] No audio frames after thread join.")
                return None
            recording_data = np.concatenate(audio_frames, axis=0)
            sf.write(save_path, recording_data, SAMPLE_RATE)
            print(f"[Stop Recording] Audio saved successfully: {save_path}")
            audio_frames = []  # Clear buffer
            return save_path
        else:
            print("[Stop Recording] Error: Cannot save, unsupported audio library.")
            audio_frames = []
            return None
    except Exception as e:
        print(f"[Stop Recording] Error saving audio: {e}")
        traceback.print_exc()
        audio_frames = []
        return None


def cancel_recording():
    """Cancels the current recording without saving."""
    global is_recording, audio_frames, recording_thread
    if not is_recording:
        print("[Cancel Recording] No active recording to cancel.")
        return False

    print("[Cancel Recording] Canceling recording...")
    is_recording = False  # Signal thread to stop

    if recording_thread and recording_thread.is_alive():
        recording_thread.join(timeout=1.0)  # Wait briefly
        if recording_thread.is_alive():
            print("[Cancel Recording] Warning: Recording thread did not terminate quickly.")

    recording_thread = None
    audio_frames = []  # Discard frames
    print("[Cancel Recording] Recording cancelled and data discarded.")
    return True


# --- Network Communication ---

def send_audio_and_receive_response(audio_filepath):
    """
    Sends audio file via Socket, receives JSON and audio response.
    Includes timeouts and robust receiving logic.
    """
    # ...(Implementation from the previous response, including timeouts)...
    if not os.path.exists(audio_filepath):
        print(f"[VISION_INTERACTION] Error: Audio file not found for sending: {audio_filepath}")
        return None, None

    received_json = None
    received_audio_path = None
    sock = None

    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(CONNECTION_TIMEOUT)

        print(
            f"[VISION_INTERACTION] Connecting to server {SERVER_IP}:{SERVER_PORT} (Timeout: {CONNECTION_TIMEOUT}s)...")
        sock.connect((SERVER_IP, SERVER_PORT))
        print("[VISION_INTERACTION] Connected to server.")
        sock.settimeout(RECV_TIMEOUT)

        # 1. Send metadata
        filesize = os.path.getsize(audio_filepath)
        filename = os.path.basename(audio_filepath)
        filename_safe = filename.replace("\n", "_").replace(":", "_")
        metadata = f"{filename_safe}:{filesize}\n"
        print(f"[VISION_INTERACTION] Sending metadata: {metadata.strip()} ({len(metadata.encode())} bytes)")
        sock.sendall(metadata.encode())
        print(f"[VISION_INTERACTION] Metadata sent.")

        # 2. Send audio file
        print(f"[VISION_INTERACTION] Sending audio file '{audio_filepath}' ({filesize} bytes)...")
        with open(audio_filepath, 'rb') as f:
            bytes_sent = 0
            while True:
                bytes_read = f.read(BUFFER_SIZE)
                if not bytes_read: break
                sock.sendall(bytes_read)
                bytes_sent += len(bytes_read)
        print(f"[VISION_INTERACTION] Audio file sent ({bytes_sent} bytes).")

        # 3. Receive Response
        # --- Receive JSON ---
        print(f"[VISION_INTERACTION] Waiting for JSON length (Timeout: {RECV_TIMEOUT}s)...")
        json_len_bytes = sock.recv(4)
        if not json_len_bytes or len(json_len_bytes) < 4:
            print("[VISION_INTERACTION] Error: Did not receive complete JSON length.")
            return None, None
        json_len = int.from_bytes(json_len_bytes, 'big')
        print(f"[VISION_INTERACTION] Expecting JSON: {json_len} bytes.")

        if 0 < json_len < 10 * 1024 * 1024:  # Check reasonable size
            json_data_bytes = b''
            while len(json_data_bytes) < json_len:
                chunk = sock.recv(min(json_len - len(json_data_bytes), BUFFER_SIZE))
                if not chunk:
                    print("[VISION_INTERACTION] Error: Connection closed while receiving JSON.")
                    break
                json_data_bytes += chunk

            if len(json_data_bytes) == json_len:
                try:
                    received_json = json.loads(json_data_bytes.decode())
                    print(f"[VISION_INTERACTION] Received JSON: {received_json}")
                except json.JSONDecodeError as e_json:
                    print(
                        f"[VISION_INTERACTION] JSON decode error: {e_json}. Data: '{json_data_bytes.decode(errors='ignore')}'")
                    received_json = {"error": "JSONDecodeError"}
            else:
                print(
                    f"[VISION_INTERACTION] Error: Incomplete JSON data. Expected {json_len}, got {len(json_data_bytes)}.")
                received_json = {"error": "Incomplete JSON"}
        elif json_len == 0:
            print("[VISION_INTERACTION] Server indicated no JSON response (length 0).")
            received_json = {}
        else:
            print(f"[VISION_INTERACTION] Error: Invalid JSON length received ({json_len}).")
            received_json = {"error": "Invalid JSON Length"}

        # --- Receive Audio ---
        print(f"[VISION_INTERACTION] Waiting for audio filesize...")
        audio_filesize_bytes = sock.recv(4)
        if not audio_filesize_bytes or len(audio_filesize_bytes) < 4:
            print("[VISION_INTERACTION] Error: Did not receive complete audio filesize.")
            return None, received_json  # JSON might be valid
        audio_filesize = int.from_bytes(audio_filesize_bytes, 'big')
        print(f"[VISION_INTERACTION] Expecting audio file: {audio_filesize} bytes.")

        # Clean old temp file if exists
        if os.path.exists(TEMP_AUDIO_FILENAME_RECV):
            try:
                os.remove(TEMP_AUDIO_FILENAME_RECV)
            except OSError as e:
                print(f"Warning: Could not remove old temp audio: {e}")

        if 0 < audio_filesize < 50 * 1024 * 1024:  # Reasonable audio size limit
            print(f"[VISION_INTERACTION] Receiving audio file '{TEMP_AUDIO_FILENAME_RECV}'...")
            with open(TEMP_AUDIO_FILENAME_RECV, 'wb') as f_recv:
                bytes_received = 0
                while bytes_received < audio_filesize:
                    chunk = sock.recv(min(BUFFER_SIZE, audio_filesize - bytes_received))
                    if not chunk:
                        print("[VISION_INTERACTION] Error: Connection closed while receiving audio.")
                        break
                    f_recv.write(chunk)
                    bytes_received += len(chunk)

            if bytes_received == audio_filesize:
                print(f"[VISION_INTERACTION] Received audio saved: '{TEMP_AUDIO_FILENAME_RECV}'")
                received_audio_path = TEMP_AUDIO_FILENAME_RECV
            else:
                print(f"[VISION_INTERACTION] Error: Incomplete audio. Expected {audio_filesize}, got {bytes_received}.")
                if os.path.exists(TEMP_AUDIO_FILENAME_RECV):  # Clean incomplete file
                    try:
                        os.remove(TEMP_AUDIO_FILENAME_RECV)
                    except OSError as e:
                        print(f"Warning: Could not remove incomplete temp audio: {e}")
                received_audio_path = None
        elif audio_filesize == 0:
            print("[VISION_INTERACTION] Server indicated no audio response (size 0).")
            received_audio_path = None
        else:
            print(f"[VISION_INTERACTION] Error: Invalid audio filesize ({audio_filesize}).")
            received_audio_path = None

        return received_audio_path, received_json

    except socket.timeout as e_timeout:
        print(f"[VISION_INTERACTION] Socket timeout: {e_timeout}")
        return None, received_json  # Return potentially valid JSON
    except socket.error as e_sock:
        print(f"[VISION_INTERACTION] Socket error: {e_sock}")
        return None, received_json
    except Exception as e_general:
        print(f"[VISION_INTERACTION] Unexpected error in send/receive: {e_general}")
        traceback.print_exc()
        return None, received_json
    finally:
        if sock:
            print("[VISION_INTERACTION] Closing socket.")
            sock.close()


# --- Helper: Transformation ---
def create_transformation_matrix(translation_mm, rpy_degrees):
    """Creates a 4x4 transformation matrix from translation (mm) and RPY (deg)."""
    if R is None: raise ImportError("scipy is required for transformations.")
    T = np.eye(4)
    T[:3, 3] = np.array(translation_mm) / 1000.0  # Convert mm to meters
    try:
        r = R.from_euler('xyz', rpy_degrees, degrees=True)
        T[:3, :3] = r.as_matrix()
    except ValueError as e:
        print(f"[Transform Helper] Error creating rotation from RPY {rpy_degrees}: {e}. Using identity.")
        # Keep T[:3,:3] as identity
    return T


def transformation_matrix_to_xyzrpy(matrix):
    """Converts a 4x4 matrix back to [x,y,z (mm), r,p,y (deg)]."""
    if R is None: raise ImportError("scipy is required for transformations.")
    translation_meters = matrix[:3, 3]
    rotation_matrix = matrix[:3, :3]
    try:
        r = R.from_matrix(rotation_matrix)
        # Check for gimbal lock or invalid rotation matrix if needed
        euler_angles_deg = r.as_euler('xyz', degrees=True)
    except ValueError as e:
        print(f"[Transform Helper] Error converting matrix to RPY: {e}. Using [0,0,0].")
        euler_angles_deg = [0.0, 0.0, 0.0]
    translation_mm = translation_meters * 1000.0
    return list(translation_mm) + list(euler_angles_deg)


def transform_point(point_xyz_m, matrix):
    """Applies a 4x4 transformation matrix to a 3D point (in meters)."""
    point_4d = np.append(np.array(point_xyz_m), 1)
    transformed_point_4d = matrix @ point_4d
    return list(transformed_point_4d[:3])  # Return meters


# --- Core Vision & Grasp Logic ---

def move_arm_to_observe(arm_choice, target_coords_mm, robot_controllers):
    """Moves the chosen arm's camera to observe the target area."""
    # ...(Implementation from previous response)...
    print(f"[Observe] Moving {arm_choice} arm to observe near {target_coords_mm}...")
    controller = robot_controllers.get(arm_choice)
    if not controller:
        print(f"[Observe] Error: Controller for {arm_choice} not found.")
        return False

    obs_pose_xyz = list(target_coords_mm[:3])
    obs_pose_xyz[2] += OBSERVATION_HEIGHT_OFFSET_MM

    if arm_choice == "left":
        obs_rpy = list(DEFAULT_LEFT_GRASP_RPY)  # Assume camera points same as tool down
    else:
        obs_rpy = list(DEFAULT_RIGHT_GRASP_RPY)

    final_obs_pose = obs_pose_xyz + obs_rpy
    print(f"[Observe] Calculated Observation Pose (Base): {final_obs_pose}")

    move_func = getattr(controller, 'move_robot' if arm_choice == 'left' else 'move_right_robot', None)
    if not move_func:
        print(f"[Observe] Error: Move function not found for {arm_choice} controller.")
        return False

    try:
        print(f"  [{arm_choice.upper()}] Executing move to observe...")
        if not move_func(list(final_obs_pose), speed=DEFAULT_MOVE_SPEED, block=True):
            print(f"  [{arm_choice.upper()}] Error: Failed to move to observation pose.")
            return False
        print(f"  [{arm_choice.upper()}] Reached observation pose.")
        time.sleep(0.5)
        return True
    except Exception as e:
        print(f"[Observe] Error during move: {e}")
        traceback.print_exc()
        return False


def capture_and_detect(object_id, camera_client, yolo_model):
    """Captures frames and runs YOLO detection."""
    # ...(Implementation from previous response)...
    if not camera_client or not yolo_model:
        print("[Detect] Error: Camera client or YOLO model not provided.")
        return None, None, None

    print(f"[Detect] Capturing frames and detecting '{object_id}'...")
    try:
        color_image, depth_image, depth_frame = camera_client.get_frames()
        if color_image is None or depth_frame is None:
            print("[Detect] Error: Failed to capture valid frames.")
            return None, None, None

        # Optional: Adjust exposure...

        # --- YOLO Detection ---
        print("[Detect] Running YOLO prediction...")
        results = yolo_model.predict(color_image, conf=CONF_THRESHOLD, verbose=False)  # verbose=False less output

        best_box = None
        min_depth = float('inf')
        detection_found = False

        for result in results:
            if result.boxes is None: continue  # Handle cases where no boxes are found
            boxes = result.boxes.xyxy.cpu().numpy()
            confidences = result.boxes.conf.cpu().numpy()
            labels = result.names
            class_ids = result.boxes.cls.cpu().numpy()

            print(f"[Detect] Found {len(boxes)} boxes in image.")
            for i, box in enumerate(boxes):
                conf = confidences[i]
                cls_id = int(class_ids[i])
                label_name = labels.get(cls_id, "unknown")

                if label_name == object_id:
                    detection_found = True
                    x1, y1, x2, y2 = box
                    center_x = int((x1 + x2) / 2)
                    center_y = int((y1 + y2) / 2)

                    # Check bounds before getting depth
                    h, w = depth_frame.shape[:2]
                    if not (0 <= center_x < w and 0 <= center_y < h):
                        print(
                            f"[Detect] Center pixel ({center_x},{center_y}) out of bounds for depth frame ({w}x{h}). Skipping box.")
                        continue

                    depth_mm = camera_client.get_depth_for_color_pixel(depth_frame=depth_frame,
                                                                       color_point=[center_x, center_y])

                    if depth_mm is not None and 0 < depth_mm < 10000:  # Check valid range (e.g., < 10m)
                        print(f"[Detect]   Found '{object_id}' (Conf: {conf:.2f}, Depth: {depth_mm}mm)")
                        if depth_mm < min_depth:
                            min_depth = depth_mm
                            best_box = box
                    # else: print(f"[Detect]   Found '{object_id}' but invalid depth ({depth_mm})")

        if best_box is not None:
            print(f"[Detect] Best box selected: {best_box} at depth {min_depth}mm")
            return best_box, color_image, depth_frame
        else:
            msg = f"Object '{object_id}' not found with valid depth." if detection_found else f"Object '{object_id}' not found."
            print(f"[Detect] {msg}")
            return None, color_image, depth_frame

    except Exception as e:
        print(f"[Detect] Error during capture/detection: {e}")
        traceback.print_exc()
        return None, None, None


def calculate_3d_position_camera_frame(detection_box, depth_frame, camera_client):
    """Calculates object center 3D position in CAMERA frame (meters)."""
    # ...(Implementation from previous response)...
    if detection_box is None or depth_frame is None or camera_client is None: return None
    print(f"[Calc Pos Cam] Calculating 3D position for box: {detection_box}")
    try:
        # Check for intrinsics
        if not all(hasattr(camera_client, attr) for attr in ['rgb_fx', 'rgb_fy', 'ppx', 'ppy']):
            print("[Calc Pos Cam] Error: Camera client missing intrinsic attributes.")
            return None
        fx, fy = camera_client.rgb_fx, camera_client.rgb_fy
        ppx, ppy = camera_client.ppx, camera_client.ppy
        if fx == 0 or fy == 0:
            print(f"[Calc Pos Cam] Error: Invalid camera intrinsics fx={fx}, fy={fy}")
            return None

        x1, y1, x2, y2 = detection_box
        center_x = int((x1 + x2) / 2)
        center_y = int((y1 + y2) / 2)

        # Check bounds
        h, w = depth_frame.shape[:2]
        if not (0 <= center_x < w and 0 <= center_y < h):
            print(f"[Calc Pos Cam] Center pixel ({center_x},{center_y}) out of bounds ({w}x{h}).")
            return None

        depth_mm = camera_client.get_depth_for_color_pixel(depth_frame=depth_frame, color_point=[center_x, center_y])

        if depth_mm is None or depth_mm <= 10:  # Min depth check
            print(f"[Calc Pos Cam] Error: Invalid depth ({depth_mm}mm) at pixel ({center_x}, {center_y}).")
            return None

        real_z = depth_mm / 1000.0  # meters
        real_x = (center_x - ppx) * real_z / fx
        real_y = (center_y - ppy) * real_z / fy

        pos_cam_m = [real_x, real_y, real_z]
        print(f"[Calc Pos Cam] Position in Camera Frame (meters): {pos_cam_m}")
        return pos_cam_m
    except Exception as e:
        print(f"[Calc Pos Cam] Error calculating 3D position: {e}")
        traceback.print_exc()
        return None


def transform_camera_to_base(pos_cam_m, arm_choice, robot_controllers, calibration_matrices):
    """Transforms a point from camera frame to the arm's base frame (returns mm)."""
    # ...(Implementation from previous response)...
    print(f"[Transform] Transforming camera point {pos_cam_m} (m) to base frame for {arm_choice} arm.")
    controller = robot_controllers.get(arm_choice)
    T_end_to_camera_list = calibration_matrices.get(arm_choice)

    if not controller: print(f"[Transform] Error: Controller for '{arm_choice}' not found."); return None
    if not T_end_to_camera_list: print(
        f"[Transform] Error: Calibration matrix for '{arm_choice}' not found."); return None

    try:
        T_end_to_camera = np.array(T_end_to_camera_list)
        if T_end_to_camera.shape != (4, 4):
            print(f"[Transform] Error: Invalid shape for calibration matrix {arm_choice}: {T_end_to_camera.shape}");
            return None

        current_tcp_pose_base_list = controller.getTCPPose()
        if not current_tcp_pose_base_list or len(current_tcp_pose_base_list) != 6:
            print(
                f"[Transform] Error: Failed to get valid TCP pose for {arm_choice}. Got: {current_tcp_pose_base_list}");
            return None
        print(f"[Transform] Current TCP Pose (Base, mm/deg): {current_tcp_pose_base_list}")

        T_base_to_end = create_transformation_matrix(current_tcp_pose_base_list[:3], current_tcp_pose_base_list[3:])

        T_base_to_camera = T_base_to_end @ T_end_to_camera

        pos_base_m = transform_point(pos_cam_m, T_base_to_camera)  # Input point is meters
        pos_base_mm = [p * 1000.0 for p in pos_base_m]  # Convert result back to mm

        print(f"[Transform] Point in Base Frame (mm): {pos_base_mm}")
        return pos_base_mm

    except Exception as e:
        print(f"[Transform] Error during transformation: {e}")
        traceback.print_exc()
        return None


# --- Grasp Calculation and Execution (Remain the same as previous response) ---
def calculate_grasp_poses(object_base_coordinates_mm, arm_to_use, object_orientation_rpy_deg=None):
    """Calculates the sequence of TCP poses (relative to base) for grasping."""
    # ...(Implementation from previous response)...
    print(
        f"[Calculate Grasp] Input Coords (mm): {object_base_coordinates_mm}, Arm: {arm_to_use}, Object RPY: {object_orientation_rpy_deg}")
    if arm_to_use == "left":
        grasp_rpy = list(DEFAULT_LEFT_GRASP_RPY)
    else:
        grasp_rpy = list(DEFAULT_RIGHT_GRASP_RPY)
    if object_orientation_rpy_deg and len(object_orientation_rpy_deg) == 3:
        grasp_rpy[2] = (grasp_rpy[2] + object_orientation_rpy_deg[2]) % 360
    grasp_pose_xyz = [float(c) for c in object_base_coordinates_mm]
    _grasp_pose = grasp_pose_xyz + grasp_rpy
    _pre_grasp_pose = list(_grasp_pose);
    _pre_grasp_pose[2] += PRE_GRASP_OFFSET_Z_MM
    _post_grasp_pose = list(_grasp_pose);
    _post_grasp_pose[2] += POST_GRASP_LIFT_Z_MM
    print(f"[Calculate Grasp] Pre: {_pre_grasp_pose}, Grasp: {_grasp_pose}, Post: {_post_grasp_pose}")
    return _pre_grasp_pose, _grasp_pose, _post_grasp_pose


def execute_grasp_sequence(arm_choice, pre_grasp_pose, grasp_pose, post_grasp_pose, robot_controllers):
    """Executes the calculated grasp sequence."""
    # ...(Implementation from previous response, including move_func selection and error checks)...
    print(f"[Execute Grasp] Starting sequence with {arm_choice} arm.")
    controller = robot_controllers.get(arm_choice)
    if not controller: print(f"[Execute Grasp] Error: Controller for '{arm_choice}' not found."); return False

    if arm_choice == "left":
        move_func, open_gripper_func, close_gripper_func = getattr(controller, 'move_robot', None), getattr(controller,
                                                                                                            'open_gripper',
                                                                                                            None), getattr(
            controller, 'close_gripper', None)
    elif arm_choice == "right":
        move_func, open_gripper_func, close_gripper_func = getattr(controller, 'move_right_robot', None), getattr(
            controller, 'open_gripper', None), getattr(controller, 'close_gripper', None)
    else:
        print(f"[Execute Grasp] Error: Invalid arm_choice '{arm_choice}'."); return False
    if not all([move_func, open_gripper_func, close_gripper_func]): print(
        f"[Execute Grasp] Error: Controller instance for '{arm_choice}' missing methods."); return False

    try:
        print(f"  [{arm_choice.upper()}] Opening gripper...");
        open_gripper_func(speed=DEFAULT_GRIPPER_SPEED, force=DEFAULT_GRIPPER_FORCE, wait=True);
        time.sleep(0.5)
        print(f"  [{arm_choice.upper()}] Moving to Pre-Grasp...");
        success = move_func(list(pre_grasp_pose), speed=DEFAULT_MOVE_SPEED, block=True);
        time.sleep(0.2)
        if not success: print(f"  [{arm_choice.upper()}] Error: Failed Pre-Grasp."); return False
        print(f"  [{arm_choice.upper()}] Moving to Grasp...");
        success = move_func(list(grasp_pose), speed=DEFAULT_GRASP_SPEED, block=True);
        time.sleep(0.5)
        if not success: print(
            f"  [{arm_choice.upper()}] Error: Failed Grasp move."); return False  # Decide recovery later
        print(f"  [{arm_choice.upper()}] Closing gripper...");
        close_gripper_func(speed=DEFAULT_GRIPPER_SPEED, force=DEFAULT_GRIPPER_FORCE, wait=True);
        time.sleep(1.0)
        # Optional Grasp Check Here
        print(f"  [{arm_choice.upper()}] Moving to Post-Grasp...");
        success = move_func(list(post_grasp_pose), speed=DEFAULT_MOVE_SPEED, block=True)
        if not success: print(f"  [{arm_choice.upper()}] Warning: Failed Post-Grasp move.")  # Continue anyway?
        print(f"[Execute Grasp] Sequence finished.")
        return True  # Assume success if sequence completes
    except Exception as e_exec:
        print(f"[Execute Grasp] Unexpected error: {e_exec}"); traceback.print_exc(); return False


# --- Main Interaction Orchestration ---
def initiate_grasp_from_command(command_json, robot_controllers, camera_clients, models, calibration_matrices):
    """Orchestrates the vision-based grasp sequence."""
    # ...(Implementation from previous response, calling the updated functions)...
    action = command_json.get("action")
    if action != "grasp": print(f"[Initiate Grasp] Non-grasp action '{action}'. Skipping."); return False
    target_details = command_json.get("target");
    if not isinstance(target_details, dict): print(
        f"[Initiate Grasp] Error: 'target' invalid: {target_details}"); return False

    object_id = target_details.get("id", "unknown_object")
    approx_base_coords_mm = target_details.get("base_coordinates_mm")
    arm_choice = target_details.get("arm_choice")

    # Validate inputs
    if approx_base_coords_mm is None or not (
            isinstance(approx_base_coords_mm, list) and len(approx_base_coords_mm) == 3): print(
        f"[Initiate Grasp] Error: Invalid 'base_coordinates_mm'"); return False
    if arm_choice not in ["left", "right"]: print(f"[Initiate Grasp] Error: Invalid 'arm_choice'"); return False
    controller = robot_controllers.get(arm_choice)
    yolo_model = models.get(object_id) or models.get('default')
    camera_client = camera_clients.get(f"{arm_choice}_hand")  # Assumes hand camera needed
    calib_matrix = calibration_matrices.get(arm_choice)
    if controller is None:
        print(f"[Initiate Grasp] Error: Controller not available for arm '{arm_choice}'.")
        return False
    if yolo_model is None:
        print(f"[Initiate Grasp] Error: Required YOLO model ('{object_id}' or 'default') not loaded.")
        return False
    if camera_client is None:
        return False
    # Check NumPy array specifically for None
    if calib_matrix is None:
        print(f"[Initiate Grasp] Error: Calibration matrix not available for arm '{arm_choice}'.")
        return False
    print(f"[Initiate Grasp] Processing grasp for '{object_id}' near {approx_base_coords_mm}mm with {arm_choice} arm.")

    grasp_successful = False
    for attempt in range(MAX_OBSERVATION_RETRIES):
        print(f"\n[Initiate Grasp] Attempt {attempt + 1}/{MAX_OBSERVATION_RETRIES}")
        # if not move_arm_to_observe(arm_choice, approx_base_coords_mm, robot_controllers): print(
        #     "Failed move to observe. Retrying..."); time.sleep(0.5); continue
        detection_box, _, depth_frame = capture_and_detect(object_id, camera_client, yolo_model)
        if detection_box is None or depth_frame is None: print("Failed detect. Retrying..."); time.sleep(0.5); continue
        pos_cam_m = calculate_3d_position_camera_frame(detection_box, depth_frame, camera_client)
        if pos_cam_m is None: print("Failed calc cam pos. Retrying..."); time.sleep(0.5); continue
        precise_base_coords_mm = transform_camera_to_base(pos_cam_m, arm_choice, robot_controllers,
                                                          calibration_matrices)
        if precise_base_coords_mm is None: print("Failed transform to base. Retrying..."); time.sleep(0.5); continue

        # Use precise coords now
        object_orientation_rpy_deg = None  # TODO: Get from vision if possible
        poses = calculate_grasp_poses(precise_base_coords_mm, arm_choice, object_orientation_rpy_deg)
        pre_grasp_pose, grasp_pose, post_grasp_pose = poses
        if grasp_pose is None: print("Failed calc grasp poses. Stopping."); break

        grasp_successful = execute_grasp_sequence(arm_choice, pre_grasp_pose, grasp_pose, post_grasp_pose,
                                                  robot_controllers)
        if grasp_successful:
            print("Grasp success!"); break
        else:
            print(f"Grasp sequence failed attempt {attempt + 1}. Retrying..."); time.sleep(1.0)

    if not grasp_successful: print(f"Grasp failed after {MAX_OBSERVATION_RETRIES} attempts.")
    return grasp_successful


# --- Main JSON Handler ---
def handle_command_json(command_json, robot_controllers, camera_clients, models, calibration_matrices):
    """Handles JSON commands, calling the appropriate action handler."""
    # ...(Implementation from previous response, passing all args to initiate_grasp)...
    if not isinstance(command_json, dict): print(f"[Handle Command] Invalid JSON: {command_json}"); return
    print(f"[Handle Command] Received: {command_json}")
    action = command_json.get("action")

    if action == "grasp":
        initiate_grasp_from_command(command_json, robot_controllers, camera_clients, models, calibration_matrices)
    elif action == "play_message":
        message = command_json.get("message", "No message.")
        print(f"[Handle Command] Server message: {message}")
    else:
        print(f"[Handle Command] Unknown action: '{action}'")

# --- How this module is used ---
# This module does not run on its own.
# It is imported by main_controller.py (or similar).
# The main controller:
# 1. Initializes Pygame, robot controllers, cameras, YOLO models, calibration data.
# 2. Runs the main event loop (using ui.py for input/display).
# 3. When in VISION mode and the 'stop record' button is pressed:
#    - Calls vision_interaction.stop_recording_and_save()
#    - Starts a thread running DualArmController._threaded_process_vision_audio(saved_audio_path)
# 4. DualArmController._threaded_process_vision_audio:
#    - Calls vision_interaction.send_audio_and_receive_response()
#    - If JSON is received, prepares dictionaries for controllers, cameras, models, calibration.
#    - Calls vision_interaction.handle_command_json(...) passing these dictionaries.
#    - Calls vision_interaction.play_audio_file() if audio response received.