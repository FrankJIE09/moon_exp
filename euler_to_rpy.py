import numpy as np
from scipy.spatial.transform import Rotation as R

def calculate_rotated_rpy(rx_deg, ry_deg, rz_deg):
    """
    计算绕 x, y, z 轴旋转指定角度后的旋转矩阵乘积，并给出结果的 RPY 角。

    Args:
        rx_deg (float): 绕 x 轴的旋转角度，单位为度。
        ry_deg (float): 绕 y 轴的旋转角度，单位为度。
        rz_deg (float): 绕 z 轴的旋转角度，单位为度。

    Returns:
        dict: 一个字典，包含所有可能的旋转顺序及其对应的旋转矩阵和 RPY 角（以度为单位）。
              字典的键是旋转顺序的字符串 (例如: 'xyz')，值是一个包含 'rotation_matrix' (numpy.ndarray)
              和 'rpy_deg' (numpy.ndarray) 的字典。
    """
    rx_rad = np.deg2rad(rx_deg)
    ry_rad = np.deg2rad(ry_deg)
    rz_rad = np.deg2rad(rz_deg)

    rotations = {
        'xyz': R.from_euler('xyz', [rx_rad, ry_rad, rz_rad]),
        # 'xzy': R.from_euler('xzy', [rx_rad, rz_rad, ry_rad]),
        # 'yxz': R.from_euler('yxz', [ry_rad, rx_rad, rz_rad]),
        # 'yzx': R.from_euler('yzx', [ry_rad, rz_rad, rx_rad]),
        # 'zxy': R.from_euler('zxy', [rz_rad, rx_rad, ry_rad]),
        # 'zyx': R.from_euler('zyx', [rz_rad, ry_rad, rx_rad]),
    }

    results = {}
    for order, rot in rotations.items():
        rotation_matrix = rot.as_matrix()
        rpy_rad = rot.as_euler('xyz')  # 通常 RPY 对应的是 'xyz' 顺序
        rpy_deg = np.rad2deg(rpy_rad)
        results[order] = {
            'rotation_matrix': rotation_matrix,
            'rpy_deg': rpy_deg
        }

    return results

if __name__ == '__main__':
    rx = 180.0
    ry = 0.0
    rz = 180.0
    results = calculate_rotated_rpy(rx, ry, rz)
    for order, result in results.items():
        print(f"旋转顺序: {order}")
        print("旋转矩阵:")
        print(result['rotation_matrix'])
        print("RPY 角度 (度):", result['rpy_deg'])
        print("-" * 30)