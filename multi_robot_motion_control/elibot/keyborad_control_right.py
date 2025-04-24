from CPS import *
import cv2

robot_ip = "192.168.188.201"
controller = CPSClient(robot_ip)

if controller.connect():
    formatted_tcp_pos = [round(pos, 2) for pos in controller.getTCPPose()]
    print("pos :", formatted_tcp_pos)
    formatted_joint_pos = [round(pos, 2) for pos in controller.getJointPos()]
    print("joint :", formatted_joint_pos)

    # 创建窗口
    cv2.namedWindow('Robot Control', cv2.WINDOW_NORMAL)

    # 初始化速度为0
    speed = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # [X, Y, Z, Roll, Pitch, Yaw]

    # 定义不同轴的速度变化步长
    step_size_xyz = 20  # XYZ轴的步长
    step_size_rpy = 20  # RPY（Roll, Pitch, Yaw）轴的步长

    acc = 100  # 加速度
    arot = 10  # 姿态加速度
    t = 0.1  # 执行时间

    while True:
        # 显示当前速度
        img = np.zeros((300, 600, 3), dtype=np.uint8)
        cv2.putText(img, f"Speed: {speed}", (50, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(img, "Control Keys:", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(img, "'w'/'s' -> X-axis", (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(img, "'a'/'d' -> Y-axis", (50, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(img, "'q'/'e' -> Z-axis", (50, 160), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(img, "'r'/'f' -> Roll (R)", (50, 190), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(img, "'t'/'g' -> Pitch (P)", (50, 220), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(img, "'y'/'h' -> Yaw (Y)", (50, 250), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(img, "Press 'Esc' to exit", (50, 280), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        cv2.imshow('Robot Control', img)

        # 等待用户输入
        key = cv2.waitKey(100) & 0xFF
        speed = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # [X, Y, Z, Roll, Pitch, Yaw]

        # 控制XYZ速度
        if key == ord('w'):  # X轴正向
            speed[0] = step_size_xyz
        if key == ord('s'):  # X轴负向
            speed[0] = -step_size_xyz
        if key == ord('a'):  # Y轴负向
            speed[1] = -step_size_xyz
        if key == ord('d'):  # Y轴正向
            speed[1] = step_size_xyz
        if key == ord('q'):  # Z轴正向
            speed[2] = step_size_xyz
        if key == ord('e'):  # Z轴负向
            speed[2] = -step_size_xyz

        # 控制RPY速度
        if key == ord('r'):  # Roll正向
            speed[3] = step_size_rpy
        if key == ord('f'):  # Roll负向
            speed[3] = -step_size_rpy
        if key == ord('t'):  # Pitch正向
            speed[4] = step_size_rpy
        if key == ord('g'):  # Pitch负向
            speed[4] = -step_size_rpy
        if key == ord('y'):  # Yaw正向
            speed[5] = step_size_rpy
        if key == ord('h'):  # Yaw负向
            speed[5] = -step_size_rpy

        # 更新速度
        print(f"Current Speed: {speed}")
        speed[0:3] = speed[0:3] @ R.from_euler('xyz', [65.33430565, -4.20854252,-9.07946747], degrees=True).as_matrix()
        speed[3:] = [0, 0, 0]
        # 发送速度命令到机器人
        suc, result, _ = controller.moveBySpeedl(list(speed), acc, arot, t)
        if suc:
            print("Movement command sent successfully.")
        else:
            print("Failed to send movement command.")

        # 按Esc键退出
        if key == 27:
            break

    # 关闭窗口
    cv2.destroyAllWindows()
