import rospy
import os
import yaml
from std_msgs.msg import String
from geometry_msgs.msg import Pose  # 新增导入
import numpy as np

class SimpleActionPublisher:
    def __init__(self, topic_name='/simple_action/command', master_uri="http://127.0.0.1:11311", hostname="127.0.0.1"):
        # 设置 ROS 环境变量
        os.environ['ROS_MASTER_URI'] = master_uri
        os.environ['ROS_HOSTNAME'] = hostname

        # 初始化 ROS 节点
        rospy.init_node('simple_action_talker', anonymous=True)

        # 创建发布者
        self.pub = rospy.Publisher(topic_name, String, queue_size=10)

        # 创建订阅者来监听状态消息
        self.state_subscriber = rospy.Subscriber("/simple_action/state", String, self.state_callback)

        # 保存返回的 ID 和状态
        self.response_id = None
        self.status_received = False

        rospy.loginfo("SimpleActionPublisher 初始化完成，并创建了状态订阅者。")

    def publish_message(self, message):
        # 发布消息
        rospy.loginfo(f"发布消息: {message}")
        self.pub.publish(message)

    def state_callback(self, msg):
        # 解析返回的状态消息
        rospy.loginfo(f"收到状态消息: {msg.data}")  # 打印接收到的消息
        state_message = msg.data.split(',')

        rospy.loginfo(f"解析后的状态消息: {state_message}")  # 调试输出，查看解析后的消息

        if len(state_message) >= 2:
            action = state_message[0]
            status = state_message[1]

            if status == "SUCCESS" and len(state_message) > 2:
                # 如果状态是 SUCCESS，获取并保存 ID
                self.response_id = state_message[2]
                rospy.loginfo(f"成功！ID: {self.response_id}")
                self.status_received = True  # 标记收到成功状态
            elif status == "FAIL":
                # 如果状态是 FAIL，抛出错误并标记状态已收到
                error_msg = "动作失败: " + ",".join(state_message[2:])
                rospy.logerr(error_msg)
                self.status_received = True  # 标记收到失败状态
                raise Exception(error_msg)
            elif status == "RUNNING":
                rospy.loginfo("动作正在进行中...")
            else:
                rospy.loginfo(f"未知状态: {status}")

    def start(self, message):
        self.status_received = False
        # 等待直到节点初始化
        rospy.sleep(1)
        # 调用发布消息的方法
        self.publish_message(message)

        # 持续等待直到收到状态消息（SUCCESS 或 FAIL）
        rospy.loginfo("等待动作状态...")

        # 循环等待，直到收到状态消息
        while not self.status_received:
            rospy.sleep(0.5)  # 每次循环等待 0.5 秒

        # 如果收到成功的消息，返回 ID
        if self.response_id:
            return self.response_id
        else:
            return "未收到成功的响应"

class PointReader:
    def __init__(self, yaml_file):
        self.yaml_file = yaml_file
        self.points = self.load_points()

    def load_points(self):
        # 打开并读取 yaml 文件
        with open(self.yaml_file, 'r') as file:
            data = yaml.safe_load(file)
        # 获取 slam_point_id 中的所有点
        return data['slam_point_id']

    def get_point_id(self, point_name):
        # 返回给定点名的 id
        return self.points.get(point_name, None)


def get_robot_pose():
    """
    等待并获取 /robot_pose 话题的消息，返回 numpy 数组，格式为:
    [position.x, position.y, position.z, orientation.x, orientation.y, orientation.z, orientation.w]
    """
    rospy.loginfo("等待 /robot_pose 消息...")
    pose_msg = rospy.wait_for_message('/robot_pose', Pose)
    rospy.loginfo("收到 /robot_pose 消息。")
    # 将 Pose 消息转换为 numpy 数组
    pose_array = np.array([
        pose_msg.position.x,
        pose_msg.position.y,
        pose_msg.position.z,
        pose_msg.orientation.x,
        pose_msg.orientation.y,
        pose_msg.orientation.z,
        pose_msg.orientation.w
    ])
    return pose_array

def main():
    try:
        # 读取 YAML 文件中的 slam_point_id
        reader = PointReader('../../config/position.yaml')

        # 获取某个点的 ID, 例如 point2
        point_name = 'point2'
        point_id = reader.get_point_id(point_name)

        if point_id:
            rospy.loginfo(f"使用 {point_name} 对应的 ID: {point_id}")
            # 创建 SimpleActionPublisher 实例并发布消息
            publisher = SimpleActionPublisher()  # 使用默认参数
            message = f"MOVE_STRAIGHT,{point_id}"
            result = publisher.start(message)

            # 输出返回的 ID 或错误信息
            rospy.loginfo(f"结果: {result}")
        else:
            rospy.logwarn(f"未找到 {point_name} 对应的点 ID！")

        # 启动 ROS 事件循环（如有需要，可取消注释）
        # rospy.spin()

    except rospy.ROSInterruptException as e:
        rospy.logerr(f"错误: {e}")

def main2():
    """
    main2 用于读取并输出 /robot_pose 中的位置信息和朝向信息
    """
    # 初始化 ROS 节点
    rospy.init_node('robot_pose_reader', anonymous=True)
    try:
        pose = get_robot_pose()
        print(pose)
    except rospy.ROSInterruptException as e:
        rospy.logerr(f"读取机器人位置时出错: {e}")

if __name__ == '__main__':
    main2()
