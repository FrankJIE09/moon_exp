# move_to_target.py

import rospy
import tf
import numpy as np
from geometry_msgs.msg import PoseStamped
import sagepy

_listener = None

def init_pose_utils():
    """初始化 TF 监听器，如果未初始化则初始化 rospy"""
    global _listener
    if not rospy.core.is_initialized():
        rospy.init_node("move_to_target_utils", anonymous=True)
    _listener = tf.TransformListener()
    rospy.loginfo("TF Listener initialized")

def tf_to_matrix(translation, rotation):
    """将 TF 的 translation + quaternion 转换为 4x4 变换矩阵"""
    T = tf.transformations.quaternion_matrix(rotation)
    T[0:3, 3] = translation
    return T

def get_tf_matrix():
    global _listener
    if _listener is None:
        init_pose_utils()

    rospy.loginfo("Waiting for transform between map and base_link...")
    _listener.waitForTransform("map", "base_link", rospy.Time(0), rospy.Duration(4.0))

    try:
        (trans, rot) = _listener.lookupTransform("map", "base_link", rospy.Time(0))
        T = tf_to_matrix(trans, rot)
        return T
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logerr("Transform lookup failed: %s", e)
        return None

def get_target_pose(target_in_base_link):
    T = get_tf_matrix()
    if T is None:
        rospy.logerr("get transform from base_link to map failed")
        return None

    point_vehicle = np.array([*target_in_base_link, 1.0])
    point_map = T @ point_vehicle

    robot_pose = sagepy.get_robot_pose()
    rospy.loginfo(f"robot_pose: {robot_pose}")

    target_pose = PoseStamped()
    target_pose.header.frame_id = "map"
    target_pose.header.stamp = rospy.Time.now()
    target_pose.pose.position.x = point_map[0]
    target_pose.pose.position.y = point_map[1]
    target_pose.pose.position.z = point_map[2]
    target_pose.pose.orientation = robot_pose.orientation

    rospy.loginfo(f"目标点 PoseStamped:\n{target_pose}")
    return target_pose

def move_to_target_pose(target_in_base_link):
    """
    统一接口：输入 base_link 下的目标点，自动转换并执行移动动作
    """
    target_pose = get_target_pose(target_in_base_link)
    if target_pose:
        sagepy.move_arc(
            target_pose_stamped=target_pose,
            control_frame="map",
            is_adjust_accurately=True,
            radius=0.25,
            speed=0.1,
            free_speed=False,
            stop_distance=0.0
        )
        sagepy.turn(
            target_pose=target_pose.pose,
            theta_tolerance=0.01,
            total_time=0.005
        )
