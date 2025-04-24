import rospy

import sagepy

rospy.init_node("test")
sagepy.rotate(theta=1.57, rad=0.3)

# sagepy.move_by_distance(distance=0.2, speed=-0.2)
