#!/usr/bin/env python

import rospy, time
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
import numpy as np

class JointCommandPublisher():

    # Initialize Variables
    final_time = 5.0

    def __init__(self):

        # Create Publisher and Subscriber
        self.joint_command_pub = rospy.Publisher('/ur_rtde/controllers/joint_space_controller/command', JointTrajectoryPoint, queue_size=1)
        self.subscriber = rospy.Subscriber('/joint_states', JointState, self.robotCallback, queue_size=1)

        # Wait for Subscriber to Connect
        time.sleep(1)

    def robotCallback(self, msg:JointState):

        rospy.logwarn("Compute Command")
        point = JointTrajectoryPoint()

        # Little Movement
        point.positions.append(msg.position[0] + 0.1)
        point.positions.append(msg.position[1] + 0.0)
        point.positions.append(msg.position[2] + 0.0)
        point.positions.append(msg.position[3] + 0.0)
        point.positions.append(msg.position[4] + 0.0)
        point.positions.append(msg.position[5] + 0.0)

        point.time_from_start = rospy.Duration(sec=3, nanosec=0)

        self.joint_command_pub.publish(point)
        rospy.logwarn("Command Published")
        rospy.signal_shutdown("Command Published")

if __name__ == '__main__':

    # Initialize ROS Node
    rospy.init_node('joint_command_publisher', anonymous=True)
    node = JointCommandPublisher()
    rospy.spin()
