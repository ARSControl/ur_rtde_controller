#!/usr/bin/env python

import rclpy, time
from rclpy.node import Node
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np

class JointCommandPublisher(Node):

    # Initialize Variables
    final_time = 5.0

    def __init__(self, node_name='trajectory_publisher'):

        # Node Initialization
        super().__init__(node_name)

        # Create Publisher and Subscriber
        self.joint_command_pub = self.create_publisher(JointTrajectoryPoint, '/ur_rtde/controllers/joint_space_controller/command', 1)
        self.subscriber = self.create_subscription(JointState, '/joint_states', self.robotCallback, 1)

        # Wait for Subscriber to Connect
        time.sleep(1)

    def robotCallback(self, msg:JointState):

        self.get_logger().warn("Compute Command")
        point = JointTrajectoryPoint()

        # Little Movement
        point.positions.append(msg.position[0] + 0.1)
        point.positions.append(msg.position[1] + 0.0)
        point.positions.append(msg.position[2] + 0.0)
        point.positions.append(msg.position[3] + 0.0)
        point.positions.append(msg.position[4] + 0.0)
        point.positions.append(msg.position[5] + 0.0)

        point.time_from_start = Duration(sec=3, nanosec=0)

        self.joint_command_pub.publish(point)
        self.get_logger().warn("Command Published")
        rclpy.shutdown()

if __name__ == '__main__':

    # Initialize ROS Node
    rclpy.init(args=None)
    node = JointCommandPublisher()
    rclpy.spin(node)
