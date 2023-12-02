#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from typing import List

from std_msgs.msg import Bool
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from ur_rtde_controller.msg import CartesianPoint

from std_srvs.srv import Trigger
from ur_rtde_controller.srv import RobotiQGripperControl, GetForwardKinematic, GetInverseKinematic

GRIPPER_OPEN = 100
GRIPPER_CLOSE = 0
DYNAMIC_PLANNER = False

class UR10e_RTDE_Move(Node):

    trajectory_execution_received = False
    trajectory_executed = False

    def __init__(self, node_name='ur10e_rtde_move'):

        # Initialize ROS node
        super().__init__(node_name)

        # Publishers
        self.ur10Pub      = self.create_publisher(JointState, '/desired_joint_pose', 1)
        self.jointPub     = self.create_publisher(JointTrajectoryPoint, '/ur_rtde/controllers/joint_space_controller/command', 1)
        self.cartesianPub = self.create_publisher(CartesianPoint, '/ur_rtde/controllers/cartesian_space_controller/command', 1)

        # Subscribers
        if DYNAMIC_PLANNER: self.trajectory_execution_sub = self.create_subscription(Bool, '/trajectory_execution', self.trajectoryExecutionCallback, 1)
        else: self.trajectory_execution_sub = self.create_subscription(Bool, '/ur_rtde/trajectory_executed', self.trajectoryExecutionCallback, 1)

        # Init Gripper Service
        self.gripper_client = self.create_client(RobotiQGripperControl, '/ur_rtde/robotiq_gripper/command')

        # IK, FK Services
        self.get_FK_client = self.create_client(GetForwardKinematic, 'ur_rtde/getFK')
        self.get_IK_client = self.create_client(GetInverseKinematic, 'ur_rtde/getIK')

        # Stop Robot Service
        self.stop_service_client = self.create_client(Trigger, '/ur_rtde/controllers/stop_robot')

    def trajectoryExecutionCallback(self, msg:Bool):

        """ Trajectory Execution Callback """

        # Set Trajectory Execution Flags
        self.trajectory_execution_received = True
        self.trajectory_executed = msg.data

    def move_joint(self, joint_positions:List[float]) -> bool:

        """ Joint Space Movement """

        assert type(joint_positions) is list, f"Joint Positions must be a List | {type(joint_positions)} given | {joint_positions}"
        assert len(joint_positions) == 6, f"Joint Positions Length must be 6 | {len(joint_positions)} given"

        if DYNAMIC_PLANNER:

            # Destination Position (if `time_from_start` = 0 -> read velocity[0])
            pos = JointState()
            pos.position = joint_positions

            # Publish Joint Position
            self.ur10Pub.publish(pos)

        else:

            # Destination Position (if `time_from_start` = 0 -> read velocity[0])
            pos = JointTrajectoryPoint()
            pos.time_from_start = Duration(sec=0, nanosec=0)
            pos.velocities = [0.4]
            pos.positions = joint_positions

            # Publish Joint Position
            self.get_logger().warn('Joint Space Movement')
            self.jointPub.publish(pos)

        # Wait for Trajectory Execution
        while not self.trajectory_execution_received and rclpy.ok():

            # Debug Print
            self.get_logger().info('Waiting for Trajectory Execution', throttle_duration_sec=5, skip_first=True)

        # Reset Trajectory Execution Flag
        self.trajectory_execution_received = False

        # Exception with Trajectory Execution
        if not self.trajectory_executed: print("ERROR: An exception occurred during Trajectory Execution"); return False
        else: return True

    def move_cartesian(self, tcp_position:Pose) -> bool:

        """ Cartesian Movement """

        assert type(tcp_position) is Pose, f"Joint Positions must be a Pose | {type(tcp_position)} given | {tcp_position}"

        # Destination Position (if `time_from_start` = 0 -> read velocity[0])
        pos = CartesianPoint()
        pos.cartesian_pose = tcp_position
        pos.velocity = 0.02

        # Publish Cartesian Position
        self.get_logger().warn('Cartesian Movement')
        self.cartesianPub.publish(pos)

        # Wait for Trajectory Execution
        while not self.trajectory_execution_received and rclpy.ok():

            # Debug Print
            self.get_logger().info('Waiting for Trajectory Execution', throttle_duration_sec=5, skip_first=True)

        # Reset Trajectory Execution Flag
        self.trajectory_execution_received = False

        # Exception with Trajectory Execution
        if not self.trajectory_executed: print("ERROR: An exception occurred during Trajectory Execution"); return False
        else: return True

    def FK(self, joint_positions:List[float]) -> Pose:

        # Set Forward Kinematic Request
        req = GetForwardKinematic.Request()
        req.joint_position = joint_positions

        # Wait For Service
        self.get_FK_client.wait_for_service('ur_rtde/getFK')

        # Call Forward Kinematic - Asynchronous
        future = self.get_FK_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res:GetForwardKinematic.Response = future.result()

        return res.tcp_position

    def IK(self, pose:Pose, near_pose:List[float]=None) -> List[float]:

        # Set Inverse Kinematic Request
        req = GetInverseKinematic.Request()
        req.tcp_position = pose

        if near_pose is not None and len(near_pose) == 6: req.near_position = near_pose
        else: req.near_position = []

        # Wait For Service
        self.get_IK_client.wait_for_service('ur_rtde/getIK')

        # Call Inverse Kinematic - Asynchronous
        future = self.get_IK_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res:GetInverseKinematic.Response = future.result()

        return list(res.joint_position)

    def move_gripper(self, position, speed=100, force=100,gripper_enabled=True) -> bool:

        """ Open-Close Gripper Function """

        # Return True if Gripper is not Enabled
        if not gripper_enabled: return True

        # Set Gripper Request
        req = RobotiQGripperControl.Request()
        req.position, req.speed, req.force = position, speed, force

        # Wait For Service
        self.gripper_client.wait_for_service('/ur_rtde/robotiq_gripper/command')

        # Call Gripper Service - Asynchronous
        future = self.gripper_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res:RobotiQGripperControl.Response = future.result()

        return True

    def stop_robot(self) -> bool:

        # Wait For Service
        self.stop_service_client.wait_for_service('/ur_rtde/controllers/stop_robot')

        # Call Stop Service - Asynchronous
        req = Trigger.Request()
        future = self.stop_service_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res:Trigger.Response = future.result()

        return True
