#!/usr/bin/env python

import rospy, time
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np

class TrajectoryPublisher():

    # Initialize Variables
    joint_pos = np.zeros(6)
    final_time = 5.0

    def __init__(self):

        # Create Publisher and Subscriber
        self.trajectory_pub = rospy.Publisher("/ur_rtde/controllers/trajectory_controller/command", JointTrajectory, queue_size=1)
        self.subscriber = rospy.Subscriber("/joint_states", JointState, self.robotCallback, queue_size=1)

        # Wait for Subscriber to Connect
        time.sleep(1)

    def robotCallback(self, msg:JointState):

        rospy.logwarn("Compute Trajectory")

        for i in range(len(msg.position)):
            self.joint_pos[i] = msg.position[i]

        A = np.zeros((6, 7))
        b = np.zeros((6, 1))

        for i in range(7):
            A[0][i] = np.power(0, 6-i)
            A[1][i] = np.power(self.final_time, 6-i)
            try: A[2][i] = (6-i)*np.power(0, 5-i)
            except: A[2][i] = 0.0
            try: A[3][i] = (6-i)*np.power(self.final_time, 5-i)
            except: A[3][i] = 0.0
            try: A[4][i] = (5-i)*(6-i)*np.power(0, 4-i)
            except: A[4][i] = 0.0
            try: A[5][i] = (5-i)*(6-i)*np.power(self.final_time, 4-i)
            except: A[5][i] = 0.0

        b[0] = self.joint_pos[0]
        b[1] = self.joint_pos[0] - 0.5
        b[2] = 0.0
        b[3] = 0.0
        b[4] = 0.0
        b[5] = 0.0

        x = np.linalg.lstsq(A, b)
        T = np.linspace(0, self.final_time, int(self.final_time/0.01+1))
        traj = JointTrajectory()
        point = JointTrajectoryPoint()

        for i in range(len(T)):
            point.time_from_start = rospy.Duration(secs=int(T[i]), nsecs=int((T[i] - int(T[i])) * 1e9))
            jp, jv = 0.0, 0.0
            for j in range(7):
                jp = jp + x[0][j]*np.power(T[i], 6-j)
                if(6-j == 0):
                    jv = jv + 0.0
                else:
                    jv = jv + (6-j)*x[0][j]*np.power(T[i], 5-j)
            p = [0.0]*6
            v = [0.0]*6
            for i in range(6):
                p[i] = self.joint_pos[i]
                v[i] = 0.0
            p[0] = jp
            v[0] = jv

            for i in range(6): 
                point.positions.append(p[i])
                point.velocities.append(v[i])

            traj.points.append(point)

            point = JointTrajectoryPoint()

        self.trajectory_pub.publish(traj)
        rospy.logwarn("Trajectory Published")

        # Do Nothing
        while not rospy.is_shutdown(): pass

if __name__ == '__main__':

    # Initialize ROS Node
    rospy.init_node('trajectory_publisher', anonymous=True)
    node = TrajectoryPublisher()
    rospy.spin()
