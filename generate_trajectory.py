#!/usr/bin/env python
# license removed for brevity
import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np

joint_pos = np.zeros(6)
final_time = 10.0
trajecory_pub = rospy.Publisher("/ur_rtde/controllers/trajectory_controller/command", JointTrajectory, queue_size=1)

def robotCallback(msg):
  for i in range(len(msg.position)):
    joint_pos[i] = msg.position[i]
      
  A = np.zeros((6,7))
  b = np.zeros((6,1))

  for i in range(7):
    A[0][i] = np.power(0, 6-i)
    A[1][i] = np.power(final_time, 6-i)
    try:
      A[2][i] = (6-i)*np.power(0, 5-i)
    except:
      A[2][i] = 0.0
    try:
      A[3][i] = (6-i)*np.power(final_time, 5-i)
    except:
      A[3][i] = 0.0
    try:
      A[4][i] = (5-i)*(6-i)*np.power(0, 4-i)
    except:
      A[4][i] = 0.0
    try:
      A[5][i] = (5-i)*(6-i)*np.power(final_time, 4-i)
    except:
      A[5][i] = 0.0

  b[0] = joint_pos[0]
  b[1] = joint_pos[0] + 0.05
  b[2] = 0.0
  b[3] = 0.0
  b[4] = 0.0
  b[5] = 0.0

  x = np.linalg.lstsq(A, b)
  T = np.linspace(0, final_time, int(final_time/0.001+1))
  traj = JointTrajectory()
  point = JointTrajectoryPoint()
  for i in range(len(T)):
    point.time_from_start = rospy.Duration(T[i])
    jp = 0.0
    jv = 0.0
    for j in range(7):
      jp = jp + x[0][j]*np.power(T[i], 6-j)
      if(6-j == 0):
        jv = jv + 0.0
      else:
        jv = jv + (6-j)*x[0][j]*np.power(T[i], 5-j)
    p = [0.0]*6
    v = [0.0]*6
    for i in range(6):
      p[i] = joint_pos[i]
      v[i] = 0.0
    p[0] = jp
    v[0] = jv
    for i in range(6):
      point.positions.append(p[i])
      # point.velocities.append(v[i])
    traj.points.append(point)
    point = JointTrajectoryPoint()
  trajecory_pub.publish(traj)
  rospy.signal_shutdown("A")

    
def listener():

  # In ROS, nodes are uniquely named. If two nodes with the same
  # name are launched, the previous one is kicked off. The
  # anonymous=True flag means that rospy will choose a unique
  # name for our 'listener' node so that multiple listeners can
  # run simultaneously.
  rospy.init_node('trajpublish', anonymous=True)
  rospy.Subscriber("/joint_states", JointState, robotCallback)
  
  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()

if __name__ == '__main__':
  listener()
