# UR_RTDE Controller

Controller for the UR10e using the RTDE Libraries
https://sdurobotics.gitlab.io/ur_rtde/api/api.html

## Dependencies

- ROS2 Humble
- Eigen3
- ur_rtde

## Installation

- Clone the Repository inside the `colcon_ws`:

        cd ~/colcon_ws/src
        git clone git@github.com:ARSControl/ur_rtde_controller.git

- Install the RTDE Libraries

        sudo add-apt-repository ppa:sdurobotics/ur-rtde
        sudo apt-get update
        sudo apt install librtde librtde-dev

- Install Python Requirements:

        pip install -r ../path/to/this/repo/requirements.txt

- Build your workspace

        cd ~/colcon_ws
        colcon build --symlink-install

## Build New Robot Kinematic Libraries

The Kinematics Libraries are already available for the following robots:

- CB3 Series (UR3, UR5, UR10)
- e-Series (UR3e, UR5e, UR10e, UR16e)

If you want to add a new robot, follow these steps:

- Install `invoke`:

        pip install invoke

- Create the Kinematic Source Files in `src/kinematic/robot_name_kinematic`:

  - `compute_robot_name_direct_kinematic.cpp`
  - `compute_robot_name_jacobian.cpp`
  - `compute_robot_name_jacobian_dot_dq.cpp`

- Use <https://github.com/ARSControl/robot_kinematic> to generate the Robot Kinematic Source Files (Little Manual Edit is Needed).

- Edit the `src/kinematic/tasks.py` build file adding the new source and destination path.

- Build the Robot Kinematic Library:

        cd path/to/package/src
        invoke build

- Add the new libraries to the `scripts/kinematic_wrapper` script file.

## Running

- Set the UR Control Mode to `Remote` on the TP

- To use the RobotiQ Gripper remember to Activate it from the TP `UR+` Interface

- Launch RTDE Controller

        ros2 launch ur_rtde_controller rtde_controller.launch.py ROBOT_IP:=192.168.xx.xx enable_gripper:=true/false

## Joint Trajectory Controller

Topic: `/ur_rtde/controllers/trajectory_controller/command` (`trajectory_msgs/JointTrajectory`)

- The trajectory is executed at the controller `rate` (default 500 Hz -> 2 ms).
- Trajectories already sampled at `1/rate` (with velocities) are executed as they are, otherwise they are resampled:
  - positions + velocities + accelerations -> quintic segments
  - positions + velocities -> cubic Hermite segments
  - positions only -> clamped cubic spline (zero initial and final velocity)
- The first point must be within `trajectory_start_tolerance` [rad] of the actual joint position, the initial velocity must match the actual one and the final velocity must be zero.
- `joint_names` (if given) are used to reorder the joints.
- A new trajectory replaces the one in execution.
- At the end, `/ur_rtde/trajectory_executed` publishes `true` if the final point is reached within `trajectory_goal_tolerance` [rad], `false` otherwise.

## Torque Mode (Direct Joint Torque Control)

Requires PolyScope >= 5.23 (e-Series / UR Series). The controller only exposes the robot dynamics and forwards torque commands: the control law (e.g. impedance) runs in an external node.

- Start / stop: `/ur_rtde/torque_mode/start`, `/ur_rtde/torque_mode/stop` (`std_srvs/Trigger`). `/ur_rtde/torque_mode/active` (`std_msgs/Bool`, transient local) reports the state.
- Published at every robot cycle: `/ur_rtde/dynamics` (`RobotDynamics`): joint position, velocity, measured torque, mass matrix `M(q)`, Coriolis/centrifugal torques `C(q,dq)*dq`, TCP Jacobian and its time derivative (row-major), TCP pose and wrench.
- Command: `/ur_rtde/controllers/torque_controller/command` (`std_msgs/Float64MultiArray`, 6 joint torques [Nm]). Gravity is compensated by the robot: do not add it. The command is only saturated to `torque_limits`.
- Friction compensation (UR internal): default from the `torque_friction_compensation` param (`true`), can be changed with `/ur_rtde/torque_mode/set_friction_compensation` (`std_srvs/SetBool`).
- Seen from the command, the robot dynamics is `M(q) ddq + C(q, dq) dq = tau_cmd + tau_ext`. Note that `coriolis` in `RobotDynamics` is already the vector `C(q, dq) dq`, e.g. computed torque: `tau_cmd = M ddq_des + coriolis`.
- The robot stays in position control until the first command arrives (timeout 1 s). The last command is held for up to `torque_watchdog_cycles` missed cycles; after that, torque mode is stopped and the robot is stopped. Torque mode also stops on emergency / protective stop.
- While torque mode is active, motion commands, freedrive, FK/IK and FT zeroing are rejected.

Parameters: `torque_limits` (default `[50, 50, 25, 10, 10, 10]` Nm, set them for your robot), `torque_watchdog_cycles` (default 5), `torque_friction_compensation` (default `true`).
