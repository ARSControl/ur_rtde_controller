# UR_RTDE Controller

Controller for the UR10e using the RTDE Libraries
https://sdurobotics.gitlab.io/ur_rtde/api/api.html

## Dependencies

- ROS2 Foxy
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

- Build your workspace

        cd ~/colcon_ws
        colcon build --symlink-install

## Build New Robot Kinematic Libraries

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
  
        ros2 launch ur_rtde_controller rtde_controller_launch.py ROBOT_IP:=192.168.xx.xx enable_gripper:=true/false
