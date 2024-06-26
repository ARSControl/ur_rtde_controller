# UR_RTDE Controller

Controller for the UR10e using the RTDE Libraries
https://sdurobotics.gitlab.io/ur_rtde/api/api.html

## Dependencies

- ROS Noetic
- Eigen3
- ur_rtde

## Installation

- Clone the Repository inside the `catkin_ws`:

        cd ~/catkin_ws/src
        git clone git@github.com:ARSControl/ur_rtde_controller.git

- Install the RTDE Libraries

        sudo add-apt-repository ppa:sdurobotics/ur-rtde
        sudo apt-get update
        sudo apt install librtde librtde-dev

- Install Python Requirements:

        pip install -r ../path/to/this/repo/requirements.txt

- Build your workspace

        cd ~/catkin_ws
        catkin_make

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
  
        roslaunch ur_rtde_controller rtde_controller.launch ROBOT_IP:=192.168.xx.xx enable_gripper:=true/false
