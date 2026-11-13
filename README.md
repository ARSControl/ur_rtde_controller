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

## Independent output-only feedback

The optional `rtde_feedback_collector` is a separate ROS process. Its launch
file starts only that collector, not the motion controller. It negotiates a
standard RTDE output recipe and publishes complete decoded packets and health;
it does not create a control or IO interface, write robot input registers,
upload scripts or issue motion or stop commands. Starting it still opens a
network connection and adds controller/network load. It requires separate
recording, a declared measurement configuration and qualification before
physical-response latency can be reported.

After reviewing the recording procedure and sourcing a verified build, the
collector-only launch uses:

```bash
ros2 launch ur_rtde_controller rtde_feedback_collector.launch.py \
  robot_ip:=192.168.2.40 frequency:=500.0 queue_capacity:=512 \
  expected_version:=5.9.3 timeout_s:=3.0
```

The packet topic is `/ur_rtde/diagnostics/rtde_feedback_packet`; collector
health is `/ur_rtde/diagnostics/rtde_feedback_health`. Local packet sequence
and source timestamp gaps do not prove robot-side packet loss or losslessness.
The collector never reconnects automatically: a fault or disconnect ends the
session, and restarting it creates a new session. The observer must recheck
its baseline and cannot silently resume a trial. If no complete RTDE data
packet arrives within `timeout_s`, the session faults; text messages do not
extend this deadline. Frame and TCP configuration
must be independently established; the collector does not validate them.
