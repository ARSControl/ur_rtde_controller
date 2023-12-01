# UR_RTDE Controllers #

Controllers for the UR10e using the RTDE Libraries
https://sdurobotics.gitlab.io/ur_rtde/api/api.html

## Dependencies ##

* ROS2 Foxy
* Eigen3
* ur_rtde

## Installation ##

* Clone the Repository inside the `colcon_ws`:

        cd ~/colcon_ws/src
        git clone git@github.com:ARSControl/ur_rtde_controller.git

* Install the RTDE Libraries

        sudo add-apt-repository ppa:sdurobotics/ur-rtde
        sudo apt-get update
        sudo apt install librtde librtde-dev

* Build your workspace

        cd ~/colcon_ws
        colcon build --symlink-install

## Running ##

* Set the UR Control Mode to `Remote` on the TP

* To use the Robotiq Gripper remember to Activate it from the TP `UR+` Interface

* Launch RTDE Controller
  
        ros2 launch ur_rtde_controller rtde_controller_launch.py ROBOT_IP:=192.168.xx.xx enable_gripper:=true/false
