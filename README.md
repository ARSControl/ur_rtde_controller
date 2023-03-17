# UR_RTDE Controllers #

Controllers for the UR10e using the RTDE Libraries
https://sdurobotics.gitlab.io/ur_rtde/api/api.html

## Dependencies ##

* Eigen3
* ur_rtde

## Installation ##

* Clone the Repository inside the `catkin_ws`:

        cd ~/catkin_ws/src
        git clone git@github.com:ARSControl/ur_rtde_controller.git

* Install the RTDE Libraries

        sudo add-apt-repository ppa:sdurobotics/ur-rtde
        sudo apt-get update
        sudo apt install librtde librtde-dev

* Build your workspace

        cd ~/catkin_ws
        catkin_make

## Running ##

* Set the UR Control Mode to `Remote` on the TP

* To use the Robotiq Gripper remember to Activate it from the TP `UR+` Interface

* Launch RTDE Controller
  
        roslaunch ur_rtde_controller rtde_controller.launch ROBOT_IP:=192.168.xx.xx enable_gripper:=true/false
