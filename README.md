# UR_RTDE Controller #

Controllers for the UR10e using the RTDE Libreries
https://sdurobotics.gitlab.io/ur_rtde/api/api.html

## Dependancies ##

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

* Launch RTDE Controller
  
        roslaunch ur_rtde_controller rtde_controller.launch
