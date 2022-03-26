# UR_RTDE Controller #

Controllers for the UR10e using the RTDE Libreries
https://sdurobotics.gitlab.io/ur_rtde/api/api.html

## Dependancies ##

* Eigen3

## Installation ##

* Clone the Repository inside the `catkin_ws`:

        git clone https://davide_ferrari@bitbucket.org/davide_ferrari/ur_rtde_controller.git
        cd ur_rtde_controller
        git submodule update --init --recursive

* Build the RTDE Libraries

        cd ur_rtde
        mkdir build
        cd build
        cmake ..
        make

* Build your workspace

        catkin_make

## Running ##

* Launch Cartesian Controller
  
        roslaunch ur_speed_control cartesian_controller.launch
