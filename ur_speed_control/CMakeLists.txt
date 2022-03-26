cmake_minimum_required(VERSION 2.8.3)
add_compile_options(-std=c++11)

project(ur_speed_control)

find_package(catkin REQUIRED COMPONENTS
	roscpp
)

find_package(ur_rtde REQUIRED PATHS ../ur_rtde/build/ur_rtde)
find_package(Eigen3 REQUIRED)
#finding the package it not enough, it is required to include Eigen3 directories
#because the path of the header is /usr/local/include/eigen3/Eigen/... 
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN3_DEFINITIONS})

catkin_package()

include_directories(
	${catkin_INCLUDE_DIRS}
	include
)

add_executable(ur_speed_control src/ur_speed_control/ur_speed_control.cpp)
target_link_libraries(ur_speed_control ${catkin_LIBRARIES})
target_link_libraries(ur_speed_control PRIVATE ur_rtde::rtde)

add_executable(ur_force_control src/ur_force_control/ur_force_control.cpp)
target_link_libraries(ur_force_control ${catkin_LIBRARIES})
target_link_libraries(ur_force_control PRIVATE ur_rtde::rtde)

add_executable(ur_joint_position_control src/ur_joint_position_control/ur_joint_position_control.cpp)
target_link_libraries(ur_joint_position_control ${catkin_LIBRARIES})
target_link_libraries(ur_joint_position_control PRIVATE ur_rtde::rtde)

add_executable(whole_body_admittance src/whole_body_admittance/whole_body_admittance.cpp)
target_link_libraries(whole_body_admittance ${catkin_LIBRARIES})
target_link_libraries(whole_body_admittance PRIVATE ur_rtde::rtde)

add_executable(speed_controller src/speed_controller/speed_controller.cpp)
target_link_libraries(speed_controller ${catkin_LIBRARIES})






