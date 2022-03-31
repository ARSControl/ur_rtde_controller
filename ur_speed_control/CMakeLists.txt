cmake_minimum_required(VERSION 2.8.3)
add_compile_options(-std=c++11)

project(ur_speed_control)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	message_generation
)

find_package(ur_rtde REQUIRED PATHS ../ur_rtde/build/ur_rtde)
find_package(Eigen3 REQUIRED)

add_service_files(
  FILES
  command_gripper.srv
)

generate_messages(
  DEPENDENCIES
    std_msgs
    ${PROJECT_NAME}
)

add_definitions(${EIGEN3_DEFINITIONS})

catkin_package()

include_directories(
	${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
	include
)

add_executable(ur_speed_control src/ur_speed_control/ur_speed_control.cpp src/ur_speed_control/ur_speed_control_node.cpp)
add_dependencies(ur_speed_control ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(ur_speed_control ${catkin_LIBRARIES})
target_link_libraries(ur_speed_control PRIVATE ur_rtde::rtde)

add_executable(ur_force_control src/ur_force_control/ur_force_control.cpp)
add_dependencies(ur_speed_control ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(ur_force_control ${catkin_LIBRARIES})
target_link_libraries(ur_force_control PRIVATE ur_rtde::rtde)

add_executable(ur_joint_position_control src/ur_joint_position_control/ur_joint_position_control.cpp)
add_dependencies(ur_speed_control ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(ur_joint_position_control ${catkin_LIBRARIES})
target_link_libraries(ur_joint_position_control PRIVATE ur_rtde::rtde)

add_executable(whole_body_admittance src/whole_body_admittance/whole_body_admittance.cpp)
add_dependencies(ur_speed_control ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(whole_body_admittance ${catkin_LIBRARIES})
target_link_libraries(whole_body_admittance PRIVATE ur_rtde::rtde)

add_executable(cartesian_controller src/cartesian_controller/cartesian_controller.cpp)
add_dependencies(ur_speed_control ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(cartesian_controller ${catkin_LIBRARIES})
