cmake_minimum_required(VERSION 3.5)
project(ur_rtde_controller)

add_compile_options(-std=c++17)

find_package(Eigen3 REQUIRED)
find_package(ur_rtde REQUIRED)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	message_generation

  std_msgs
  geometry_msgs
  trajectory_msgs
  std_srvs
)

add_message_files(
  FILES
  RobotStatus.msg
  CartesianPoint.msg
)

add_service_files(
  FILES
  RobotiQGripperControl.srv
  GetForwardKinematic.srv
  GetInverseKinematic.srv
  StartFreedriveMode.srv
)

generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
    ${PROJECT_NAME}
)

add_definitions(${EIGEN3_DEFINITIONS})

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    message_runtime
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# Position Controller
add_executable(position_controller src/rtde_controller/position_controller.cpp)
add_dependencies(position_controller ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(position_controller ${catkin_LIBRARIES} ur_rtde::rtde)

# Velocity Controller
add_executable(velocity_controller src/rtde_controller/velocity_controller.cpp)
add_dependencies(velocity_controller ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(velocity_controller ${catkin_LIBRARIES} ur_rtde::rtde)
