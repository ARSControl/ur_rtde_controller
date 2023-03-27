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
  CartesianPoint.msg
)

add_service_files(
  FILES
  RobotiQGripperControl.srv
  GetForwardKinematic.srv
  GetInverseKinematic.srv
  StartFreedriveMode.srv
  GetRobotStatus.srv
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

add_library(polyfit_lib src/polyfit/polyfit.cpp)

# RTDE Controller
add_executable(rtde_controller src/rtde_controller/rtde_controller.cpp)
add_dependencies(rtde_controller ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(rtde_controller ${catkin_LIBRARIES} ur_rtde::rtde polyfit_lib)
