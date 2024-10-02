cmake_minimum_required(VERSION 3.5)
project(ur_rtde_controller)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

# Find Dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

# Find Messages Dependencies
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Find Messages Generation Dependencies
find_package(rosidl_default_generators REQUIRED)

# Eigen3 Dependencies
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# UR_RTDE Dependencies
find_package(ur_rtde REQUIRED)

# Set Cusom Messages
set(msg_files
  "msg/CartesianPoint.msg"
)

# Set Custom Services
set(srv_files
  "srv/RobotiQGripperControl.srv"
  "srv/GetForwardKinematic.srv"
  "srv/GetInverseKinematic.srv"
  "srv/StartFreedriveMode.srv"
  "srv/GetRobotStatus.srv"
  "srv/GetGripperPosition.srv"
)

# Generate Messages, Services and Actions
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES  std_msgs sensor_msgs geometry_msgs trajectory_msgs std_srvs
)

# add_definitions(${EIGEN3_DEFINITIONS})

# Include C++ Directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${${PROJECT_NAME}_INCLUDE_DIRS}
)

add_library(polyfit_lib src/polyfit/polyfit.cpp)

# RTDE Controller - C++ Executables
add_executable(rtde_controller src/rtde_controller/rtde_controller.cpp)
ament_target_dependencies(rtde_controller rclcpp std_msgs sensor_msgs geometry_msgs trajectory_msgs std_srvs)
rosidl_target_interfaces(rtde_controller ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(rtde_controller ur_rtde::rtde polyfit_lib)

# Install C++ Executables
install(TARGETS
  rtde_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python Modules
ament_python_install_package(scripts)

# Install Python Scripts
install(PROGRAMS
  scripts/joint_command_publisher.py
  scripts/generate_trajectory.py
  scripts/move_robot.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install Launch Files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  #set(ament_cmake_copyright_FOUND TRUE)
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
