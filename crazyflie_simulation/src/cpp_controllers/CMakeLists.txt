cmake_minimum_required(VERSION 3.8)
set(CMAKE_CXX_STANDARD 20)

project(cpp_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(actuator_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# additional packages
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})


add_executable(motor_control_node
    src/MotorControlNode.cpp
    src/pid_controller.cpp  #
)
ament_target_dependencies(motor_control_node rclcpp std_msgs)
ament_target_dependencies(motor_control_node rclcpp actuator_msgs)
ament_target_dependencies(motor_control_node rclcpp nav_msgs)
# include_directories(include/cpp_controllers)
target_include_directories(motor_control_node PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/cpp_controllers)


install(TARGETS
  motor_control_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

#add_compile_options(-Wno-unused-variable)