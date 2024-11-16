cmake_minimum_required(VERSION 3.8)
set(CMAKE_CXX_STANDARD 20)

project(cpp_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(actuator_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

#----------------------------additional packages-----------------------------------------------------
find_package(Eigen3 REQUIRED)
# Manually set the include path if Eigen3_INCLUDE_DIRS is empty
if(NOT Eigen3_INCLUDE_DIRS)
  message(STATUS "Eigen3_INCLUDE_DIRS is empty, setting it manually")
  set(Eigen3_INCLUDE_DIRS /usr/include/eigen3)
endif()
#----------------------------additional packages-----------------------------------------------------

#----------------------------Add the executables-----------------------------------------------------
add_executable(motor_control_node
    src/MotorControlNode.cpp
    src/pid_controller.cpp  
)

add_executable(keyboard_control_node
    src/KeyboardControlNode.cpp
)
#----------------------------Add the executables-----------------------------------------------------

# To use ros2 messages and services
ament_target_dependencies(motor_control_node rclcpp)
ament_target_dependencies(motor_control_node std_msgs)
ament_target_dependencies(motor_control_node actuator_msgs)
ament_target_dependencies(motor_control_node nav_msgs)
ament_target_dependencies(motor_control_node rosgraph_msgs)
ament_target_dependencies(keyboard_control_node std_msgs)
ament_target_dependencies(keyboard_control_node rclcpp)


# include_directories(include/cpp_controllers)
target_include_directories(motor_control_node PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/cpp_controllers)
target_include_directories(keyboard_control_node PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/cpp_controllers)
target_include_directories(motor_control_node PUBLIC ${Eigen3_INCLUDE_DIRS})


install(TARGETS motor_control_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS keyboard_control_node DESTINATION lib/${PROJECT_NAME})

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


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-missing-field-initializers -Wno-unused-parameter")
