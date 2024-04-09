cmake_minimum_required(VERSION 3.5)
project(tutorial_ros2_motor)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED) # 추가됨: 오도메트리 메시지 타입을 사용하기 위해 필요
find_package(tf2_ros REQUIRED) # 추가됨: TF2 변환을 발행하기 위해 필요
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

add_executable(motor_node src/motor_node.cpp)
target_link_libraries(motor_node pigpiod_if2)
ament_target_dependencies(motor_node rclcpp std_msgs)

add_executable(teleop_node src/teleop_node.cpp)
ament_target_dependencies(teleop_node rclcpp std_msgs)

# 추가된 odometry_node 실행 파일
add_executable(odometry_node src/odometry_node.cpp)
ament_target_dependencies(odometry_node rclcpp std_msgs nav_msgs tf2_ros) # 수정됨: nav_msgs와 tf2_ros 의존성 추가


install(TARGETS
  motor_node
  teleop_node
  odometry_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
