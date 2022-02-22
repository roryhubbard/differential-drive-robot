cmake_minimum_required(VERSION 3.8)
project(robotino_autonomy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robotino_msgs REQUIRED)

set(target "autonomy")

add_executable(${target} src/${target}.cpp)

ament_target_dependencies(${target}
  "rclcpp"
  "rclcpp_action"
  "nav_msgs"
  "geometry_msgs"
  "robotino_msgs")

install(TARGETS ${target}
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
