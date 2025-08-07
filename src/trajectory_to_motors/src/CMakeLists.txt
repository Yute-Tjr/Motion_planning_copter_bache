cmake_minimum_required(VERSION 3.5)
project(trajectory_to_motors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)

# 编译节点
add_executable(trajectory_converter src/trajectory_converter_node.cpp)
ament_target_dependencies(trajectory_converter
  rclcpp geometry_msgs std_msgs mavros_msgs)

# 安装可执行文件
install(TARGETS
  trajectory_converter
  DESTINATION lib/${PROJECT_NAME})

ament_package()
