cmake_minimum_required(VERSION 3.8)
project(trajectory_to_motors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖（添加AirSim相关）
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(airsim_ros_pkgs REQUIRED)  # 添加AirSim ROS包依赖

add_executable(trajectory_converter_node src/trajectory_converter_node.cpp)
target_include_directories(trajectory_converter_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(trajectory_converter_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  trajectory_converter_node
  rclcpp
  geometry_msgs
  std_msgs
  mavros_msgs
  nav_msgs
  airsim_ros_pkgs  # 链接AirSim依赖
)

install(TARGETS trajectory_converter_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)

  set(ament_cmake_cpplint_FOUND TRUE)
  
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
