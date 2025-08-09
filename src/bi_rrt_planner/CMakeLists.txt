cmake_minimum_required(VERSION 3.8)
project(bi_rrt_planner)

# 查找必要依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# 关键修改：添加OMPL查找
find_package(ompl REQUIRED)
include_directories(${OMPL_INCLUDE_DIRS})

add_executable(bi_rrt_planner_node src/bi_rrt_planner_node.cpp)
target_include_directories(bi_rrt_planner_node PUBLIC
  ${OMPL_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(bi_rrt_planner_node PUBLIC c_std_99 cxx_std_17)
target_link_libraries(bi_rrt_planner_node 
  ${OMPL_LIBRARIES}
)
ament_target_dependencies(bi_rrt_planner_node
  rclcpp
  geometry_msgs
  nav_msgs
  visualization_msgs
)

install(TARGETS bi_rrt_planner_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()