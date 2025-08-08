# Motion_planning_copter_bache

## General Description for Bi-RRT base on Ros2 | 基于Ros2的Bi-RRT简介

本项目旨在实现无人机的运动规划，采用了双向快速随机树（Bi-RRT）算法。该算法能够在复杂环境中为无人机规划一条从起点到终点的可行路径，适用于三维空间的路径规划任务。项目基于 ROS 2 框架开发，便于与其他机器人系统集成。

This project aims to implement motion planning for UAVs (drones) using the Bidirectional Rapidly-exploring Random Tree (Bi-RRT) algorithm. The algorithm can plan a feasible path from start to goal in complex environments, suitable for 3D space path planning. The project is developed based on the ROS 2 framework for easy integration with other robotic systems.

## Prerequisites | 先决条件

- 操作系统：Ubuntu 24.04  
  Operating System: Ubuntu 24.04
- ROS 2（建议版本：jazzy）  
  ROS 2 (Recommended: jazzy)
- colcon 构建工具  
  colcon build tool
- C++17 编译器（如 g++ 9 及以上）  
  C++17 compiler (e.g., g++ 9 or above)
- Python 3.8 及以上（用于辅助脚本）  
  Python 3.8 or above (for auxiliary scripts)
- 依赖库：OMPL Eigen3 rclcpp rmw_fastrtps_cpp rosidl_typesupport\
  Dependencies: OMPL, Eigen3 rclcpp rmw_fastrtps_cpp rosidl_typesupport

## Installation Instruction | 安装说明

1. 克隆本仓库到本地工作空间：  
   Clone this repository to your local workspace:

   ```bash
   git clone <your-repo-url> Motion_planning_copter_bache
   cd Motion_planning_copter_bache
   ```

2. 安装依赖（以 Ubuntu 为例）：  
   Install dependencies (for Ubuntu):SSS
   ```bash
   sudo apt update
   sudo apt install ros-jazzy-desktop python3-colcon-common-extensions libeigen3-dev ros-jazzy-ompl
   ```
   

3. 构建工作空间：  
   Build the workspace:

   ```bash
   cd src
   # 确保 bi_rrt_planner 和 trajectory_to_motors在 src 目录下
   # Make sure bi_rrt_planner and trajectory_to_motors are in the src directory
   cd ..
   colcon build 
   ```

4. 环境配置：  
   Source the environment:

   ```bash
   source install/setup.bash
   ```
  
5. 运行：\
    Run program:

    ```bash
    ros2 run bi_rrt_planner bi_rrt_planner_node
    # 开启两个新的终端
    # 发布起始位置
    ros2 topic pub /start_pose geometry_msgs/msg/PoseStamped "header:
      stamp: {sec: 0, nanosec: 0}
      frame_id: 'map'
    pose:
      position: {x: 0.0, y: 7.0, z: 1.0}
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}"
    # 发布目标位置
    ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped "header:
      stamp: {sec: 0, nanosec: 0}
      frame_id: 'map'
    pose:
      position: {x: 5.0, y: -5.0, z: 3.0}
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}"
    # 发布障碍物
    ros2 topic pub /obstacles visualization_msgs/msg/MarkerArray "
    {markers: [
      {header: {frame_id: 'map', stamp: {sec: 0, nanosec: 0}},
        ns: 'static_obs', id: 0, type: 3, action: 0,
        pose: {position: {x: 2.5, y: 2.5, z: 1.0}},
        scale: {x: 2.0, y: 2.0, z: 2.0},
        color: {r: 1.0, g: 0.0, b: 0.0, a: 0.7}},
      {header: {frame_id: 'map', stamp: {sec: 0, nanosec: 0}},
        ns: 'static_obs', id: 1, type: 1, action: 0,
        pose: {position: {x: -1.0, y: 3.0, z: 0.5}},
        scale: {x: 1.5, y: 3.0, z: 1.0},
        color: {r: 0.0, g: 1.0, b: 0.0, a: 0.6}},
      {header: {frame_id: 'map', stamp: {sec: 0, nanosec: 0}},
        ns: 'static_obs', id: 2, type: 3, action: 0,
        pose: {position: {x: 4.0, y: -2.0, z: 1.5}},
        scale: {x: 1.0, y: 1.0, z: 3.0},
        color: {r: 0.0, g: 0.0, b: 1.0, a: 0.5}}
    ]}"
    ```
6. 可视化：\
    Visible:

    ```bash
    ros2 run rviz2 rviz2
    ```

## Demonstration of the result | 结果演示

[点击观看视频|Watch the video](https://www.bilibili.com/video/BV16otmzoEZH/?spm_id_from=333.1387.homepage.video_card.click&vd_source=1c137efa9119501a36d33da9b3ce3d32)

---

如需进一步帮助，请参考视频或联系开发者。  
For further assistance, please refer to the video or contact the developer.
