#include <rclcpp/rclcpp.hpp>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <memory>
#include <vector>
#include <cmath>

class BiRRTPlanner : public rclcpp::Node {
public:
    BiRRTPlanner() : Node("bi_rrt_planner") {
        // 参数声明
        declare_parameter("planner_range", 0.5);      // 规划器步长(m)
        declare_parameter("safe_distance", 0.5);      // 障碍物安全距离(m)
        declare_parameter("planning_time", 5.0);      // 最大规划时间(s)
        declare_parameter("bounds.min_x", -20.0);     // 环境边界
        declare_parameter("bounds.max_x", 20.0);
        declare_parameter("bounds.min_y", -20.0);
        declare_parameter("bounds.max_y", 20.0);
        declare_parameter("bounds.min_z", 0.0);
        declare_parameter("bounds.max_z", 10.0);

        // 初始化订阅和发布
        start_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "start_pose", 10, std::bind(&BiRRTPlanner::startCallback, this, std::placeholders::_1));
        goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal_pose", 10, std::bind(&BiRRTPlanner::goalCallback, this, std::placeholders::_1));
        obstacles_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
            "obstacles", 10, std::bind(&BiRRTPlanner::obstaclesCallback, this, std::placeholders::_1));
        
        path_pub_ = create_publisher<nav_msgs::msg::Path>("planned_path", 10);
        debug_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("debug_markers", 10);

        // 初始化OMPL状态空间
        setupStateSpace();
        
        RCLCPP_INFO(get_logger(), "Bi-RRT Planner with obstacle avoidance initialized");
    }

private:
    struct Obstacle {
        double x, y, z;
        double radius;
        int id;
        int type;  // 添加类型标识 (2=SPHERE, 3=CYLINDER)
        struct {
            double x, y, z;  // 添加scale成员
        } scale;
    };

    void setupStateSpace() {
        auto space = std::make_shared<ompl::base::SE3StateSpace>();
        
        // 设置环境边界
        ompl::base::RealVectorBounds bounds(3);
        bounds.setLow(0, get_parameter("bounds.min_x").as_double());
        bounds.setHigh(0, get_parameter("bounds.max_x").as_double());
        bounds.setLow(1, get_parameter("bounds.min_y").as_double());
        bounds.setHigh(1, get_parameter("bounds.max_y").as_double());
        bounds.setLow(2, get_parameter("bounds.min_z").as_double());
        bounds.setHigh(2, get_parameter("bounds.max_z").as_double());
        space->setBounds(bounds);

        si_ = std::make_shared<ompl::base::SpaceInformation>(space);
        si_->setStateValidityChecker([this](const ompl::base::State* state) { return isStateValid(state); });
        si_->setStateValidityCheckingResolution(0.05);  // 5cm检查精度
    }

     void startCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        start_pose_ = *msg;
        
        if (!goal_pose_.header.frame_id.empty()) {
            planPath();
        }
    }

    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        goal_pose_ = *msg;
        if (!start_pose_.header.frame_id.empty()) {
            planPath();
        }
    }

    void obstaclesCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
        obstacles_.clear();
        for (const auto& marker : msg->markers) {
            if (marker.action == visualization_msgs::msg::Marker::ADD || 
                marker.action == visualization_msgs::msg::Marker::MODIFY) {
                if (marker.type == visualization_msgs::msg::Marker::SPHERE || 
                    marker.type == visualization_msgs::msg::Marker::CYLINDER) {
                    Obstacle obs;
                    obs.x = marker.pose.position.x;
                    obs.y = marker.pose.position.y;
                    obs.z = marker.pose.position.z;
                    obs.type = marker.type;  // 设置类型
                    obs.scale.x = marker.scale.x;  // 设置scale
                    obs.scale.y = marker.scale.y;
                    obs.scale.z = marker.scale.z;
                    obs.id = marker.id;
                    obstacles_.push_back(obs);
                }
            }
        }
        publishDebugMarkers();
    }

    bool isStateValid(const ompl::base::State* state) const {
        const auto* se3 = state->as<ompl::base::SE3StateSpace::StateType>();
        const double x = se3->getX(), y = se3->getY(), z = se3->getZ();

        // 检查边界
        if (!si_->satisfiesBounds(state)) return false;

        const double safe_dist = get_parameter("safe_distance").as_double();
        
        for (const auto& obs : obstacles_) {
            // 圆柱体障碍物 (type=3)
            if (obs.type == 3) { 
                const double dx = x - obs.x;
                const double dy = y - obs.y;
                const double dist_xy = sqrt(dx*dx + dy*dy);
                const double radius = obs.scale.x / 2.0 + safe_dist;
                const double half_height = obs.scale.z / 2.0;
                const bool in_z_range = (z >= obs.z - half_height) && 
                                    (z <= obs.z + half_height);

                if (dist_xy < radius && in_z_range) {
                    return false;
                }
            }
            // 球体障碍物 (type=2)
            else if (obs.type == 2) { 
                const double dx = x - obs.x;
                const double dy = y - obs.y;
                const double dz = z - obs.z;
                const double dist_sq = dx*dx + dy*dy + dz*dz;
                const double min_dist = obs.scale.x / 2.0 + safe_dist;
                
                if (dist_sq < min_dist*min_dist) return false;
            }
        }
        return true;
    }

    void planPath() {
        // 创建规划问题
        auto pdef = std::make_shared<ompl::base::ProblemDefinition>(si_);
        
        // 设置起始状态
        ompl::base::ScopedState<> start(si_);
        auto* start_se3 = start->as<ompl::base::SE3StateSpace::StateType>();
        start_se3->setXYZ(
            start_pose_.pose.position.x,
            start_pose_.pose.position.y,
            start_pose_.pose.position.z);
        start_se3->rotation().setIdentity();

        // 设置目标状态
        ompl::base::ScopedState<> goal(si_);
        auto* goal_se3 = goal->as<ompl::base::SE3StateSpace::StateType>();
        goal_se3->setXYZ(
            goal_pose_.pose.position.x,
            goal_pose_.pose.position.y,
            goal_pose_.pose.position.z);
        goal_se3->rotation().setIdentity();

        // 验证状态有效性
        if (!si_->isValid(start.get())) {
            RCLCPP_ERROR(get_logger(), "Invalid start state!");
            return;
        }
        if (!si_->isValid(goal.get())) {
            RCLCPP_ERROR(get_logger(), "Invalid goal state!");
            return;
        }

        pdef->setStartAndGoalStates(start, goal);

        // 配置规划器
        auto planner = std::make_shared<ompl::geometric::RRTConnect>(si_);
        planner->setRange(get_parameter("planner_range").as_double());
        planner->setProblemDefinition(pdef);
        planner->setup();

        // 执行规划
        ompl::base::PlannerStatus solved = planner->solve(
            ompl::base::timedPlannerTerminationCondition(get_parameter("planning_time").as_double()));

        // 处理结果
        if (solved) {
            auto path = std::dynamic_pointer_cast<ompl::geometric::PathGeometric>(pdef->getSolutionPath());
            if (path) {
                path->interpolate(100);  // 插值使路径更平滑
                publishPath(*path);
                RCLCPP_INFO(get_logger(), "Found path with %zu states", path->getStateCount());
            }
        } else {
            RCLCPP_WARN(get_logger(), "Planning failed!");
        }
    }

    void publishPath(const ompl::geometric::PathGeometric& path) {
        nav_msgs::msg::Path ros_path;
        ros_path.header.stamp = now();
        ros_path.header.frame_id = "map";

        for (size_t i = 0; i < path.getStateCount(); ++i) {
            const auto* state = path.getState(i)->as<ompl::base::SE3StateSpace::StateType>();
            
            geometry_msgs::msg::PoseStamped pose;
            pose.header = ros_path.header;
            pose.pose.position.x = state->getX();
            pose.pose.position.y = state->getY();
            pose.pose.position.z = state->getZ();
            pose.pose.orientation.w = 1.0;  // 无旋转
            
            ros_path.poses.push_back(pose);
        }

        path_pub_->publish(ros_path);
    }

    void publishDebugMarkers() {
        visualization_msgs::msg::MarkerArray markers;
        
        // 障碍物标记
        for (const auto& obs : obstacles_) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = now();
            marker.ns = "obstacles";
            marker.id = obs.id;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = obs.x;
            marker.pose.position.y = obs.y;
            marker.pose.position.z = obs.z;
            marker.scale.x = obs.radius * 2.0;
            marker.scale.y = obs.radius * 2.0;
            marker.scale.z = obs.radius * 2.0;
            marker.color.r = 1.0;
            marker.color.a = 0.3;
            markers.markers.push_back(marker);
        }

        debug_pub_->publish(markers);
    }

    // 成员变量
    std::vector<Obstacle> obstacles_;
    ompl::base::SpaceInformationPtr si_;
    geometry_msgs::msg::PoseStamped start_pose_;
    geometry_msgs::msg::PoseStamped goal_pose_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr start_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr obstacles_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BiRRTPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}