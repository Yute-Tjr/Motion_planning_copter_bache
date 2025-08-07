#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "airsim_ros_msgs/srv/set_local_position.hpp"  // AirSim服务


// 坐标系转换：AirSim的NED(北东地)到ENU(东北天)
geometry_msgs::msg::Point ned_to_enu(const geometry_msgs::msg::Point& ned) {
    geometry_msgs::msg::Point enu;
    enu.x = ned.y;   // 东 = 北东地的东
    enu.y = ned.x;   // 北 = 北东地的北
    enu.z = -ned.z;  // 天 = 北东地的地的负值
    return enu;
}

// 速度坐标系转换
geometry_msgs::msg::Vector3 ned_to_enu_vel(const geometry_msgs::msg::Vector3& ned_vel) {
    geometry_msgs::msg::Vector3 enu_vel;
    enu_vel.x = ned_vel.y;
    enu_vel.y = ned_vel.x;
    enu_vel.z = -ned_vel.z;
    return enu_vel;
}

const float k = 0.01;     // 升力系数
const float l = 0.15;     // 螺旋桨到机体中心的距离(m)
const float b = 0.001;    // 阻力系数
const float m = 1.2;      // 四旋翼质量(kg)
const float g = 9.8;      // 重力加速度(m/s²)

// 串级PID参数

const float Kp_pos = 8.0, Ki_pos = 0.05, Kd_pos = 1.2;

const float Kp_vel = 4.0, Ki_vel = 0.02, Kd_vel = 0.8;

class TrajectoryConverterNode : public rclcpp::Node {
public:
    TrajectoryConverterNode() : Node("trajectory_converter_node") {
        // 订阅Bi-RRT规划的路径点
        traj_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/planned_path", 10,  // 订阅规划器输出的路径
            std::bind(&TrajectoryConverterNode::trajectory_callback, this, std::placeholders::_1));
        
        // 订阅AirSim的无人机状态（里程计）
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/airsim_node/drone_1/odom_local_ned", 10,  // AirSim的NED坐标系里程计
            std::bind(&TrajectoryConverterNode::odom_callback, this, std::placeholders::_1));
        
        // 发布电机控制指令到AirSim
        motor_pub_ = this->create_publisher<mavros_msgs::msg::ActuatorControl>(
            "/mavros/actuator_control", 10);  // 使用mavros接口控制电机

        // 初始化PID积分项
        integral_pos_x_ = 0.0;
        integral_pos_y_ = 0.0;
        integral_pos_z_ = 0.0;
        integral_vel_x_ = 0.0;
        integral_vel_y_ = 0.0;
        integral_vel_z_ = 0.0;

        // 记录上一时刻误差
        last_err_pos_x_ = 0.0;
        last_err_pos_y_ = 0.0;
        last_err_pos_z_ = 0.0;
        last_err_vel_x_ = 0.0;
        last_err_vel_y_ = 0.0;
        last_err_vel_z_ = 0.0;

        // 定时器，固定频率控制
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),  // 100Hz控制频率
            std::bind(&TrajectoryConverterNode::control_loop, this));
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // 将AirSim的NED坐标转换为ENU
        auto enu_pos = ned_to_enu(msg->pose.pose.position);
        current_x_ = enu_pos.x;
        current_y_ = enu_pos.y;
        current_z_ = enu_pos.z;

        // 速度转换
        auto enu_vel = ned_to_enu_vel(msg->twist.twist.linear);
        current_vx_ = enu_vel.x;
        current_vy_ = enu_vel.y;
        current_vz_ = enu_vel.z;
    }

    void trajectory_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        // 接收规划的目标位置
        desired_x_ = msg->pose.position.x;
        desired_y_ = msg->pose.position.y;
        desired_z_ = msg->pose.position.z;
    }

    void control_loop() {
        if (!is_initialized_) {
            // 等待首次位置数据
            if (current_x_ != 0 || current_y_ != 0 || current_z_ != 0) {
                is_initialized_ = true;
                RCLCPP_INFO(this->get_logger(), "Control loop initialized");
            }
            return;
        }

        calculate_motor_speeds();
    }


    void calculate_motor_speeds() {
        // 1. 位置环PID计算期望速度
        float err_pos_x = desired_x_ - current_x_;
        float err_pos_y = desired_y_ - current_y_;
        float err_pos_z = desired_z_ - current_z_;

        // 积分项限幅
        integral_pos_x_ = clamp(integral_pos_x_ + err_pos_x * 0.01, -5.0, 5.0);
        integral_pos_y_ = clamp(integral_pos_y_ + err_pos_y * 0.01, -5.0, 5.0);
        integral_pos_z_ = clamp(integral_pos_z_ + err_pos_z * 0.01, -5.0, 5.0);

        // 微分项
        float der_pos_x = (err_pos_x - last_err_pos_x_) / 0.01;
        float der_pos_y = (err_pos_y - last_err_pos_y_) / 0.01;
        float der_pos_z = (err_pos_z - last_err_pos_z_) / 0.01;

        // 位置环输出（期望速度）
        float desired_vx = Kp_pos * err_pos_x + Ki_pos * integral_pos_x_ + Kd_pos * der_pos_x;
        float desired_vy = Kp_pos * err_pos_y + Ki_pos * integral_pos_y_ + Kd_pos * der_pos_y;
        float desired_vz = Kp_pos * err_pos_z + Ki_pos * integral_pos_z_ + Kd_pos * der_pos_z;

        
        last_err_pos_x_ = err_pos_x;
        last_err_pos_y_ = err_pos_y;
        last_err_pos_z_ = err_pos_z;

        // 2. 速度环PID计算力和力矩
        float err_vel_x = desired_vx - current_vx_;
        float err_vel_y = desired_vy - current_vy_;
        float err_vel_z = desired_vz - current_vz_;

        
        integral_vel_x_ = clamp(integral_vel_x_ + err_vel_x * 0.01, -3.0, 3.0);
        integral_vel_y_ = clamp(integral_vel_y_ + err_vel_y * 0.01, -3.0, 3.0);
        integral_vel_z_ = clamp(integral_vel_z_ + err_vel_z * 0.01, -3.0, 3.0);

        f
        loat der_vel_x = (err_vel_x - last_err_vel_x_) / 0.01;
        float der_vel_y = (err_vel_y - last_err_vel_y_) / 0.01;
        float der_vel_z = (err_vel_z - last_err_vel_z_) / 0.01;

        // 计算力和力矩
        float F = m * (g + Kp_vel * err_vel_z + Ki_vel * integral_vel_z_ + Kd_vel * der_vel_z);
        float Mx = Kp_vel * err_vel_x + Ki_vel * integral_vel_x_ + Kd_vel * der_vel_x;
        float My = Kp_vel * err_vel_y + Ki_vel * integral_vel_y_ + Kd_vel * der_vel_y;
        float Mz = 0.0;  // 简化处理偏航

        last_err_vel_x_ = err_vel_x;
        last_err_vel_y_ = err_vel_y;
        last_err_vel_z_ = err_vel_z;

        // 3. 解算螺旋桨转速
        float u1 = (F/(4*k) + Mx/(2*l*k) + My/(2*l*k) + Mz/(4*b));
        float u2 = (F/(4*k) - Mx/(2*l*k) - My/(2*l*k) - Mz/(4*b));
        float u3 = (F/(4*k) - Mx/(2*l*k) + My/(2*l*k) + Mz/(4*b));
        float u4 = (F/(4*k) + Mx/(2*l*k) - My/(2*l*k) - Mz/(4*b));

        // 计算转速并限制范围
        float n1 = sqrt(fabs(u1));
        float n2 = sqrt(fabs(u2));
        float n3 = sqrt(fabs(u3));
        float n4 = sqrt(fabs(u4));

        // 4. 发布到AirSim（使用mavros接口）
        mavros_msgs::msg::ActuatorControl msg;
        msg.header.stamp = this->now();
        msg.group_mix = 0;  // 电机控制组
        msg.controls[0] = n1 / 10000.0;  // 归一化到[0,1]范围
        msg.controls[1] = n2 / 10000.0;
        msg.controls[2] = n3 / 10000.0;
        msg.controls[3] = n4 / 10000.0;
        motor_pub_->publish(msg);
    }

    
    float clamp(float value, float min, float max) {
        return std::max(min, std::min(value, max));
    }

    // 订阅者和发布者
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr traj_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<mavros_msgs::msg::ActuatorControl>::SharedPtr motor_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // 状态变量
    float desired_x_ = 0.0, desired_y_ = 0.0, desired_z_ = 0.0;
    float current_x_ = 0.0, current_y_ = 0.0, current_z_ = 0.0;
    float current_vx_ = 0.0, current_vy_ = 0.0, current_vz_ = 0.0;
    bool is_initialized_ = false;

    // PID参数
    float integral_pos_x_, integral_pos_y_, integral_pos_z_;
    float integral_vel_x_, integral_vel_y_, integral_vel_z_;
    float last_err_pos_x_, last_err_pos_y_, last_err_pos_z_;
    float last_err_vel_x_, last_err_vel_y_, last_err_vel_z_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryConverterNode>());
    rclcpp::shutdown();
    return 0;

}
