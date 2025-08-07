#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "mavros_msgs/msg/state.hpp"

const float k = 0.01;     // 升力系数
const float l = 0.15;     // 螺旋桨到机体中心的距离(m)
const float b = 0.001;    // 阻力系数
const float m = 1.2;      // 四旋翼质量(kg)
const float g = 9.8;      // 重力加速度(m/s²)

// 串级PID参数
// 外环（位置环）参数
const float Kp_pos = 8.0, Ki_pos = 0.05, Kd_pos = 1.2;
// 内环（速度环）参数
const float Kp_vel = 4.0, Ki_vel = 0.02, Kd_vel = 0.8;

class TrajectoryConverterNode : public rclcpp::Node {
public:
    TrajectoryConverterNode() : Node("trajectory_converter_node") {
        // 订阅期望轨迹（世界坐标系下的位置）
        traj_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/desired_trajectory", 10,
            std::bind(&TrajectoryConverterNode::trajectory_callback, this, std::placeholders::_1));
        
        // 订阅无人机当前状态（位置、速度、姿态，通过mavros获取）
        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "/mavros/state", 10,
            std::bind(&TrajectoryConverterNode::state_callback, this, std::placeholders::_1));
        
        // 发布螺旋桨转速（4个螺旋桨的目标转速）
        motor_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/motor_speeds", 10);

        // 初始化PID积分项
        integral_pos_x_ = 0.0;
        integral_pos_y_ = 0.0;
        integral_pos_z_ = 0.0;
        integral_vel_x_ = 0.0;
        integral_vel_y_ = 0.0;
        integral_vel_z_ = 0.0;

        // 记录上一时刻误差（用于微分计算）
        last_err_pos_x_ = 0.0;
        last_err_pos_y_ = 0.0;
        last_err_pos_z_ = 0.0;
        last_err_vel_x_ = 0.0;
        last_err_vel_y_ = 0.0;
        last_err_vel_z_ = 0.0;
    }

private:
    // 回调函数：处理期望轨迹
    void trajectory_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        desired_x_ = msg->pose.position.x;
        desired_y_ = msg->pose.position.y;
        desired_z_ = msg->pose.position.z;

        // 计算螺旋桨转速并发布
        calculate_motor_speeds();
    }

    // 回调函数：更新无人机当前状态（简化示例，实际需获取位置、速度、姿态）
    void state_callback(const mavros_msgs::msg::State::SharedPtr msg) {
        // 实际应用中需从传感器获取当前位置（current_x_, current_y_, current_z_）
        // 和当前速度（current_vx_, current_vy_, current_vz_）
        current_state_ = *msg;
    }

    // 核心函数：计算螺旋桨转速
    void calculate_motor_speeds() {
        // 1. 外环（位置环PID）：计算期望速度（参考《串级 PID 控制原理及应用.doc》伪代码）
        float err_pos_x = desired_x_ - current_x_;
        float err_pos_y = desired_y_ - current_y_;
        float err_pos_z = desired_z_ - current_z_;

        // 位置环积分项（带限幅，防止积分饱和）
        integral_pos_x_ = clamp(integral_pos_x_ + err_pos_x * 0.01, -5.0, 5.0);
        integral_pos_y_ = clamp(integral_pos_y_ + err_pos_y * 0.01, -5.0, 5.0);
        integral_pos_z_ = clamp(integral_pos_z_ + err_pos_z * 0.01, -5.0, 5.0);

        // 位置环微分项
        float der_pos_x = (err_pos_x - last_err_pos_x_) / 0.01;
        float der_pos_y = (err_pos_y - last_err_pos_y_) / 0.01;
        float der_pos_z = (err_pos_z - last_err_pos_z_) / 0.01;

        // 位置环输出（期望速度）
        float desired_vx = Kp_pos * err_pos_x + Ki_pos * integral_pos_x_ + Kd_pos * der_pos_x;
        float desired_vy = Kp_pos * err_pos_y + Ki_pos * integral_pos_y_ + Kd_pos * der_pos_y;
        float desired_vz = Kp_pos * err_pos_z + Ki_pos * integral_pos_z_ + Kd_pos * der_pos_z;

        // 更新上一时刻位置误差
        last_err_pos_x_ = err_pos_x;
        last_err_pos_y_ = err_pos_y;
        last_err_pos_z_ = err_pos_z;

        // 2. 内环（速度环PID）：计算所需力和力矩
        float err_vel_x = desired_vx - current_vx_;
        float err_vel_y = desired_vy - current_vy_;
        float err_vel_z = desired_vz - current_vz_;

        // 速度环积分项
        integral_vel_x_ = clamp(integral_vel_x_ + err_vel_x * 0.01, -3.0, 3.0);
        integral_vel_y_ = clamp(integral_vel_y_ + err_vel_y * 0.01, -3.0, 3.0);
        integral_vel_z_ = clamp(integral_vel_z_ + err_vel_z * 0.01, -3.0, 3.0);

        // 速度环微分项
        float der_vel_x = (err_vel_x - last_err_vel_x_) / 0.01;
        float der_vel_y = (err_vel_y - last_err_vel_y_) / 0.01;
        float der_vel_z = (err_vel_z - last_err_vel_z_) / 0.01;

        // 速度环输出（力和力矩）
        float F = m * (g + Kp_vel * err_vel_z + Ki_vel * integral_vel_z_ + Kd_vel * der_vel_z);  // 总升力
        float Mx = Kp_vel * err_vel_x + Ki_vel * integral_vel_x_ + Kd_vel * der_vel_x;          // 滚转力矩
        float My = Kp_vel * err_vel_y + Ki_vel * integral_vel_y_ + Kd_vel * der_vel_y;          // 俯仰力矩
        float Mz = 0.0;  // 偏航力矩（简化处理，实际需根据航向误差计算）

        // 更新上一时刻速度误差
        last_err_vel_x_ = err_vel_x;
        last_err_vel_y_ = err_vel_y;
        last_err_vel_z_ = err_vel_z;
        // 3. 解算螺旋桨转速
        float u1 = (F/(4*k) + Mx/(2*l*k) + My/(2*l*k) + Mz/(4*b));  // n1²
        float u2 = (F/(4*k) - Mx/(2*l*k) - My/(2*l*k) - Mz/(4*b));  // n2²
        float u3 = (F/(4*k) - Mx/(2*l*k) + My/(2*l*k) + Mz/(4*b));  // n3²
        float u4 = (F/(4*k) + Mx/(2*l*k) - My/(2*l*k) - Mz/(4*b));  // n4²
        // 计算转速（确保非负，物理上可行）
        float n1 = sqrt(fabs(u1));
        float n2 = sqrt(fabs(u2));
        float n3 = sqrt(fabs(u3));
        float n4 = sqrt(fabs(u4));
        // 发布转速
        std_msgs::msg::Float32MultiArray speeds_msg;
        speeds_msg.data = {n1, n2, n3, n4};
        motor_pub_->publish(speeds_msg);
    }

    // 辅助函数：限制积分项范围
    float clamp(float value, float min, float max) {
        if (value < min) return min;
        if (value > max) return max;
        return value;
    }

    // 订阅者和发布者
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr traj_sub_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr motor_pub_;

    // 轨迹与状态变量
    float desired_x_, desired_y_, desired_z_;  // 期望位置
    float current_x_ = 0.0, current_y_ = 0.0, current_z_ = 0.0;  // 当前位置（实际应从传感器获取）
    float current_vx_ = 0.0, current_vy_ = 0.0, current_vz_ = 0.0;  // 当前速度
    mavros_msgs::msg::State current_state_;

    // PID积分项与误差记录
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


