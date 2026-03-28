#include <chrono>
#include <cmath>
#include <limits>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <std_msgs/msg/int32.hpp>
// 【新增核心 1】：引入标准三维点坐标消息，用于接收 Python 发来的 XYZ
#include <geometry_msgs/msg/point.hpp> 

using namespace std::chrono_literals;

class CircleFlightNode : public rclcpp::Node {
public:
    CircleFlightNode() : Node("circle_flight_node"), flight_time_(0.0), current_mode_(0) {
        std::string ns = this->get_namespace();
        drone_id_ = 1; 
        if (ns.find("px4_") != std::string::npos) {
            drone_id_ = std::stoi(ns.substr(ns.find("px4_") + 4));
        }

        rclcpp::QoS qos_best_effort = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().transient_local();
        rclcpp::QoS qos_reliable = rclcpp::QoS(rclcpp::KeepLast(5)).reliable().transient_local();

        offboard_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("fmu/in/offboard_control_mode", qos_best_effort);
        traj_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("fmu/in/trajectory_setpoint", qos_best_effort);
        cmd_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("fmu/in/vehicle_command", qos_reliable);
        
        // 监听全局模式切换频道
        cmd_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/swarm_command", 10, std::bind(&CircleFlightNode::command_callback, this, std::placeholders::_1));

        // 【新增核心 2】：AI 轨迹数据接收通道！监听相对路径 "ai_trajectory"
        ai_traj_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "ai_trajectory", 10, std::bind(&CircleFlightNode::ai_trajectory_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(100ms, std::bind(&CircleFlightNode::timer_callback, this));
        
        current_x_ = 0.0; current_y_ = 0.0; current_z_ = -8.0;
        // 初始化 AI 默认给定的起始坐标
        ai_target_x_ = 0.0; ai_target_y_ = 0.0; ai_target_z_ = -8.0;

        RCLCPP_INFO(this->get_logger(), ">>> 交互式与 AI 追踪节点已启动！本机 ID: %d <<<", drone_id_);
    }

private:
    void command_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        if (current_mode_ != msg->data) {
            current_mode_ = msg->data;
            RCLCPP_INFO(this->get_logger(), "【变阵指令】本机 ID %d 准备切换至模式: %d", drone_id_, current_mode_);
        }
    }

    // 【新增核心 3】：一听到 Python 发来的 AI 预测坐标，就立刻存进大脑里
    void ai_trajectory_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
        ai_target_x_ = msg->x;
        ai_target_y_ = msg->y;
        ai_target_z_ = msg->z;
    }

    void timer_callback() {
        uint64_t timestamp = this->get_clock()->now().nanoseconds() / 1000;

        px4_msgs::msg::OffboardControlMode offboard_msg;
        offboard_msg.position = true;
        offboard_msg.timestamp = timestamp;
        offboard_pub_->publish(offboard_msg);

        float nan = std::numeric_limits<float>::quiet_NaN();
        float target_x = 0.0, target_y = 0.0, target_z = -8.0; 

        if (flight_time_ > 5.0) {
            if (current_mode_ == 1) { 
                float radius = 4.0;
                float angular_speed = 0.5;
                float t = flight_time_ - 5.0;
                float phase_offset = (drone_id_ - 1) * (2.0 * M_PI / 3.0); 
                target_x = radius * std::cos(angular_speed * t + phase_offset);
                target_y = radius * std::sin(angular_speed * t + phase_offset);
            } 
            else if (current_mode_ == 2) { 
                if (drone_id_ == 1) { target_x = 0.0; target_y = 0.0; }
                else if (drone_id_ % 2 == 0) { target_x = -2.0 * (drone_id_/2); target_y = 2.0 * (drone_id_/2); }
                else { target_x = -2.0 * (drone_id_/2); target_y = -2.0 * (drone_id_/2); }
            }
            else if (current_mode_ == 3) { 
                target_x = 0.0;
                if (drone_id_ == 1) { target_y = 0.0; }
                else if (drone_id_ % 2 == 0) { target_y = 2.0 * (drone_id_/2); }
                else { target_y = -2.0 * (drone_id_/2); }
            }
            // 【新增核心 4】：模式 4 彻底激活！直接把 AI 传来的坐标当作自己的目标
            else if (current_mode_ == 4) {
                target_x = ai_target_x_;
                target_y = ai_target_y_;
                target_z = ai_target_z_;
            }
        }

        // 依然保留极其平滑的一阶低通滤波
        float alpha = 0.05; 
        current_x_ += (target_x - current_x_) * alpha;
        current_y_ += (target_y - current_y_) * alpha;
        current_z_ += (target_z - current_z_) * alpha;

        px4_msgs::msg::TrajectorySetpoint traj_msg;
        traj_msg.position = {current_x_, current_y_, current_z_};
        traj_msg.yaw = 0.0;
        traj_msg.velocity = {nan, nan, nan};
        traj_msg.acceleration = {nan, nan, nan};
        traj_msg.jerk = {nan, nan, nan};
        traj_msg.timestamp = timestamp;
        traj_pub_->publish(traj_msg);

        if (std::abs(flight_time_ - 2.0) < 0.05) {
            this->send_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0, 0);
            this->send_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 21196.0, 0);
        }

        flight_time_ += 0.1;
    }

    void send_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0, uint8_t target_sys = 0) {
        px4_msgs::msg::VehicleCommand cmd_msg;
        cmd_msg.command = command;
        cmd_msg.param1 = param1;
        cmd_msg.param2 = param2;
        cmd_msg.target_system = target_sys;
        cmd_msg.target_component = 1;
        cmd_msg.source_system = 255;
        cmd_msg.source_component = 1;
        cmd_msg.from_external = true;
        cmd_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        cmd_pub_->publish(cmd_msg);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr traj_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr cmd_pub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr cmd_sub_;
    
    // AI 数据的专属接收器和存储变量
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr ai_traj_sub_;
    float ai_target_x_, ai_target_y_, ai_target_z_;
    
    double flight_time_;
    int current_mode_; 
    int drone_id_;
    float current_x_, current_y_, current_z_; 
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CircleFlightNode>());
    rclcpp::shutdown();
    return 0;
}
