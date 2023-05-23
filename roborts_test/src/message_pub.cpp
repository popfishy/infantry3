#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <algorithm>
#include <cstdint>
#include <vector>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "auto_aim_interfaces/msg/target.hpp"
#include "auto_aim_interfaces/msg/game_status.hpp"

using namespace std::chrono_literals;
const int RED = 0;
const int BLUE = 1;

class MessagePub : public rclcpp::Node
{

public:
    MessagePub() : Node("aimdata_msg_pub")
    {
        RCLCPP_INFO(this->get_logger(), "Start aimdata_msg pub !");
        // aimdata_pub_ = this->create_publisher<auto_aim_interfaces::msg::AimData>("/aim_data", rclcpp::SensorDataQoS());
        target_pub_ =
            this->create_publisher<auto_aim_interfaces::msg::Target>("/processor/target", rclcpp::SensorDataQoS());
        twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        gamestatus_pub_ = this->create_publisher<auto_aim_interfaces::msg::GameStatus>(
            "/game_status", rclcpp::QoS(rclcpp::KeepLast(1)));
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/joint_states", rclcpp::QoS(rclcpp::KeepLast(1)));
        timer_ = this->create_wall_timer(5ms, std::bind(&MessagePub::timer_callback, this));
    }

private:
    void timer_callback()
    {
        joint_state.header.stamp = this->now();
        joint_state.name.push_back("pitch_joint");
        joint_state.name.push_back("yaw_joint");
        joint_state.position.push_back(0);
        joint_state.position.push_back(0);
        // target_msg_.header.stamp = joint_state.header.stamp = this->now();
        // target_msg_.suggest_fire = 1;
        // target_msg_.task_mode = 3;
        // target_msg_.tracking = 1;
        // target_msg_.position.x = 1;
        // target_msg_.position.y = 2;
        // target_msg_.position.z = 3;
        // target_msg_.yaw = 3.24;
        // target_msg_.velocity.x = 0.2;
        // target_msg_.velocity.y = 0.2;
        // target_msg_.velocity.z = 0.3;
        // target_msg_.v_yaw = 0.02;
        // target_msg_.pitch = -1.3;
        // target_msg_.radius_1 = 0.3;
        // target_msg_.radius_2 = 0.2;
        // target_msg_.z_2 = 0.25;

        twist_msg_.linear.x = 0.1;
        twist_msg_.linear.y = 0;
        twist_msg_.angular.z = 0.1;
        gamestatus_msg_.enemy_color = 1;
        gamestatus_msg_.game_status = 4;
        gamestatus_msg_.outpost_status = 1;

        // std::cout << "uint8_t game_status: " << int(gamestatus_msg_.game_status) << std::endl;

        // std::copy(
        //     reinterpret_cast<const uint8_t *>(&packet),
        //     reinterpret_cast<const uint8_t *>(&packet) + sizeof(SendPacket), can_frame_msg_.data.begin());
        // aimdata_pub_->publish(aimdata_msg_);
        // target_pub_->publish(target_msg_);
        twist_pub_->publish(twist_msg_);
        gamestatus_pub_->publish(gamestatus_msg_);
        joint_state_pub_->publish(joint_state);
    }

private:
    auto_aim_interfaces::msg::Target target_msg_;
    geometry_msgs::msg::Twist twist_msg_;
    auto_aim_interfaces::msg::GameStatus gamestatus_msg_;
    sensor_msgs::msg::JointState joint_state;

    rclcpp::TimerBase::SharedPtr timer_;
    int count = 0;
    rclcpp::Publisher<auto_aim_interfaces::msg::Target>::SharedPtr target_pub_;
    rclcpp::Publisher<auto_aim_interfaces::msg::GameStatus>::SharedPtr gamestatus_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MessagePub>());
    rclcpp::shutdown();
    return 0;
}
