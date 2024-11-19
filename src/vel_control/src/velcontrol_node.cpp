#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <iostream>
#include <algorithm>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class VelControlNode : public rclcpp::Node {
public:
    VelControlNode()
        : Node("velcontrol_node"),
          target_x_(0.0),
          target_y_(0.0),
          target_theta_(0.0),
          distance_threshold_(2.0),              // 到達判定のための距離閾値[m] 
          angle_threshold_(10.0 * M_PI / 180.0), // 到達判定のための角度閾値[rad]
          scale_factor_(1.0 / 10.0),             // 目標位置付近での減速スケールファクタ
          target_set_(false),
          last_cmd_vel_time_(this->now())
    {
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "odom_pose", 10, std::bind(&VelControlNode::pose_callback, this, std::placeholders::_1));

        target_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "target_point", 10, std::bind(&VelControlNode::target_callback, this, std::placeholders::_1));

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    }

private:
    // メンバ変数
    double target_x_;
    double target_y_;
    double target_theta_;
    double distance_threshold_;
    double angle_threshold_;
    double scale_factor_;
    bool target_set_;
    rclcpp::Time last_cmd_vel_time_;                  // 最後にcmd_velが発信された時刻
    const std::chrono::seconds timeout_threshold_{2}; // cmd_velが2s間更新されない場合に停止

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    void target_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() == 3) {
            target_x_ = msg->data[0];
            target_y_ = msg->data[1];
            target_theta_ = msg->data[2];
            target_set_ = true;
            RCLCPP_INFO(this->get_logger(), "Received target point: (%.2f, %.2f, theta: %.2f)", target_x_, target_y_, target_theta_);
        } else {
            RCLCPP_WARN(this->get_logger(), "Invalid target point message received.");
        }
    }

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        geometry_msgs::msg::Twist cmd_vel_msg;

        // 現在の時刻を取得
        rclcpp::Time current_time = this->now();

        // cmd_velの発信から一定時間が経過している場合、停止処理
        if ((current_time - last_cmd_vel_time_).seconds() > timeout_threshold_.count()) {
            stop_robot(cmd_vel_msg);
            RCLCPP_WARN(this->get_logger(), "cmd_vel has been fixed for too long, stopping robot.");
        } else {
            // 目標が設定されていない場合
            if (!target_set_) {
                stop_robot(cmd_vel_msg); //【停止】
                return;
            } else {
                // 現在位置・姿勢(x, y, yaw)
                double robot_x = msg->pose.position.x;
                double robot_y = msg->pose.position.y;
                double robot_yaw = std::atan2(2.0 * (msg->pose.orientation.w * msg->pose.orientation.z + msg->pose.orientation.x * msg->pose.orientation.y),
                                            1.0 - 2.0 * (msg->pose.orientation.y * msg->pose.orientation.y + msg->pose.orientation.z * msg->pose.orientation.z));

                // 目標位置方向への角度
                double angle_to_target = std::atan2(target_y_ - robot_y, target_x_ - robot_x);

                // 目標位置までの距離
                double distance = std::hypot(target_x_ - robot_x, target_y_ - robot_y);

                // 目標姿勢までの角度差
                double anglediff = target_theta_ - robot_yaw;
                // [-π, π]の範囲に正規化
                while (anglediff > M_PI) anglediff -= 2.0 * M_PI;
                while (anglediff < -M_PI) anglediff += 2.0 * M_PI;

                // 到達判定・速度司令
                if (distance < distance_threshold_ && std::abs(anglediff) < angle_threshold_) {
                    stop_robot(cmd_vel_msg);
                    RCLCPP_INFO(this->get_logger(), "Reached target point and orientation.");
                } else {
                    if (distance < distance_threshold_ && std::abs(anglediff) >= angle_threshold_) {
                        rotate_towards_target(cmd_vel_msg, distance, anglediff);
                    } else if (std::abs(anglediff) < angle_threshold_ && distance >= distance_threshold_) {
                        move_towards_target(angle_to_target, cmd_vel_msg, distance, anglediff, robot_yaw);
                    } else {
                        move_and_rotate_towards_target(angle_to_target, cmd_vel_msg, distance, anglediff, robot_yaw);
                    }
                }
            }
        }

        cmd_vel_pub_->publish(cmd_vel_msg);
        RCLCPP_INFO(this->get_logger(), "cmd_vel: linear.x=%.2f, linear.y=%.2f, angular.z=%.2f",
                    cmd_vel_msg.linear.x, cmd_vel_msg.linear.y, cmd_vel_msg.angular.z);

        // 最後のcmd_vel発信時刻を更新
        last_cmd_vel_time_ = current_time;
    }

    // 停止処理
    void stop_robot(geometry_msgs::msg::Twist &cmd_vel_msg) {
        cmd_vel_msg.linear.x = 0.0;
        cmd_vel_msg.linear.y = 0.0;
        cmd_vel_msg.angular.z = 0.0;
        RCLCPP_INFO(this->get_logger(), "Waiting for target point...");
    }

    // 目標に向かって回転
    void rotate_towards_target(geometry_msgs::msg::Twist &cmd_vel_msg, double distance, double anglediff) {
        cmd_vel_msg.linear.x = 0.0;
        cmd_vel_msg.linear.y = 0.0;
        double A = 1.0;
        cmd_vel_msg.angular.z = std::clamp(A * anglediff, -1.0, 1.0); // 角速度（最大角速度を制限）
        RCLCPP_INFO(this->get_logger(), "Rotating towards target (distance: %.2f, angle: %.2f)", distance, anglediff);
    }

    // 目標に向かって進む
    void move_towards_target(double angle_to_target, geometry_msgs::msg::Twist &cmd_vel_msg, double distance, double anglediff, double robot_yaw) {
        double V = std::min(std::sqrt(scale_factor_ * distance), 1.0); // 最大速度を制限
        cmd_vel_msg.linear.x = V * cos(angle_to_target - robot_yaw);   // X方向速度
        cmd_vel_msg.linear.y = V * sin(angle_to_target - robot_yaw);   // Y方向速度
        cmd_vel_msg.angular.z = 0;
        RCLCPP_INFO(this->get_logger(), "Moving towards target (distance: %.2f, angle: %.2f)", distance, anglediff);
    }

    // 進みながら回転
    void move_and_rotate_towards_target(double angle_to_target, geometry_msgs::msg::Twist &cmd_vel_msg, double distance, double anglediff, double robot_yaw) {
        double V = std::min(std::sqrt(scale_factor_ * distance), 1.0); // 最大速度を制限
        cmd_vel_msg.linear.x = V * cos(angle_to_target - robot_yaw);   // X方向速度
        cmd_vel_msg.linear.y = V * sin(angle_to_target - robot_yaw);   // Y方向速度
        double A = 1.0;
        cmd_vel_msg.angular.z = std::clamp(A * anglediff, -1.0, 1.0); // 角速度（最大角速度を制限）
        RCLCPP_INFO(this->get_logger(), "Moving towards target (distance: %.2f, angle: %.2f)", distance, anglediff);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VelControlNode>());
    rclcpp::shutdown();
    return 0;
}