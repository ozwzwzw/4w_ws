#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"
#include <map>
#include <iostream>
#include <cmath>

class TargetNode : public rclcpp::Node {
public:
    TargetNode() : Node("target_node"), target_set_(false) {
        // 目標地点・姿勢 (x, y, theta) を定義
        target_points_[1] = {5.0, 0.0, 0.0};
        target_points_[2] = {5.0, 5.0, M_PI/2};
        target_points_[3] = {-5.0, 5.0, M_PI};
        target_points_[4] = {-5.0, 0.0, -M_PI/2};
        target_points_[5] = {-5.0, -5.0, 0.0};
        target_points_[6] = {5.0, -5.0, M_PI/2};
        target_points_[7] = {0.0, 0.0, 0.0};

        target_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("target_point", 10);

        input_thread_ = std::thread(&TargetNode::input_loop, this);
    }

    ~TargetNode() {
        if (input_thread_.joinable()) {
            input_thread_.join();
        }
    }

private:
    void input_loop() {
        while (rclcpp::ok()) {
            int target_number;
            std::cout << "Enter target point (1-7): ";
            std::cin >> target_number;

            if (target_points_.find(target_number) != target_points_.end()) {
                auto target = target_points_[target_number];
                RCLCPP_INFO(this->get_logger(), "Moving to target point %d: (%.2f, %.2f, %.2f)", target_number, target[0], target[1], target[2]);

                std_msgs::msg::Float64MultiArray target_msg;
                target_msg.data = {target[0], target[1], target[2]};

                target_pub_->publish(target_msg);
                target_set_ = true;
            } else {
                std::cout << "Invalid target number. Please enter a number between 1 and 7." << std::endl;
            }
        }
    }

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr target_pub_;
    std::map<int, std::array<double, 3>> target_points_;
    std::thread input_thread_;
    bool target_set_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TargetNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
