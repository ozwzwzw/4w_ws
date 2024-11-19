/*
// 半径2メートルの円周上をぐるぐる ///////////////////////////////////////////////////////////////////////////////////////

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cmath>

class OdomPoseNode : public rclcpp::Node {
public:
    OdomPoseNode() : Node("odompose_node"), angle_(0.0) {
        // タイマー (0.1sごとにコールバックを呼び出す)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&OdomPoseNode::timer_callback, this)
        );

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("odom_pose", 10);
    }

private:
    void timer_callback() {
        // 半径2メートルの円を描くための位置計算
        double radius = 2.0;
        double angular_speed = 0.1; // 角速度[rad/s]

        angle_ += angular_speed * 0.1; // タイマー周期（0.1s）に合わせて角度を更新

        // 円の上の位置を計算
        double position_x = radius * cos(angle_);
        double position_y = radius * sin(angle_);

        // オリエンテーション（yaw）を計算（Z軸の回転）
        double orientation_z = sin(angle_ / 2.0);
        double orientation_w = cos(angle_ / 2.0);

        // Yaw角を計算
        double yaw = angle_;

        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = "odom"; 
        pose_msg.pose.position.x = position_x; // X位置
        pose_msg.pose.position.y = position_y; // Y位置
        pose_msg.pose.position.z = 0.0;        // Z位置
        pose_msg.pose.orientation.x = 0.0;     // Xオリエンテーション
        pose_msg.pose.orientation.y = 0.0;     // Yオリエンテーション
        pose_msg.pose.orientation.z = orientation_z; // Zオリエンテーション
        pose_msg.pose.orientation.w = orientation_w; // Wオリエンテーション（クォータニオン）

        pose_pub_->publish(pose_msg);
        RCLCPP_INFO(this->get_logger(), "Publishing odom_pose: (%.2f, %.2f, %.2f)", position_x, position_y, yaw);
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    double angle_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomPoseNode>());
    rclcpp::shutdown();
    return 0;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
*/



// 現在位置・姿勢 = 0.0 に固定 ////////////////////////////////////////////////////////////////////////////////////////////////

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class OdomPoseNode : public rclcpp::Node {
public:
    OdomPoseNode() : Node("odompose_node") {
        // タイマー (0.1sごとにコールバックを呼び出す)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&OdomPoseNode::timer_callback, this)
        );

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("odom_pose", 10);
    }

private:
    void timer_callback() {
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = "odom"; 
        pose_msg.pose.position.x = 0.0; // X位置
        pose_msg.pose.position.y = 0.0; // Y位置
        pose_msg.pose.position.z = 0.0; // Z位置
        pose_msg.pose.orientation.x = 0.0; // Xオリエンテーション
        pose_msg.pose.orientation.y = 0.0; // Yオリエンテーション
        pose_msg.pose.orientation.z = 0.0; // Zオリエンテーション
        pose_msg.pose.orientation.w = 1.0; // Wオリエンテーション（クォータニオン）

        pose_pub_->publish(pose_msg);
        RCLCPP_INFO(this->get_logger(), "Publishing odom_pose: (0.0, 0.0, 0.0)");
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomPoseNode>());
    rclcpp::shutdown();
    return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
