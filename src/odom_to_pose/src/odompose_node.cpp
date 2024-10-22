//-----------------------------------------------------------------------------------------------//
// 【プログラムの流れ】
//  "nav_msgs::msg::Odometry"型のメッセージを受け取り、
//  その中に含まれている位置と姿勢のデータを"geometry_msgs::msg::PoseStamped"型に変換
//
// (1) サブスクライブ
// 	・オドメトリデータ (nav_msgs::msg::Odometry) を受信
//  ・位置 (position.x, position.y)、姿勢（四元数）の情報を取得
//
// (2) 四元数からヨー角の計算
//	：四元数をロール、ピッチ、ヨー角に変換し、ヨー角を抽出
//
// (3) パブリッシュ
//	・geometry_msgs::msg::PoseStamped メッセージを生成し、/odom_poseトピックに送信
//
//-----------------------------------------------------------------------------------------------//

#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Matrix3x3.h"

using namespace std::chrono_literals;

class OdomPoseNode : public rclcpp::Node {
public:
    OdomPoseNode() : Node("odompose_node") {
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&OdomPoseNode::odom_callback, this, std::placeholders::_1));

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("odom_pose", 10);
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        
        double position_x = msg->pose.pose.position.x;
        double position_y = msg->pose.pose.position.y;

        double orientation_x = msg->pose.pose.orientation.x;
        double orientation_y = msg->pose.pose.orientation.y;
        double orientation_z = msg->pose.pose.orientation.z;
        double orientation_w = msg->pose.pose.orientation.w;

        tf2::Quaternion q(orientation_x, orientation_y, orientation_z, orientation_w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = "odom"; 
        pose_msg.pose.position.x = position_x;
        pose_msg.pose.position.y = position_y;
        pose_msg.pose.position.z = 0.0;
        pose_msg.pose.orientation = msg->pose.pose.orientation;

        pose_pub_->publish(pose_msg);

        RCLCPP_INFO(this->get_logger(), "Position - x: %.2f, y: %.2f, yaw: %.2f", position_x, position_y, yaw);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomPoseNode>());
    rclcpp::shutdown();
    return 0;
}