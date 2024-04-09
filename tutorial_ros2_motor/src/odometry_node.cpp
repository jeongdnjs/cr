#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp" // 추가: 오도메트리 메시지를 위한 헤더
#include "tf2_ros/transform_broadcaster.h" // 추가: TF 변환을 위한 헤더
#include "geometry_msgs/msg/transform_stamped.h" // 추가: 변환 메시지를 위한 헤더
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <cmath>
#include <string>
#include <tutorial_ros2_motor/motor_node.hpp>


class OdometryNode : public rclcpp::Node {
public:
    OdometryNode()
        : Node("odometry_node"),
          wheel_radius(0.0575), // 바퀴 반지름 (미터 단위)
          wheel_distance(0.4104), // 바퀴 사이의 거리 (미터 단위)
          encoder_resolution(228), // 엔코더 해상도
          x_position(0.0), y_position(0.0), theta(0.0),
          last_encoder_count_1A(0), last_encoder_count_1B(0),
          last_encoder_count_2A(0), last_encoder_count_2B(0) {
        encoder_info_subscription_ = this->create_subscription<std_msgs::msg::Int64MultiArray>(
            "EncoderInfo", 10,
            std::bind(&OdometryNode::encoder_info_callback, this, std::placeholders::_1));
            odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 50);
            // 추가: TF 변환 발행을 위한 브로드캐스터 생성
            tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

private:
    void encoder_info_callback(const std_msgs::msg::Int64MultiArray::SharedPtr msg) {
        int encoder_count_1A = msg->data[0];
        int encoder_count_1B = msg->data[1];
        int encoder_count_2A = msg->data[2];
        int encoder_count_2B = msg->data[3];

        if (encoder_count_1A != last_encoder_count_1A ||
            encoder_count_1B != last_encoder_count_1B ||
            encoder_count_2A != last_encoder_count_2A ||
            encoder_count_2B != last_encoder_count_2B) {

            double distance_left = (encoder_count_1A + encoder_count_1B - last_encoder_count_1A - last_encoder_count_1B) / 2.0 * (2 * M_PI * wheel_radius) / encoder_resolution;
            double distance_right = -(encoder_count_2A + encoder_count_2B - last_encoder_count_2A - last_encoder_count_2B) / 2.0 * (2 * M_PI * wheel_radius) / encoder_resolution;

            update_position(distance_left, distance_right);
            publish_odometry(distance_left, distance_right); // odometry 메시지 및 변환 발행 

            RCLCPP_INFO(this->get_logger(), "left: %f, right: %f, Position: (%f cm, %f cm), Orientation: %f degrees", distance_left, distance_right, x_position * 100, y_position * 100, theta * (180.0 / M_PI));
        }

        last_encoder_count_1A = encoder_count_1A;
        last_encoder_count_1B = encoder_count_1B;
        last_encoder_count_2A = encoder_count_2A;
        last_encoder_count_2B = encoder_count_2B;
    }

    void update_position(double distance_left, double distance_right) {
        double delta_distance = (distance_right + distance_left) / 2.0;
        double delta_theta = (distance_right - distance_left) / wheel_distance;

        theta += delta_theta;
        theta = wrap_angle(theta);

        x_position += delta_distance * cos(theta); // theta->delta_theta?
        y_position += delta_distance * sin(theta);
    }

    double wrap_angle(double theta) {
        return fmod(theta + M_PI, 2 * M_PI) - M_PI;
    }

    void publish_odometry(double distance_left, double distance_right) {
        auto now = this->get_clock()->now();
        // 오도메트리 메시지 구성 및 발행
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = now;
        odom.header.frame_id = "odom";
        odom.pose.pose.position.x = x_position;
        odom.pose.pose.position.y = y_position;
        odom.pose.pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, sin(theta / 2), cos(theta / 2)));
        odom.child_frame_id = "base_link";
        odometry_publisher_->publish(odom);

        // TF 변환 구성 및 발행
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = now;
        transform.header.frame_id = "odom";
        transform.child_frame_id = "base_link";
        transform.transform.translation.x = x_position;
        transform.transform.translation.y = y_position;
        transform.transform.rotation = tf2::toMsg(tf2::Quaternion(0, 0, sin(theta / 2), cos(theta / 2)));
        tf_broadcaster_->sendTransform(transform);
    }


    double wheel_radius;
    double wheel_distance;
    int encoder_resolution;
    double x_position, y_position, theta;
    int last_encoder_count_1A, last_encoder_count_1B, last_encoder_count_2A, last_encoder_count_2B;

    rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr encoder_info_subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_; // 추가: 오도메트리 퍼블리셔
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_; // 추가: TF 브로드캐스터
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdometryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
