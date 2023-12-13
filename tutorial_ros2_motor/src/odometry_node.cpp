#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <cmath>

class OdometryNode : public rclcpp::Node
{
public:
    OdometryNode() : Node("odometry_node"), x(0.0), y(0.0), th(0.0)
    {
        encoder_subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "encoder_data", 10,
            std::bind(&OdometryNode::encoderCallback, this, std::placeholders::_1));

        odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    }

private:
    void encoderCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
    {
        int encoder_count_1A = msg->data[0];
        int encoder_count_1B = msg->data[1];
        int encoder_count_2A = msg->data[2];
        int encoder_count_2B = msg->data[3];

        double wheel_radius = 5.75; // cm
        double robot_radius = 20.52; // cm
        int encoder_resolution = 228;

        double wheel_circumference = 2.0 * M_PI * wheel_radius;
        double distance_1 = ((encoder_count_1A + encoder_count_1B) / static_cast<double>(encoder_resolution)) * wheel_circumference;
        double distance_2 = ((encoder_count_2A + encoder_count_2B) / static_cast<double>(encoder_resolution)) * wheel_circumference;

        double total_distance = (distance_1 + distance_2) / 2.0;
        double rotation = (distance_1 - distance_2) / (robot_radius * 2.0);

        // Update odometry
        double delta_x = total_distance * cos(th);
        double delta_y = total_distance * sin(th);
        th += rotation;

        x += delta_x;
        y += delta_y;

        // Publish odometry message
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = this->now();
        odom.header.frame_id = "odom";
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.orientation = toQuaternion(0, 0, th);

        odometry_publisher_->publish(odom);
    }

    geometry_msgs::msg::Quaternion toQuaternion(double roll, double pitch, double yaw)
    {
        geometry_msgs::msg::Quaternion q;
        // Convert euler angle to quaternion
        double t0 = std::cos(yaw * 0.5);
        double t1 = std::sin(yaw * 0.5);
        double t2 = std::cos(roll * 0.5);
        double t3 = std::sin(roll * 0.5);
        double t4 = std::cos(pitch * 0.5);
        double t5 = std::sin(pitch * 0.5);

        q.w = t0 * t2 * t4 + t1 * t3 * t5;
        q.x = t0 * t3 * t4 - t1 * t2 * t5;
        q.y = t0 * t2 * t5 + t1 * t3 * t4;
4        return q;
    }

    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr encoder_subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;

    double x, y, th; // Position and orientation
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometryNode>());
    rclcpp::shutdown();
    return 0;
}
