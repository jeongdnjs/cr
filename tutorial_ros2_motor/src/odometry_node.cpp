#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64_multi_array.hpp"
#include <cmath>

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


    double wheel_radius;
    double wheel_distance;
    int encoder_resolution;
    double x_position, y_position, theta;
    int last_encoder_count_1A, last_encoder_count_1B, last_encoder_count_2A, last_encoder_count_2B;

    rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr encoder_info_subscription_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdometryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
