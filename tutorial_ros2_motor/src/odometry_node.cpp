#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64_multi_array.hpp"
#include <cmath>

class OdometryNode : public rclcpp::Node {
public:
  OdometryNode() 
    : Node("odometry_node"),
      wheel_radius(0.0575),
      wheel_distance(0.4104),
      encoder_resolution(228),
      x_position(0.0),
      y_position(0.0),
      theta(0.0),
      last_encoder_count_1A(0),
      last_encoder_count_1B(0),
      last_encoder_count_2A(0),
      last_encoder_count_2B(0)
  {
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

    // 엔코더 값이 변했는지 확인
    if (encoder_count_1A != last_encoder_count_1A ||
        encoder_count_1B != last_encoder_count_1B ||
        encoder_count_2A != last_encoder_count_2A ||
        encoder_count_2B != last_encoder_count_2B) {
      
      double distance_left = (encoder_count_1A + encoder_count_1B - last_encoder_count_1A - last_encoder_count_1B) / 2.0 * (2 * M_PI * wheel_radius) / encoder_resolution;
      double distance_right = -(encoder_count_2A + encoder_count_2B - last_encoder_count_2A - last_encoder_count_2B) / 2.0 * (2 * M_PI * wheel_radius) / encoder_resolution;

      update_position(distance_left, distance_right);
    }

    // 엔코더 값 저장
    last_encoder_count_1A = encoder_count_1A;
    last_encoder_count_1B = encoder_count_1B;
    last_encoder_count_2A = encoder_count_2A;
    last_encoder_count_2B = encoder_count_2B;

    double x_position_cm = x_position * 100;
    double y_position_cm = y_position * 100;
    double theta_deg = theta * (180.0 / M_PI);

    RCLCPP_INFO(this->get_logger(), "Position: (%f cm, %f cm), Orientation: %f degrees", x_position_cm, y_position_cm, theta_deg);
  }

  void update_position(double distance_left, double distance_right) {
    double delta_distance = (distance_right + distance_left) / 2.0;
    double delta_theta = (distance_right - distance_left) / wheel_distance;

    theta += delta_theta;
    x_position += delta_distance * cos(theta);
    y_position += delta_distance * sin(theta);
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