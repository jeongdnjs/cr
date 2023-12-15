#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64_multi_array.hpp"
#include <cmath>

class OdometryNode : public rclcpp::Node
{
public:
  OdometryNode() 
    : Node("odometry_node"), 
      wheel_radius(0.0575), // 바퀴 반지름 (미터 단위)
      wheel_distance(0.4104), // 바퀴 사이의 거리 (미터 단위)
      encoder_resolution(228), // 엔코더 해상도
      x_position(0.0), // 현재 X 위치
      y_position(0.0), // 현재 Y 위치
      theta(0.0) // 현재 방향 각도 (라디안)
  {
    encoder_info_subscription_ = this->create_subscription<std_msgs::msg::Int64MultiArray>(
        "EncoderInfo", 10,
        std::bind(&OdometryNode::encoder_info_callback, this, std::placeholders::_1));
  }

private:
  void encoder_info_callback(const std_msgs::msg::Int64MultiArray::SharedPtr msg)
  {
    int encoder_count_1A = msg->data[0];
    int encoder_count_1B = msg->data[1];
    int encoder_count_2A = msg->data[2];
    int encoder_count_2B = msg->data[3];

    double distance_left = (encoder_count_1A + encoder_count_1B) / 2.0 * (2 * M_PI * wheel_radius) / encoder_resolution;
    double distance_right = (encoder_count_2A + encoder_count_2B) / 2.0 * (2 * M_PI * wheel_radius) / encoder_resolution;

    update_position(distance_left, distance_right);
    
    RCLCPP_INFO(this->get_logger(), "Position: (%f, %f), Orientation: %f radians", x_position, y_position, theta);
  }

  void update_position(double distance_left, double distance_right)
  {
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

  rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr encoder_info_subscription_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OdometryNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
