#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64_multi_array.hpp"

class OdometryNode : public rclcpp::Node
{
public:
  OdometryNode() : Node("odometry_node")
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
    RCLCPP_INFO(this->get_logger(), "Encoder counts: Motor 1A: %d, Motor 1B: %d, Motor 2A: %d, Motor 2B: %d",
              encoder_count_1A, encoder_count_1B, encoder_count_2A, encoder_count_2B);
  }

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
