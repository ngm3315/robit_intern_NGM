#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>

class MultiListener : public rclcpp::Node {
public:
  MultiListener() : Node("multi_listener") {
    sub_int_ = create_subscription<std_msgs::msg::Int32>(
      "int_topic", 10,
      [this](std_msgs::msg::Int32::SharedPtr msg) {
        RCLCPP_INFO(get_logger(), "Recv int=%d", msg->data);
      });

    sub_str_ = create_subscription<std_msgs::msg::String>(
      "string_topic", 10,
      [this](std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(get_logger(), "Recv str=%s", msg->data.c_str());
      });

    sub_flt_ = create_subscription<std_msgs::msg::Float32>(
      "float_topic", 10,
      [this](std_msgs::msg::Float32::SharedPtr msg) {
        RCLCPP_INFO(get_logger(), "Recv float=%.2f", msg->data);
      });
  }

private:
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_int_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_str_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_flt_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MultiListener>());
  rclcpp::shutdown();
  return 0;
}
