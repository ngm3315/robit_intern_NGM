#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class Listener : public rclcpp::Node {
public:
  Listener() : rclcpp::Node("listener") {
    sub_ = create_subscription<std_msgs::msg::String>(
      "chatter", 10,
      [this](std_msgs::msg::String::SharedPtr msg){
        RCLCPP_INFO(get_logger(), "Recv: %s", msg->data.c_str());
      });
  }
private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Listener>());
  rclcpp::shutdown();
  return 0;
}
