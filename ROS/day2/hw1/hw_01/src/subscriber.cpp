#include <rclcpp/rclcpp.hpp>
#include <custom_interfaces/msg/vector_data.hpp>
#include <iostream>

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("vector_subscriber_cli");
  auto sub = node->create_subscription<custom_interfaces::msg::VectorData>(
    "vector_topic", 10,
    [](const custom_interfaces::msg::VectorData& msg){
      std::cout << "[CLI SUB] ";
      for(auto x: msg.data) std::cout << x << ' ';
      std::cout << std::endl;
    });
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
