#include <rclcpp/rclcpp.hpp>
#include <custom_interfaces/msg/vector_data.hpp>
#include <iostream>
#include <sstream>

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("vector_publisher_cli");
  auto pub  = node->create_publisher<custom_interfaces::msg::VectorData>("vector_topic", 10);

  while (rclcpp::ok()){
    std::cout << "정수 입력(q 종료): ";
    std::string line; if(!std::getline(std::cin, line)) break;
    if(line=="q"||line=="Q") break;

    custom_interfaces::msg::VectorData msg;
    std::stringstream ss(line);
    int val;
    while (ss >> val) msg.data.push_back(val);

    pub->publish(msg);
    std::cout << "Published " << msg.data.size() << " ints\n";
  }
  rclcpp::shutdown();
  return 0;
}
