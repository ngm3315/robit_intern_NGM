#include "rclcpp/rclcpp.hpp"

class MyCppNode : public rclcpp::Node
{
public:
    MyCppNode() : Node("my_cpp_node")
    {
        RCLCPP_INFO(this->get_logger(), "Hello, ROS 2 C++ Node!");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyCppNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
MyNode::MyNode() : Node("mynode")
{
    publisher = this->create_publisher<std_msgs::msg::String>("topicname", 10);
}

MyNode::~MyNode()
{
}
