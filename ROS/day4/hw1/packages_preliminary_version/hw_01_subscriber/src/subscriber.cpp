#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"

using rclcpp_lifecycle::LifecycleNode;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class LifecycleListener : public LifecycleNode {
public:
  LifecycleListener() : LifecycleNode("lifecycle_listener") {}

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
    RCLCPP_INFO(get_logger(), "configured");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
    sub_ = create_subscription<std_msgs::msg::String>(
      "/chatter", rclcpp::SystemDefaultsQoS(),
      [this](const std_msgs::msg::String & msg){
        RCLCPP_INFO(this->get_logger(), "recv: %s", msg.data.c_str());
      });
    RCLCPP_INFO(get_logger(), "activated -> start subscribing");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
    sub_.reset();
    RCLCPP_INFO(get_logger(), "deactivated -> stop subscribing");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override {
    sub_.reset();
    RCLCPP_INFO(get_logger(), "cleaned up");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override {
    sub_.reset();
    RCLCPP_INFO(get_logger(), "shutdown");
    return CallbackReturn::SUCCESS;
  }
private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LifecycleListener>();
  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(node->get_node_base_interface());
  exe.spin();
  rclcpp::shutdown();
  return 0;
}
