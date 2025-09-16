#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"

using rclcpp_lifecycle::LifecycleNode;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using namespace std::chrono_literals;

class LifecycleTalker : public LifecycleNode {
public:
  LifecycleTalker() : LifecycleNode("lifecycle_talker") {}

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
    pub_ = create_publisher<std_msgs::msg::String>("/chatter", rclcpp::SystemDefaultsQoS());
    count_ = 0;
    RCLCPP_INFO(get_logger(), "configured");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
    pub_->on_activate();
    timer_ = create_wall_timer(500ms, [this]{
      std_msgs::msg::String msg;
      msg.data = "hello " + std::to_string(count_++);
      pub_->publish(msg);
    });
    RCLCPP_INFO(get_logger(), "activated -> start publishing");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
    timer_.reset();
    pub_->on_deactivate();
    RCLCPP_INFO(get_logger(), "deactivated -> stop publishing");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override {
    timer_.reset();
    pub_.reset();
    RCLCPP_INFO(get_logger(), "cleaned up");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override {
    timer_.reset();
    pub_.reset();
    RCLCPP_INFO(get_logger(), "shutdown");
    return CallbackReturn::SUCCESS;
  }

private:
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  uint64_t count_{0};
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LifecycleTalker>();
  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(node->get_node_base_interface());
  exe.spin();
  rclcpp::shutdown();
  return 0;
}
