#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"

using rclcpp_lifecycle::LifecycleNode;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

struct QosCfg {
  std::string reliability;
  std::string durability;
  std::string history;
  int depth;
  std::string liveliness;
  int64_t lease_ms;
};

class LifecycleListener : public LifecycleNode {
public:
  LifecycleListener() : LifecycleNode("lifecycle_listener") {
    topic_ = declare_parameter<std::string>("topic", "/chatter");
    qos_ = load_qos();
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
    RCLCPP_INFO(get_logger(), "configured: topic=%s", topic_.c_str());
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
    auto qos = make_qos(qos_);
    sub_ = create_subscription<std_msgs::msg::String>(
      topic_, qos,
      [this](const std_msgs::msg::String & msg){
        RCLCPP_INFO(get_logger(), "recv: %s", msg.data.c_str());
      });
    RCLCPP_INFO(get_logger(), "activated");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
    sub_.reset();
    RCLCPP_INFO(get_logger(), "deactivated");
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
  QosCfg load_qos() {
    QosCfg c;
    c.reliability = declare_parameter<std::string>("qos.reliability", "reliable");
    c.durability  = declare_parameter<std::string>("qos.durability", "volatile");
    c.history     = declare_parameter<std::string>("qos.history", "keep_last");
    c.depth       = declare_parameter<int>("qos.depth", 10);
    c.liveliness  = declare_parameter<std::string>("qos.liveliness", "automatic");
    c.lease_ms    = declare_parameter<int64_t>("qos.liveliness_lease_ms", 0);
    return c;
  }

 rclcpp::QoS make_qos(const QosCfg& c) {
  rclcpp::QoS qos = (c.history == "keep_all")
      ? rclcpp::QoS(rclcpp::KeepAll())
      : rclcpp::QoS(rclcpp::KeepLast(c.depth));

  if (c.reliability == "best_effort") qos.best_effort(); else qos.reliable();
  if (c.durability == "transient_local") qos.transient_local(); else qos.durability_volatile();
  if (c.liveliness == "manual_by_topic") qos.liveliness(rclcpp::LivelinessPolicy::ManualByTopic);
  else qos.liveliness(rclcpp::LivelinessPolicy::Automatic);
  if (c.lease_ms > 0) qos.liveliness_lease_duration(rclcpp::Duration(std::chrono::milliseconds(c.lease_ms)));
  return qos;
}


  std::string topic_;
  QosCfg qos_;
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
