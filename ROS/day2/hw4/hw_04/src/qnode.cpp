#include "hw_04/qnode.hpp"

QNode::QNode() {
  node_ = std::make_shared<rclcpp::Node>("hw_04_node");
  running_ = true;
  spin_ = std::thread([this]{ this->spinThread(); });
}

QNode::~QNode() {
  running_ = false;
  if (spin_.joinable()) spin_.join();
}

std::shared_ptr<rclcpp::Node> QNode::node() { return node_; }

void QNode::spinThread() {
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node_);
  while (running_) {
    exec.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}
