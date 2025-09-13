#pragma once
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <atomic>

class QNode {
public:
  QNode();
  ~QNode();

  std::shared_ptr<rclcpp::Node> node();

private:
  void spinThread();

  std::shared_ptr<rclcpp::Node> node_;
  std::thread spin_;
  std::atomic<bool> running_{false};
};
