#pragma once
#include <QObject>
#include <thread>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class QNode : public QObject {
  Q_OBJECT
public:
  explicit QNode(QObject* parent=nullptr);
  ~QNode();

  void publish(const std::string& text);

signals:
  void received(const QString& text);

private:
  void spinThread();

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  rclcpp::executors::SingleThreadedExecutor exec_;
  std::thread th_;
  std::atomic<bool> running_{false};
};
