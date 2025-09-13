#include "hw_03/qnode.hpp"

QNode::QNode(QObject* parent) : QObject(parent) {
  if (!rclcpp::ok()) rclcpp::init(0, nullptr);
  node_ = std::make_shared<rclcpp::Node>("hw03_qt_node");

  pub_ = node_->create_publisher<std_msgs::msg::String>("/hw03/chat", 10);

  sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/hw03/chat", 10,
    [this](std_msgs::msg::String::SharedPtr msg){
      emit received(QString::fromStdString(msg->data));
    });

  exec_.add_node(node_);
  running_ = true;
  th_ = std::thread(&QNode::spinThread, this);
}

QNode::~QNode() {
  running_ = false;
  if (th_.joinable()) th_.join();
  exec_.cancel();
  sub_.reset();
  pub_.reset();
  node_.reset();
  // rclcpp::shutdown(); // 다른 노드와 공유 가능성 고려해 생략
}

void QNode::spinThread() {
  while (running_ && rclcpp::ok()) {
    exec_.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void QNode::publish(const std::string& text) {
  auto msg = std_msgs::msg::String();
  msg.data = text;
  pub_->publish(msg);
}
