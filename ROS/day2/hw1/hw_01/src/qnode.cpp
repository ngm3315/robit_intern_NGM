#include "hw_01/qnode.hpp"
#include <sstream>

QNode::QNode(QObject* parent) : QThread(parent) {
  node_ = std::make_shared<rclcpp::Node>("hw01_qt_node");
  pub_ = node_->create_publisher<custom_interfaces::msg::VectorData>("vector_topic", 10);

  sub_ = node_->create_subscription<custom_interfaces::msg::VectorData>(
    "vector_topic", 10,
    [this](const custom_interfaces::msg::VectorData& msg){
      std::ostringstream oss;
      oss << "[SUB] ";
      for (auto x : msg.data) oss << x << ' ';
      emit receivedVector(QString::fromStdString(oss.str()));
    });
}

QNode::~QNode(){ stop(); }

void QNode::publishVector(const QVector<int>& v){
  custom_interfaces::msg::VectorData msg;
  msg.data.reserve(v.size());
  for(int x : v) msg.data.push_back(x);
  pub_->publish(msg);

  QString s = "[PUB] ";
  for(int x : v) s += QString::number(x) + " ";
  emit rosLog(s);
}

void QNode::run(){
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node_);
  while(running_){
    exec.spin_some();
    rclcpp::sleep_for(std::chrono::milliseconds(20));
  }
}

void QNode::stop(){
  running_ = false;
}
