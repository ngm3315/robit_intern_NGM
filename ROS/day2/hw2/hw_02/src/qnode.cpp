#include "hw_02/qnode.hpp"
QNode::QNode(){ int argc=0; char** argv=nullptr; rclcpp::init(argc,argv);
  node=rclcpp::Node::make_shared("hw_02"); this->start(); }
QNode::~QNode(){ if(rclcpp::ok()) rclcpp::shutdown(); }
void QNode::run(){ rclcpp::WallRate r(20);
  while(rclcpp::ok()){ rclcpp::spin_some(node); r.sleep(); }
  Q_EMIT rosShutDown();
}

