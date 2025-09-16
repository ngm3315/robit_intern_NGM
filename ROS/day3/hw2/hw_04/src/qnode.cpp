#include "hw_04/qnode.hpp"

// 인자 없는 생성자 정의로 통일
QNode::QNode() : rclcpp::Node("robot_arm") {
  declare_parameter<std::vector<double>>("link_lengths", {100.0,100.0,100.0});
  declare_parameter<std::vector<double>>("init_angles",  {0.0,0.0,0.0});
  link_lengths_ = get_parameter("link_lengths").as_double_array();
  init_angles_  = get_parameter("init_angles").as_double_array();

  RCLCPP_INFO(get_logger(), "links=[%f,%f,%f] init=[%f,%f,%f]",
              link_lengths_[0], link_lengths_[1], link_lengths_[2],
              init_angles_[0],  init_angles_[1],  init_angles_[2]);
}
