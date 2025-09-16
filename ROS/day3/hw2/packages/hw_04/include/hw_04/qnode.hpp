#pragma once
#include <rclcpp/rclcpp.hpp>
#include <vector>

class QNode : public rclcpp::Node {
public:
  QNode();                                // ← 인자 없는 생성자로 통일
  // 필요 시 getter 추가
  std::vector<double> link_lengths() const { return link_lengths_; }
  std::vector<double> init_angles()  const { return init_angles_; }

private:
  std::vector<double> link_lengths_;
  std::vector<double> init_angles_;
};
