#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>

class DS4ToTwist : public rclcpp::Node {
public:
  DS4ToTwist() : Node("ds4_to_twist") {
    // 파라미터
    axis_linear_  = declare_parameter<int>("axis_linear", 1);   // L-스틱 세로
    axis_angular_ = declare_parameter<int>("axis_angular", 3);  // R-스틱 가로
    scale_linear_  = declare_parameter<double>("scale_linear", 1.0);
    scale_angular_ = declare_parameter<double>("scale_angular", 1.0);
    deadzone_      = declare_parameter<double>("deadzone", 0.05);

    pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&DS4ToTwist::onJoy, this, std::placeholders::_1));
  }

private:
  void onJoy(const sensor_msgs::msg::Joy::SharedPtr msg) {
    if ((int)msg->axes.size() <= std::max(axis_linear_, axis_angular_)) return;

    auto dz = [&](double v){ return (std::abs(v) < deadzone_) ? 0.0 : v; };

    // 듀얼쇼크: 위가 -1이므로 부호 반전
    double v  = -dz(msg->axes[axis_linear_])  * scale_linear_;
    double wz =  dz(msg->axes[axis_angular_]) * scale_angular_;

    geometry_msgs::msg::Twist t;
    t.linear.x  = v;
    t.angular.z = wz;
    pub_->publish(t);
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  int axis_linear_, axis_angular_;
  double scale_linear_, scale_angular_, deadzone_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DS4ToTwist>());
  rclcpp::shutdown();
  return 0;
}
