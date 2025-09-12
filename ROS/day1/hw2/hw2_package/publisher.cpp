#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>

using namespace std::chrono_literals;

class MultiPublisher : public rclcpp::Node {
public:
  MultiPublisher() : Node("multi_publisher"), count_(0) {
    pub_int_ = create_publisher<std_msgs::msg::Int32>("int_topic", 10);
    pub_str_ = create_publisher<std_msgs::msg::String>("string_topic", 10);
    pub_flt_ = create_publisher<std_msgs::msg::Float32>("float_topic", 10);

    timer_ = create_wall_timer(1s, [this] { publish_msgs(); });
  }

private:
  void publish_msgs() {
    auto i = std_msgs::msg::Int32();
    i.data = count_;

    auto s = std_msgs::msg::String();
    s.data = "Hello " + std::to_string(count_);

    auto f = std_msgs::msg::Float32();
    f.data = 3.14f * count_;

    pub_int_->publish(i);
    pub_str_->publish(s);
    pub_flt_->publish(f);

    RCLCPP_INFO(get_logger(), "Published -> int=%d, str=%s, float=%.2f",
                i.data, s.data.c_str(), f.data);

    count_++;
  }

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_int_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_str_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_flt_;
  rclcpp::TimerBase::SharedPtr timer_;
  int count_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MultiPublisher>());
  rclcpp::shutdown();
  return 0;
}
