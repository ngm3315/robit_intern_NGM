#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

class IntegrateOdom : public rclcpp::Node {
public:
  IntegrateOdom() : Node("integrate_odom"), x_(0), y_(0), th_(0) {
    sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&IntegrateOdom::onTwist, this, std::placeholders::_1));
    tfbr_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    last_ = now();
    timer_ = create_wall_timer(std::chrono::milliseconds(20),
              std::bind(&IntegrateOdom::onTimer, this));
  }

private:
  void onTwist(const geometry_msgs::msg::Twist::SharedPtr m) {
    v_ = m->linear.x;
    wz_ = m->angular.z;
  }

  void onTimer() {
    auto t = now();
    double dt = (t - last_).seconds();
    last_ = t;

    // 비선형 적분
    th_ += wz_ * dt;
    double c = std::cos(th_), s = std::sin(th_);
    x_ += v_ * c * dt;
    y_ += v_ * s * dt;

    // map -> odom 변환을 '현재 포즈'로 브로드캐스트
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = t;
    tf.header.frame_id = "map";
    tf.child_frame_id  = "odom";
    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.translation.z = 0.0;
    tf2::Quaternion q; q.setRPY(0,0,th_);
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();
    tfbr_->sendTransform(tf);

    // 참고용 오도메트리도 퍼블리시
    nav_msgs::msg::Odometry od;
    od.header.stamp = t;
    od.header.frame_id = "map";
    od.child_frame_id  = "odom";
    od.pose.pose.position.x = x_;
    od.pose.pose.position.y = y_;
    od.pose.pose.position.z = 0.0;
    od.pose.pose.orientation.x = q.x();
    od.pose.pose.orientation.y = q.y();
    od.pose.pose.orientation.z = q.z();
    od.pose.pose.orientation.w = q.w();
    od.twist.twist.linear.x = v_;
    od.twist.twist.angular.z = wz_;
    odom_pub_->publish(od);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tfbr_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_;
  double v_{0.0}, wz_{0.0};
  double x_, y_, th_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IntegrateOdom>());
  rclcpp::shutdown();
  return 0;
}
