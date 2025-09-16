#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/srv/set_pen.hpp>
#include <termios.h>
#include <unistd.h>
#include <chrono>
#include <cmath>
#include <iostream>

using namespace std::chrono_literals;

class Drawer : public rclcpp::Node {
public:
  Drawer() : Node("hw01_drawer") {
    pub_ = create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    pen_cli_ = create_client<turtlesim::srv::SetPen>("/turtle1/set_pen");
    RCLCPP_INFO(get_logger(),
      "Controls: WASD move, q stop\n"
      "t triangle(red,3)  r square(green,4)  c circle(blue,2)\n");
  }

  void spin_keyboard() {
    enable_raw_();
    while (rclcpp::ok()) {
      char ch;
      if (read(STDIN_FILENO, &ch, 1) == 1) {
        if (ch == 'q') { stop(); break; }
        handle_key_(ch);
      }
      rclcpp::sleep_for(5ms);
    }
    disable_raw_();
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr pen_cli_;
  struct termios old_{};

  void set_pen_(int r,int g,int b,int width,bool off=false){
    auto req = std::make_shared<turtlesim::srv::SetPen::Request>();
    req->r=r; req->g=g; req->b=b; req->width=width; req->off = off?1u:0u;
    if (!pen_cli_->wait_for_service(1s)) return;
    auto fut = pen_cli_->async_send_request(req);
    (void)fut;
  }

  void pub_twist_(double lin, double ang){
    geometry_msgs::msg::Twist t;
    t.linear.x = lin; t.angular.z = ang;
    pub_->publish(t);
  }

  void move_for(double v, double sec){           // 직진 v m/s, sec 초
    auto end = now_sec_() + sec;
    while (rclcpp::ok() && now_sec_() < end){ pub_twist_(v,0.0); rclcpp::sleep_for(20ms); }
    stop();
  }
  void rotate_for(double wz, double sec){        // 회전 wz rad/s, sec 초
    auto end = now_sec_() + sec;
    while (rclcpp::ok() && now_sec_() < end){ pub_twist_(0.0,wz); rclcpp::sleep_for(20ms); }
    stop();
  }
  void circle_for(double v, double wz, double sec){
    auto end = now_sec_() + sec;
    while (rclcpp::ok() && now_sec_() < end){ pub_twist_(v,wz); rclcpp::sleep_for(20ms); }
    stop();
  }
  static double now_sec_(){
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
  }
  void stop(){ pub_twist_(0,0); }

  // 도형
  void draw_triangle(){
    set_pen_(255,0,0,3);
    for(int i=0;i<3;i++){ move_for(2.0,1.0); rotate_for(2.0944,1.0); } // 120도
  }
  void draw_square(){
    set_pen_(0,255,0,4);
    for(int i=0;i<4;i++){ move_for(2.0,1.0); rotate_for(M_PI/2,1.0); } // 90도
  }
  void draw_circle(){
    set_pen_(0,0,255,2);
    circle_for(2.0,2.0,3.2); // 속도·시간은 자유 조정
  }

  // 키 입력 처리
  void handle_key_(char ch){
    switch(ch){
      case 'w': pub_twist_(2.0,0.0); break;
      case 's': pub_twist_(-2.0,0.0); break;
      case 'a': pub_twist_(0.0, 2.0); break;
      case 'd': pub_twist_(0.0,-2.0); break;
      case 't': draw_triangle(); break;
      case 'r': draw_square();  break;
      case 'c': draw_circle();  break;
      case ' ': stop();         break;
      default: break;
    }
  }

  void enable_raw_(){
    tcgetattr(STDIN_FILENO, &old_);
    struct termios raw = old_;
    raw.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
  }
  void disable_raw_(){ tcsetattr(STDIN_FILENO, TCSANOW, &old_); }
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Drawer>();
  // 키보드 루프는 메인 스레드에서
  node->spin_keyboard();
  rclcpp::shutdown();
  return 0;
}
