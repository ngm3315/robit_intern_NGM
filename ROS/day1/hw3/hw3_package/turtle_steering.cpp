// turtlesim을 키보드(WASD)와 단축키(t,r,c)로 조작해 도형을 그리는 노드 예제
// - /turtle1/cmd_vel 로 속도 명령(Twist) 퍼블리시
// - /turtle1/set_pen 서비스로 펜 색과 굵기 설정
// - t: 삼각형, r: 사각형, c: 원, q: 종료

#include <rclcpp/rclcpp.hpp>                 // ROS2 C++ 클라이언트 라이브러리
#include <geometry_msgs/msg/twist.hpp>       // 속도 명령 메시지
#include <turtlesim/srv/set_pen.hpp>         // 펜 설정 서비스
#include <termios.h>                         // 터미널 입력 모드 제어(비차단/에코 끄기)
#include <unistd.h>                          // POSIX 표준
#include <iostream>

class TurtleSteering : public rclcpp::Node {
public:
  // 노드 이름을 "turtle_steering"으로 초기화
  TurtleSteering() : Node("turtle_steering") {
    // 거북이 선속/각속 명령 퍼블리셔 생성
    pub_ = create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    // 펜 설정 서비스 클라이언트 생성
    client_pen_ = create_client<turtlesim::srv::SetPen>("/turtle1/set_pen");
  }

  // 메인 루프: 키 입력을 받아 동작 실행
  void run() {
    std::cout << "WASD로 이동, t=삼각형, r=사각형, c=원, q=종료\n";
    while (rclcpp::ok()) {
      char key = getKey();              // 터미널에서 1글자 입력
      if (key == 'q') break;            // 종료
      if (key == 'w') move(2.0, 0.0);   // 전진
      else if (key == 's') move(-2.0, 0.0); // 후진
      else if (key == 'a') move(0.0, 2.0);  // 좌회전(제자리)
      else if (key == 'd') move(0.0, -2.0); // 우회전(제자리)
      else if (key == 't') drawTriangle();  // 삼각형
      else if (key == 'r') drawSquare();    // 사각형
      else if (key == 'c') drawCircle();    // 원
    }
  }

private:
  // 퍼블리셔와 서비스 클라이언트 핸들
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr client_pen_;

  // 일정 시간(duration) 동안 선속/각속 명령을 퍼블리시
  // lin: linear.x, ang: angular.z, duration: 초 단위
  void move(double lin, double ang, double duration=1.0) {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = lin;
    cmd.angular.z = ang;

    rclcpp::Rate rate(10);                       // 10 Hz
    int ticks = static_cast<int>(duration * 10); // 총 반복 횟수
    for (int i = 0; i < ticks; i++) {
      pub_->publish(cmd);
      rate.sleep();
    }
    stop(); // 정지 명령 한번 보내서 관성 제거
  }

  // 즉시 정지(0 속도 퍼블리시)
  void stop() {
    geometry_msgs::msg::Twist cmd; // 기본값 0
    pub_->publish(cmd);
  }

  // 펜 색상과 굵기 설정
  // r,g,b: 0~255, width: 선 굵기, off=true면 펜 끔(이동해도 선 미표시)
  void setPen(int r, int g, int b, int width, bool off=false) {
    auto req = std::make_shared<turtlesim::srv::SetPen::Request>();
    req->r = r; req->g = g; req->b = b; req->width = width; req->off = off;

    // 서비스가 올라올 때까지 대기
    while (!client_pen_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_INFO(get_logger(), "waiting pen service");
    }
    // 비동기 요청(응답은 굳이 확인하지 않음)
    client_pen_->async_send_request(req);
  }

  // 삼각형 그리기: 빨강, 굵기 3
  // 각 변 그리기 → 일정 각속도로 회전 → 3회 반복
  void drawTriangle() {
    setPen(255, 0, 0, 3);
    for (int i = 0; i < 3; i++) {
      move(2.0, 0.0, 1.5);  // 직선 이동
      move(0.0, 2.1, 1.0);  // 회전(약 120도 근사)
    }
  }

  // 사각형 그리기: 초록, 굵기 4
  // 90도 회전을 위해 각속도와 시간(1.57초)을 근사 사용
  void drawSquare() {
    setPen(0, 255, 0, 4);
    for (int i = 0; i < 4; i++) {
      move(2.0, 0.0, 1.5);   // 직선
      move(0.0, 1.57, 1.0);  // 90도 회전 근사
    }
  }

  // 원 그리기: 파랑, 굵기 2
  // 선속과 각속도를 동시에 주면 원형 궤적
  void drawCircle() {
    setPen(0, 0, 255, 2);
    move(2.0, 1.0, 6.5); // 반지름과 둘레 시간은 파라미터 조절로 변경 가능
  }

  // 터미널에서 즉시 1글자 읽기(엔터 없이)
  // ICANON, ECHO 비활성화로 버퍼링/에코 제거
  char getKey() {
    struct termios oldt, newt;
    char c;
    tcgetattr(STDIN_FILENO, &oldt);      // 현재 터미널 설정 백업
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);    // canonical 모드/에코 비활성화
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    c = getchar();                       // 1글자 즉시 읽기
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // 원래 설정 복원
    return c;
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TurtleSteering>(); // 노드 생성
  node->run();                                    // 키 입력 루프 시작
  rclcpp::shutdown();
  return 0;
}
