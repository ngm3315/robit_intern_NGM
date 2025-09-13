#include <QApplication>
#include "hw_03/main_window.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  // ROS2 초기화 먼저
  rclcpp::init(argc, argv);

  QApplication app(argc, argv);
  MainWindow w;
  w.show();

  int ret = app.exec();

  // 종료 시 ROS2도 shutdown
  rclcpp::shutdown();
  return ret;
}
