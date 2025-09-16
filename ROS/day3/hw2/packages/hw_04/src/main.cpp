#include <QApplication>
#include "hw_04/main_window.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[]) {
  // ROS2 먼저 초기화. guard condition 오류 방지.
  rclcpp::init(argc, argv);

  QApplication app(argc, argv);
  MainWindow w;
  w.show();

  int rc = app.exec();
  rclcpp::shutdown();
  return rc;
}
