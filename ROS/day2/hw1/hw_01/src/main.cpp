#include <QApplication>
#include <rclcpp/rclcpp.hpp>
#include "hw_01/main_window.hpp"

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);

  MainWindow w;
  w.show();

  int rc = app.exec();
  rclcpp::shutdown();
  return rc;
}
