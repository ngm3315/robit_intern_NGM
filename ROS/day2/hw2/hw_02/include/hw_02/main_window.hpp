#pragma once
#include <QMainWindow>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/srv/set_pen.hpp>

namespace Ui { class MainWindow; }

class MainWindow : public QMainWindow {
  Q_OBJECT
public:
  explicit MainWindow(QWidget* parent=nullptr);
  ~MainWindow();

private slots:
  void goForward();
  void goBackward();
  void turnLeft();
  void turnRight();
  void triangle();
  void square();
  void circle();

private:
  void setPen(int r,int g,int b,int width);

  Ui::MainWindow* ui;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr pen_client_;
};
