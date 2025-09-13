#include "hw_02/main_window.hpp"
#include "ui_mainwindow.h"
using namespace std::chrono_literals;

MainWindow::MainWindow(QWidget* p): QMainWindow(p), ui(new Ui::MainWindow){
  ui->setupUi(this);
  node_ = std::make_shared<rclcpp::Node>("qt_turtle_node");
  pub_  = node_->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel",10);
  pen_client_ = node_->create_client<turtlesim::srv::SetPen>("/turtle1/set_pen");

  connect(ui->w,&QPushButton::clicked,this,&MainWindow::goForward);
  connect(ui->a,&QPushButton::clicked,this,&MainWindow::turnLeft);
  connect(ui->s,&QPushButton::clicked,this,&MainWindow::goBackward);
  connect(ui->d,&QPushButton::clicked,this,&MainWindow::turnRight);
  connect(ui->circle,&QPushButton::clicked,this,&MainWindow::circle);
  connect(ui->triangle,&QPushButton::clicked,this,&MainWindow::triangle);
  connect(ui->square,&QPushButton::clicked,this,&MainWindow::square);
}

MainWindow::~MainWindow(){ delete ui; }

void MainWindow::setPen(int r,int g,int b,int width){
  auto req = std::make_shared<turtlesim::srv::SetPen::Request>();
  req->r=r; req->g=g; req->b=b; req->width=width; req->off=0;
  pen_client_->async_send_request(req);
}

void MainWindow::goForward(){ geometry_msgs::msg::Twist m; m.linear.x=2.0; pub_->publish(m); }
void MainWindow::goBackward(){ geometry_msgs::msg::Twist m; m.linear.x=-2.0; pub_->publish(m); }
void MainWindow::turnLeft(){ geometry_msgs::msg::Twist m; m.angular.z=1.5; pub_->publish(m); }
void MainWindow::turnRight(){ geometry_msgs::msg::Twist m; m.angular.z=-1.5; pub_->publish(m); }

void MainWindow::triangle(){
  setPen(255,0,0,3);
  geometry_msgs::msg::Twist m;
  for(int i=0;i<3;i++){
    m.linear.x=2.0; m.angular.z=0.0; pub_->publish(m); rclcpp::sleep_for(2s);
    m.linear.x=0.0; m.angular.z=2.094; pub_->publish(m); rclcpp::sleep_for(1s);
  }
}

void MainWindow::square(){
  setPen(0,255,0,4);
  geometry_msgs::msg::Twist m;
  for(int i=0;i<4;i++){
    m.linear.x=2.0; m.angular.z=0.0; pub_->publish(m); rclcpp::sleep_for(2s);
    m.linear.x=0.0; m.angular.z=1.571; pub_->publish(m); rclcpp::sleep_for(1s);
  }
}

void MainWindow::circle(){
  setPen(0,0,255,2);
  geometry_msgs::msg::Twist m;
  m.linear.x=2.0; m.angular.z=2.0;
  for(int i=0;i<50;i++){ pub_->publish(m); rclcpp::sleep_for(100ms); }
}
