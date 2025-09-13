#pragma once
#include <QThread>
#include <rclcpp/rclcpp.hpp>
class QNode : public QThread {
  Q_OBJECT
public:
  QNode();
  ~QNode();
protected:
  void run() override;
private:
  std::shared_ptr<rclcpp::Node> node;
Q_SIGNALS:
  void rosShutDown();   // 선언만
};

