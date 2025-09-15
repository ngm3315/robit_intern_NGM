#pragma once
#include <QThread>
#include <QString>
#include <QVector>
#include <rclcpp/rclcpp.hpp>
#include <custom_interfaces/msg/vector_data.hpp>

class QNode : public QThread {
  Q_OBJECT
public:
  explicit QNode(QObject* parent=nullptr);
  ~QNode() override;

  void publishVector(const QVector<int>& v);
  void stop();

signals:
  void rosLog(QString line);
  void receivedVector(QString line);

protected:
  void run() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<custom_interfaces::msg::VectorData>::SharedPtr pub_;
  rclcpp::Subscription<custom_interfaces::msg::VectorData>::SharedPtr sub_;
  std::atomic_bool running_{true};
};
