#pragma once
#include <QMainWindow>
#include <QVector>
#include "hw_01/qnode.hpp"

namespace Ui { class MainWindow; }

class MainWindow : public QMainWindow {
  Q_OBJECT
public:
  explicit MainWindow(QWidget* parent=nullptr);
  ~MainWindow();

private slots:
  void on_btnPublish_clicked();
  void on_btnClear_clicked();

private:
  Ui::MainWindow* ui;
  QNode* qnode;
  QVector<int> parseInput() const;
};
