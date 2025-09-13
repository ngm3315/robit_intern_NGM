#include "hw_03/main_window.hpp"
#include "ui_mainwindow.h"
#include "hw_03/qnode.hpp"
#include <QMessageBox>

MainWindow::MainWindow(QWidget *parent)
  : QMainWindow(parent), ui(new Ui::MainWindow), qnode(new QNode(this)) {
  ui->setupUi(this);
  setWindowTitle("HW03 Talker/Listener");
  connect(qnode, &QNode::received, this, &MainWindow::onMsg);
}

MainWindow::~MainWindow() { delete ui; }

void MainWindow::on_publishButton_clicked() {
  const QString text = ui->lineEdit_input->text();
  if (text.isEmpty()) return;
  qnode->publish(text.toStdString());
}

void MainWindow::onMsg(const QString& text) {
  ui->label_recv->setText(text);
}
