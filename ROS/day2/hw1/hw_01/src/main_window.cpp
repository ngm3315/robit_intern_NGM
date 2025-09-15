#include "hw_01/main_window.hpp"
#include "ui_MainWindow.h"
#include <QStringList>
#include <QRegularExpression> 

MainWindow::MainWindow(QWidget* parent)
  : QMainWindow(parent), ui(new Ui::MainWindow), qnode(new QNode(this)) {
  ui->setupUi(this);

  connect(ui->btnPublish, &QPushButton::clicked, this, &MainWindow::on_btnPublish_clicked);
  connect(ui->btnClear,   &QPushButton::clicked, this, &MainWindow::on_btnClear_clicked);
  connect(qnode, &QNode::rosLog,        ui->textLog,  &QTextEdit::append);
  connect(qnode, &QNode::receivedVector,ui->textLog,  &QTextEdit::append);

  qnode->start();
}

MainWindow::~MainWindow(){
  qnode->stop();
  qnode->wait();
  delete ui;
}

QVector<int> MainWindow::parseInput() const {
  QVector<int> out;
  const QString s = ui->editInput->text();
  
for (const QString& tok : s.split(QRegularExpression("[,\\s]+"), Qt::SkipEmptyParts))
  out.push_back(tok.toInt());
  return out;
}

void MainWindow::on_btnPublish_clicked(){
  auto v = parseInput();
  if(v.isEmpty()){
    ui->textLog->append("[WARN] 입력이 비었습니다");
    return;
  }
  qnode->publishVector(v);
}

void MainWindow::on_btnClear_clicked(){
  ui->textLog->clear();
}
