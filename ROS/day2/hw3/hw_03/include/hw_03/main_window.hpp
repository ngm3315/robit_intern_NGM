#pragma once
#include <QMainWindow>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class QNode;

class MainWindow : public QMainWindow {
  Q_OBJECT
public:
  explicit MainWindow(QWidget *parent=nullptr);
  ~MainWindow();

private slots:
  void on_publishButton_clicked();
  void onMsg(const QString& text);

private:
  Ui::MainWindow* ui;
  QNode* qnode;
};
