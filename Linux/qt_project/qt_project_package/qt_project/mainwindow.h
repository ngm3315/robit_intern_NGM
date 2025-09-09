#pragma once
#include <QMainWindow>
#include <QUdpSocket>
#include <QApplication>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent=nullptr);
    ~MainWindow();

private slots:
    void onReadyClicked();      // Ready 버튼 눌렀을 때
    void onReadyRead();         // UDP 데이터 수신

    void on_Quit_clicked();

private:
    Ui::MainWindow *ui;
    QUdpSocket *sock;
    QHostAddress peerAddr;
    quint16 peerPort;
};
