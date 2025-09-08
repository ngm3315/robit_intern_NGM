#pragma once                     // 헤더 파일이 중복 포함되지 않도록 함
#include <QMainWindow>           // QMainWindow 기반 클래스 포함
#include <QUdpSocket>            // UDP 통신을 위한 QUdpSocket 포함

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; } // Qt Designer로 만든 UI 클래스 선언
QT_END_NAMESPACE

// MainWindow 클래스: QMainWindow를 상속받아 GUI + UDP 통신 기능 제공
class MainWindow : public QMainWindow {
    Q_OBJECT  // Qt의 시그널/슬롯 매커니즘 사용을 선언

public:
    explicit MainWindow(QWidget *parent=nullptr);  // 생성자
    ~MainWindow();                                 // 소멸자

private slots:
    void on_pushButton_clicked();  // "send" 버튼 클릭 시 호출되는 슬롯
    void onReadyRead();            // 데이터 수신 이벤트 발생 시 호출되는 슬롯

private:
    Ui::MainWindow *ui;  // Qt Designer에서 만든 UI 객체
    QUdpSocket *sock;    // UDP 송수신을 담당할 소켓 객체
};
