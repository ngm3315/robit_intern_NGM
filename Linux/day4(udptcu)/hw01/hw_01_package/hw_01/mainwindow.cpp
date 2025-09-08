#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QHostAddress>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , sock(new QUdpSocket(this))   // QUdpSocket 객체 생성
{
    ui->setupUi(this);

    // 내 PC의 IP와 포트를 바인드해서 수신 대기 시작
    QHostAddress myAddr("172.100.4.199");   // 내 PC의 사설 IP 주소
    quint16 myPort = 45454;                 // 내가 수신할 포트 번호
    sock->bind(myAddr, myPort,
               QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint);

    // 데이터 수신 이벤트가 발생하면 onReadyRead() 슬롯 실행
    connect(sock, &QUdpSocket::readyRead, this, &MainWindow::onReadyRead);
}

MainWindow::~MainWindow() { delete ui; }

void MainWindow::on_pushButton_clicked() {
    // 내가 보낼 메시지 가져오기
    QString msg = ui->send->toPlainText();
    QByteArray dat = msg.toUtf8();

    // 상대방의 IP와 포트 지정 (상대방은 이 포트로 바인드해야 수신 가능)
    QHostAddress peer("172.100.7.254");
    sock->writeDatagram(dat, peer, 45455);

    // 내 화면(textEdit)에 내가 보낸 메시지도 바로 표시
    ui->textEdit->append("나: " + msg);

    // 입력창 비우기
    ui->send->clear();
}

void MainWindow::onReadyRead() {
    // 수신 버퍼에 대기 중인 데이터그램이 있으면 모두 처리
    while (sock->hasPendingDatagrams()) {
        QByteArray buf;
        buf.resize(int(sock->pendingDatagramSize()));  // 수신 데이터 크기만큼 버퍼 준비
        QHostAddress sender;
        quint16 senderPort;

        // 실제 데이터 읽어오기
        sock->readDatagram(buf.data(), buf.size(), &sender, &senderPort);

        // 수신한 메시지를 화면(textEdit)에 "상대:" 라벨과 함께 출력
        ui->textEdit->append("상대: " + QString::fromUtf8(buf));
    }
}
