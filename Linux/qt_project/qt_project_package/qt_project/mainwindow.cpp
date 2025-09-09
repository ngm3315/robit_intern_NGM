#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QHostAddress>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow), sock(new QUdpSocket(this))
{
    ui->setupUi(this);

    // 내 IP/Port 바인드
    QHostAddress myAddr("192.168.0.30");   // 내 PC IP
    quint16 myPort = 45454;                // 수신용 포트
    sock->bind(myAddr, myPort,
               QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint);

    // 상대방 정보 (송신 대상)
    peerAddr = QHostAddress("192.168.0.31");  // 상대방 PC IP
    peerPort = 45455;                          // 상대방 포트

    // 슬롯 연결
    connect(ui->Ready, &QPushButton::clicked, this, &MainWindow::onReadyClicked);
    connect(sock, &QUdpSocket::readyRead, this, &MainWindow::onReadyRead);
}

MainWindow::~MainWindow() { delete ui; }

void MainWindow::onReadyClicked() {
    // 자기 메시지 출력
    ui->Readyornot->append("나 : Ready");

    // UDP로 "Ready" 송신
    QByteArray dat = "Ready";
    sock->writeDatagram(dat, peerAddr, peerPort);
}

void MainWindow::onReadyRead() {
    while (sock->hasPendingDatagrams()) {
        QByteArray buf;
        buf.resize(sock->pendingDatagramSize());
        QHostAddress sender;
        quint16 senderPort;
        sock->readDatagram(buf.data(), buf.size(), &sender, &senderPort);

        // 수신 내용 Readyornot에 출력
        QString msg = QString::fromUtf8(buf);
        ui->Readyornot->append("상대방 : " + msg);
    }
}

void MainWindow::on_Quit_clicked()
{
    QApplication::quit();
}

