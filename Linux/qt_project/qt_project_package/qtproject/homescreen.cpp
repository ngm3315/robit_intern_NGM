#include "homescreen.h"
#include "ui_homescreen.h"
#include <QMessageBox>
#include <QHostAddress>
#include <QApplication>
#include <QDataStream>
#include <QDebug>
#include <QTimer>

HomeScreen::HomeScreen(QWidget *parent)
    : QWidget(parent), ui(new Ui::HomeScreen)
{
    ui->setupUi(this);
    connect(ui->connect_button, &QPushButton::clicked, this, &HomeScreen::onConnectClicked);
    connect(ui->quit_button, &QPushButton::clicked, this, &HomeScreen::onQuitClicked);
    connect(ui->rematch_button, &QPushButton::clicked, this, &HomeScreen::onRematchClicked);
    connect(ui->exit_to_home_button, &QPushButton::clicked, this, &HomeScreen::onExitToHomeClicked);

    ui->countdownLabel->setVisible(false);
    ui->status_label->setText("Enter your local port, opponent IP and port, then click Connect.");
    

    m_readyCheckTimer = new QTimer(this);
    connect(m_readyCheckTimer, &QTimer::timeout, this, &HomeScreen::sendReadyStatus);
}

HomeScreen::~HomeScreen() { delete ui; }

void HomeScreen::resetUI() {
    ui->connect_button->setText("Connect");
    ui->connect_button->setEnabled(true);
    ui->ip_input_line->setEnabled(true);
    ui->port_input_line->setEnabled(true);
    ui->local_port_input_line->setEnabled(true);
    ui->status_label->setText("Enter your local port, opponent IP and port, then click Connect.");
    ui->countdownLabel->setVisible(false);
    m_isReady = false;
    m_opponentReady = false;
    m_amIHost = false;
    
    ui->stackedWidget->setCurrentIndex(ui->stackedWidget->indexOf(ui->Connection_page));
    m_readyCheckTimer->stop(); // Ensure timer is stopped on reset
}

void HomeScreen::setStatus(const QString& status, bool isError) {
    ui->status_label->setText(status);
    ui->status_label->setStyleSheet(isError ? "color: red;" : "color: black;");
}

void HomeScreen::showCountdown(int count) {
    if (count > 0) {
        ui->countdownLabel->setVisible(true);
        ui->countdownLabel->setText(QString::number(count));
    } else {
        ui->countdownLabel->setVisible(false);
    }
}

void HomeScreen::showResultPage(const QString& result) {
    ui->result_text_box->setText(result);
    ui->stackedWidget->setCurrentIndex(ui->stackedWidget->indexOf(ui->Result_page));
}

void HomeScreen::showConnectionPage() {
    ui->stackedWidget->setCurrentIndex(ui->stackedWidget->indexOf(ui->Connection_page));
}

void HomeScreen::setRematchStatus(const QString& status) {
    ui->rematch_status_label->setText(status);
}

void HomeScreen::enableRematchButtons(bool enable) {
    ui->rematch_button->setEnabled(enable);
    ui->exit_to_home_button->setEnabled(enable);
}

void HomeScreen::setMyAddress(const QHostAddress& addr) {
    m_myAddress = addr;
    qDebug() << "HomeScreen: My local address set to " << m_myAddress.toString();
}

void HomeScreen::onConnectClicked() {
    QString localPortStr = ui->local_port_input_line->text();
    QString ip = ui->ip_input_line->text();
    QString peerPortStr = ui->port_input_line->text();
    bool okLocal, okPeer;
    quint16 localPort = localPortStr.toUShort(&okLocal);
    quint16 peerPort = peerPortStr.toUShort(&okPeer);

    if (localPortStr.isEmpty() || !okLocal || localPort == 0 ||
        ip.isEmpty() || peerPortStr.isEmpty() || !okPeer || peerPort == 0) {
        setStatus("Please enter valid local port, opponent IP and port.", true);
        qDebug() << "Validation failed: localPort=" << localPort << ", ip=" << ip << ", peerPort=" << peerPort;
        return;
    }

    m_localPort = localPort;
    m_peerAddress = QHostAddress(ip);
    m_peerPort = peerPort;
    

    qDebug() << "onConnectClicked: m_localPort=" << m_localPort << ", m_peerAddress=" << m_peerAddress << ", m_peerPort=" << m_peerPort;

    // First click is 'Connect', subsequent clicks are 'Ready'
    if (ui->connect_button->text() == "Connect") {
        setStatus("Ports and IP set. Click Ready to start.");
        ui->connect_button->setText("Ready");
        ui->connect_button->setEnabled(false);
        ui->ip_input_line->setEnabled(false);
        ui->port_input_line->setEnabled(false);
        ui->local_port_input_line->setEnabled(false);
        emit bindSocket(m_localPort);
        QByteArray dat;
        QDataStream out(&dat, QIODevice::WriteOnly);
        out << QString("CONNECT") << m_myAddress.toString() << m_localPort;
        emit sendDatagram(dat, m_peerAddress, m_peerPort);
        return;
    }
    if (!m_peerConnected) {                         // ← 상대가 아직 CONNECT 안 됨
        setStatus("Peer is not connected yet. Please wait...", true);
        return;
    }
    // This is the 'Ready' click
    if (m_isReady) return; // Prevent multiple ready clicks

    m_isReady = true;
    setStatus("Ready! Waiting for opponent...");
    ui->connect_button->setEnabled(false);

    m_readyCheckTimer->start(2000);

    QByteArray dat;
    QDataStream out(&dat, QIODevice::WriteOnly);
    out << QString("READY") << m_myAddress.toString()<< m_localPort; // Include my address string
    emit sendDatagram(dat, m_peerAddress, m_peerPort);

    qDebug() << "onConnectClicked: Sent READY. m_isReady=" << m_isReady;
    checkReadyState();
}

void HomeScreen::sendReadyStatus() {
    if (m_isReady && !m_opponentReady) { // Only send if I'm ready but opponent isn't
        QByteArray dat;
        QDataStream out(&dat, QIODevice::WriteOnly);
        out << QString("READY") << m_myAddress.toString()<< m_localPort; // Include my address string
        emit sendDatagram(dat, m_peerAddress, m_peerPort);
        qDebug() << "sendReadyStatus: Periodically sending READY.";
    }
}
// homescreen.cpp의 onDatagramReceived 함수에서 주소 비교 부분을 이렇게 수정하세요:

void HomeScreen::onDatagramReceived(const QByteArray& data, QHostAddress sender, quint16 senderPort, bool isConnected) {
    qDebug() << "onDatagramReceived: Data received from " << sender << ":" << senderPort << ", size=" << data.size();
    qDebug() << "  isConnected=" << isConnected << ", m_peerAddress=" << m_peerAddress << ", m_peerPort=" << m_peerPort;

    // Ensure we are connected and the sender is our peer
    if (!isConnected || senderPort != m_peerPort) {
        qDebug() << "  Ignoring datagram: Not connected or wrong port. Sender port: " << senderPort << ", Expected port: " << m_peerPort;
        return;
    }

    // 주소 비교 - IPv6-mapped IPv4 주소 처리
    bool addressMatch = false;

    // 직접 비교
    if (sender == m_peerAddress) {
        addressMatch = true;
    }
    // IPv6-mapped IPv4 주소인지 확인
    else if (sender.protocol() == QAbstractSocket::IPv6Protocol && m_peerAddress.protocol() == QAbstractSocket::IPv4Protocol) {
        // sender가 IPv6-mapped IPv4 형태인지 확인하고, IPv4 부분 추출해서 비교
        QString senderStr = sender.toString();
        if (senderStr.startsWith("::ffff:")) {
            QString ipv4Part = senderStr.mid(7); // "::ffff:" 제거
            if (QHostAddress(ipv4Part) == m_peerAddress) {
                addressMatch = true;
            }
        }
    }
    // 반대의 경우 (sender가 IPv4, peer가 IPv6-mapped)
    else if (sender.protocol() == QAbstractSocket::IPv4Protocol && m_peerAddress.protocol() == QAbstractSocket::IPv6Protocol) {
        QString peerStr = m_peerAddress.toString();
        if (peerStr.startsWith("::ffff:")) {
            QString ipv4Part = peerStr.mid(7);
            if (sender == QHostAddress(ipv4Part)) {
                addressMatch = true;
            }
        }
    }

    if (!addressMatch) {
        qDebug() << "  Ignoring datagram: Address mismatch. Sender: " << sender.toString() << ", Expected: " << m_peerAddress.toString();
        return;
    }

    // 나머지 코드는 동일...
    QDataStream in(data);
    QString header;
    in >> header;

    qDebug() << "  Header: " << header;

    if (header == "READY") {
        QString opponentAddressStr;
        quint16 opponentPort = 0;
        in >> opponentAddressStr >> opponentPort;;
        qDebug() << "  Received READY from opponent address: " << opponentAddressStr;

        m_opponentReady = true;
        // Host determination: The one with the lexicographically smaller IP address string is the host
        if (m_localPort > opponentPort) {
                m_amIHost = true;
            } else if (m_localPort < opponentPort) {
                m_amIHost = false;
            } else {
                // 포트가 우연히 같다면(같은 PC에서 잘못 입력 등), IP 문자열 보조판단
                m_amIHost = (m_myAddress.toString() > opponentAddressStr);
            }

        setStatus("Opponent is ready!");
        qDebug() << "  Received READY. m_isReady=" << m_isReady << ", m_opponentReady=" << m_opponentReady << ", m_amIHost=" << m_amIHost;
        checkReadyState();
    }else if (header == "CONNECT") {
        QString peerIp; quint16 peerLocalPort = 0;
        in >> peerIp >> peerLocalPort;
        qDebug() << "  Received CONNECT from" << peerIp << ":" << peerLocalPort;

        // 상대에게 ACK
        QByteArray ack;
        QDataStream out(&ack, QIODevice::WriteOnly);
        out << QString("CONNECT_ACK");
        emit sendDatagram(ack, m_peerAddress, m_peerPort);

        m_peerConnected = true;
        setStatus("Peer connected. You can click Ready.");
        ui->connect_button->setEnabled(true); // Ready 버튼 활성화
        return;
    }
    else if (header == "CONNECT_ACK") {
        qDebug() << "  Received CONNECT_ACK.";
        m_peerConnected = true;
        setStatus("Peer connected. You can click Ready.");
        ui->connect_button->setEnabled(true); // Ready 버튼 활성화
        return;
    }

    else if (header == "REMATCH_VOTE"|| header == "REMATCH_REQ") {
        qDebug() << "  Received REMATCH_VOTE.";
        emit opponentVotedForRematch(); // New signal
    } else if (header == "QUIT_GAME") {
        qDebug() << "  Received QUIT_GAME.";
        emit opponentQuit();
    }
}

void HomeScreen::checkReadyState() {
    qDebug() << "checkReadyState: m_isReady=" << m_isReady << ", m_opponentReady=" << m_opponentReady;
    if (m_isReady && m_opponentReady) {
        setStatus("Both players ready! Starting game...");
        m_readyCheckTimer->stop(); // Stop sending READY periodically
        qDebug() << "checkReadyState: Emitting startGame(m_amIHost=" << m_amIHost << ")";
        emit startGame(m_amIHost);
    }
}

void HomeScreen::onQuitClicked() {
    qDebug() << "onQuitClicked: Quitting application.";
    resetUI();
    // 상대방에게 QUIT_GAME 메시지 전송
    QByteArray dat;
    QDataStream out(&dat, QIODevice::WriteOnly);
    out << QString("QUIT_GAME");
    if (!m_peerAddress.isNull() && m_peerPort != 0) {
        emit sendDatagram(dat, m_peerAddress, m_peerPort);
    }
    QApplication::quit();
}

void HomeScreen::onRematchClicked() {
    qDebug() << "onRematchClicked: Rematch vote (emit rematchRequested).";
    setRematchStatus("Waiting for opponent's vote...");
    enableRematchButtons(false);
    emit rematchRequested();  // ← MainWindow가 m_myRematchVote 세팅 + REMATCH_VOTE 전송
}

void HomeScreen::onExitToHomeClicked() {
    qDebug() << "onExitToHomeClicked: Sending QUIT_GAME.";
    QByteArray dat;
    QDataStream out(&dat, QIODevice::WriteOnly);
    out << QString("QUIT_GAME");
    emit sendDatagram(dat, m_peerAddress, m_peerPort);
    resetUI();
}
