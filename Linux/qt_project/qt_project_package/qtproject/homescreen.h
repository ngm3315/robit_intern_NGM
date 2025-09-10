#pragma once

#include <QWidget>
#include <QMessageBox>
#include <QHostAddress>
#include <QApplication>
#include <QDataStream>
#include <QDebug>
#include <QTimer>
QT_BEGIN_NAMESPACE
namespace Ui { class HomeScreen; }
QT_END_NAMESPACE

class QUdpSocket;
class QTimer; // Added for m_readyCheckTimer

class HomeScreen : public QWidget {
    Q_OBJECT

public:
    explicit HomeScreen(QWidget *parent = nullptr);
    ~HomeScreen();

    void setStatus(const QString& status, bool isError = false);
    void resetUI();
    void showCountdown(int count);
    void showResultPage(const QString& result);
    void showConnectionPage();
    void setRematchStatus(const QString& status);
    void enableRematchButtons(bool enable);

    // Public getters for data needed by MainWindow
    QHostAddress getPeerAddress() const { return m_peerAddress; }
    quint16 getPeerPort() const { return m_peerPort; }
    quint16 getLocalPort() const { return m_localPort; }
    bool isHost() const { return m_amIHost; }
    void setMyAddress(const QHostAddress& addr); // New: Setter for my local IP

signals:
    void startGame(bool isHost);
    void sendDatagram(const QByteArray& data, QHostAddress peerAddress, quint16 peerPort);
    void rematchRequested();
    void opponentVotedForRematch();
     // Signal when opponent accepts rematch
    void opponentQuit();
    void Reset();
    void bindSocket(quint16 localPort); // Signal when opponent quits from post-game

public slots:
    void onDatagramReceived(const QByteArray& data, QHostAddress sender, quint16 senderPort, bool isConnected);

private slots:
    void onConnectClicked();
    void onQuitClicked();
    void onRematchClicked();
    void onExitToHomeClicked();
    void sendReadyStatus(); // New slot for periodic ready check
    void on_reset_botton_clicked();
    void onInputChanged(); // Slot to check input fields

private:
    void checkReadyState();

    Ui::HomeScreen *ui;
    bool m_isReady = false;
    bool m_opponentReady = false;
    bool m_amIHost = false;
    
    bool m_peerConnected = false;

    QHostAddress m_peerAddress;
    quint16 m_peerPort;
    quint16 m_localPort;
    QTimer *m_readyCheckTimer; // New timer for periodic ready check
    QHostAddress m_myAddress; // New: Store my local IP address
};
