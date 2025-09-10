#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QHostAddress>
#include <QCloseEvent>
#include <QStackedWidget>
#include <QUdpSocket>
#include <QTimer>
#include <QDataStream>
#include <QMessageBox>
#include <QDebug>
class QStackedWidget;
class HomeScreen;
class GameWidget;
class QUdpSocket;
class QTimer;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    QUdpSocket *m_udpSocket;

protected:
    void closeEvent(QCloseEvent *event) override; // Override closeEvent

private slots:
    void handleGameStart(bool isHost);
    void handleGameOver(const QString& result);
    void handleCountdown();
    void handleDatagramReadyRead();
    void sendDatagram(const QByteArray& data); // For GameWidget
    void sendDatagram(const QByteArray& data, QHostAddress peerAddress, quint16 peerPort); // For HomeScreen
    void handleRematchRequest();
    
    void handleOpponentQuit();
    void handleBindSocket(quint16 localPort);
    void handleOpponentRematchVote();
    void Reset();

private:
    void applyStyle();
    QStackedWidget *m_stackedWidget;
    HomeScreen *m_homeScreen;
    GameWidget *m_gameWidget;

    QTimer *m_countdownTimer;
    int m_countdownValue;

    QHostAddress m_peerAddress; // Store peer address for GameWidget communication
    quint16 m_peerPort; // Store peer port for GameWidget communication
    QHostAddress m_myLocalAddress; // New: Store my local IP address

    bool m_myRematchVote;
    bool m_opponentRematchVote;
    bool m_connected;
};

#endif // MAINWINDOW_H
