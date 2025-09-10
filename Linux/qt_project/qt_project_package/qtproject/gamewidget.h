#ifndef GAMEWIDGET_H
#define GAMEWIDGET_H

#include <QWidget>
#include <QRectF>
#include <QGraphicsScene>
#include <QGraphicsRectItem>
#include <QGraphicsEllipseItem>
#include <QGraphicsTextItem>
#include <QKeyEvent>
#include <QTimer>
#include <QFont>
#include <QtMath>
#include <QRandomGenerator>
#include <QVector>
#include <QDataStream>
#include <QDebug>

QT_BEGIN_NAMESPACE
namespace Ui { class GameWidget; }
QT_END_NAMESPACE

class QGraphicsScene;
class QGraphicsRectItem;
class QGraphicsEllipseItem;
class QGraphicsTextItem;
class QTimer;
class QKeyEvent;

class GameWidget : public QWidget
{
    Q_OBJECT

public:
    GameWidget(QWidget *parent = nullptr);
    ~GameWidget();

    void startGame(bool isHost);

signals:
    void gameOver(const QString& result);
    void sendDatagram(const QByteArray& data);

public slots:
    void onDatagramReceived(const QByteArray& data);

protected:
    void keyPressEvent(QKeyEvent *event) override;
    void keyReleaseEvent(QKeyEvent *event) override;

private slots:
    void updateAnimation();
    void spawnHealthItem();
    void spawnSawItem();
    void normalizeBall();
    void relocateItems();

private:
    void resetGame();
    void endGame(const QString& winner);
    void sendPaddlePosition();
    void sendGameState(); // Host only
    void updateScoreDisplay(); // Added back
    void updateHealthDisplay(); // Added back

    Ui::GameWidget *ui;
    QGraphicsScene *m_scene;
    QGraphicsEllipseItem *m_paddle1;
    QGraphicsEllipseItem *m_paddle2;
    QGraphicsEllipseItem *m_ellipseItem;
    QGraphicsRectItem *m_goal1;
    QGraphicsRectItem *m_goal2;
    QGraphicsTextItem *m_player1ScoreText;
    QGraphicsTextItem *m_player2ScoreText;
    QGraphicsTextItem *m_gameOverText;
    QTimer *m_animationTimer;
    QTimer *m_itemRelocationTimer;

    qreal m_ellipseSpeedX;
    qreal m_ellipseSpeedY;

    bool m_isMovingUp1, m_isMovingDown1, m_isMovingLeft1 ,m_isMovingRight1 ;
    bool m_isMovingUp2, m_isMovingDown2, m_isMovingLeft2 ,m_isMovingRight2 ;

    int m_player1Score;
    int m_player2Score;
    int m_player1Health;
    int m_player2Health;

    QVector<QGraphicsRectItem*> m_player1HealthBars;
    QVector<QGraphicsRectItem*> m_player2HealthBars;

    QGraphicsEllipseItem* m_healthItem = nullptr;
    QGraphicsEllipseItem* m_sawItem = nullptr;
    int m_lastPlayerToHitBall = 0;
    bool m_isSawBladeBall = false;
    QRectF m_leftItemSpawnArea;
    QRectF m_rightItemSpawnArea;
    QRectF m_SpawnArea;

    bool m_isHost = false;
};

#endif // GAMEWIDGET_H
