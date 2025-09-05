#pragma once
#include <QMainWindow>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsRectItem>
#include <QGraphicsEllipseItem>
#include <QGraphicsSceneMouseEvent>
#include <QPen>
#include <QBrush>
#include <QColor>
#include <QVector>

class MainWindow; // fwd

class BottleItem : public QGraphicsRectItem {
public:
    static constexpr int CAP = 3;
    BottleItem(MainWindow* owner, int id, const QRectF& r);

    int id() const { return id_; }
    bool canPush() const { return balls_.size() < CAP; }
    bool canPop() const { return !balls_.isEmpty(); }
    bool isUniformFull() const;
    bool push(const QColor& c);
    bool pop(QColor& out);
    void redraw();
protected:
    void mousePressEvent(QGraphicsSceneMouseEvent* ev) override;
private:
    MainWindow* owner_;
    int id_;
    QVector<QColor> balls_; // back == top
};

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(QWidget* parent=nullptr);
    void onBottleClicked(BottleItem* b);
private:
    void setupScene();
    void initLevel();
    void checkWin();

    QGraphicsView* view_;
    QGraphicsScene* scene_;
    BottleItem* bottles_[3]{nullptr,nullptr,nullptr};
    BottleItem* selected_{nullptr};
};
