#include "mainwindow.h"
#include <QMessageBox>
#include <QVBoxLayout>   // 추가
#include <QStatusBar>

// ===== BottleItem =====
BottleItem::BottleItem(MainWindow* owner, int id, const QRectF& r)
    : QGraphicsRectItem(r), owner_(owner), id_(id) {
    setFlag(QGraphicsItem::ItemIsSelectable, true);
    setPen(QPen(Qt::black, 2));
    setBrush(Qt::NoBrush);
}

bool BottleItem::push(const QColor& c) {
    if (!canPush()) return false;
    balls_.push_back(c);
    redraw();
    return true;
}

bool BottleItem::pop(QColor& out) {
    if (!canPop()) return false;
    out = balls_.back();
    balls_.pop_back();
    redraw();
    return true;
}

bool BottleItem::isUniformFull() const {
    if (balls_.size() != CAP) return false;
    return balls_[0] == balls_[1] && balls_[1] == balls_[2];
}

void BottleItem::redraw() {
    // remove previous ball drawings
    const auto kids = childItems();
    for (auto* ch : kids) delete ch;

    const QRectF r = rect();
    const qreal margin = 10;
    const qreal innerW = r.width() - 2*margin;
    const qreal ballH  = (r.height() - 2*margin) / CAP;

    for (int i = 0; i < balls_.size(); ++i) {
        int levelFromBottom = i; // 0 bottom
        qreal y = r.bottom() - margin - (levelFromBottom+1)*ballH;
        qreal x = r.left() + margin;
        auto* e = new QGraphicsEllipseItem(x, y, innerW, ballH-4, this);
        e->setBrush(QBrush(balls_[i]));
        e->setPen(QPen(Qt::black, 1));
        e->setZValue(1);
    }
    update();
}

void BottleItem::mousePressEvent(QGraphicsSceneMouseEvent* ev) {
    QGraphicsRectItem::mousePressEvent(ev);
    if (owner_) owner_->onBottleClicked(this);
}

// ===== MainWindow =====
MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent),
    view_(new QGraphicsView(this)),
    scene_(new QGraphicsScene(this)) {

    setWindowTitle("Bottle Sort - 3 Bottles");
    resize(540, 360);

    view_->setScene(scene_);
    view_->setRenderHint(QPainter::Antialiasing, true);
    view_->setAlignment(Qt::AlignCenter);

    auto* central = new QWidget(this);
    auto* lay = new QVBoxLayout(central);
    lay->setContentsMargins(0,0,0,0);
    lay->addWidget(view_);
    setCentralWidget(central);

    statusBar()->showMessage("소스 병을 클릭하세요");
    setupScene();
    initLevel();
}

void MainWindow::setupScene() {
    scene_->setSceneRect(0,0,520,300);

    const qreal W = 120;
    const qreal H = 220;
    const qreal gap = 40;
    const qreal startX = 20;
    const qreal y = 40;

    for (int i=0;i<3;++i) {
        QRectF r(startX + i*(W+gap), y, W, H);
        bottles_[i] = new BottleItem(this, i, r);
        scene_->addItem(bottles_[i]);

        // base decoration under bottle
        QRectF base(r.left()-2, r.bottom()+2, r.width()+4, 8);
        scene_->addRect(base, QPen(Qt::NoPen), QBrush(Qt::lightGray));
    }
}

void MainWindow::initLevel() {
    // clear
    for (auto* b : bottles_) {
        if (!b) continue;
        QColor dummy;
        while (b->pop(dummy)) {}
        b->setPen(QPen(Qt::black, 2));
    }
    // Bottle 0: (빨/파/빨) bottom->top
    bottles_[0]->push(Qt::red);
    bottles_[0]->push(Qt::blue);
    bottles_[0]->push(Qt::red);

    // Bottle 1: (파/파/빨) bottom->top
    bottles_[1]->push(Qt::blue);
    bottles_[1]->push(Qt::blue);
    bottles_[1]->push(Qt::red);

    // Bottle 2: empty
    selected_ = nullptr;
    statusBar()->showMessage("소스 병을 클릭하세요");
}

void MainWindow::onBottleClicked(BottleItem* b) {
    if (!selected_) {
        selected_ = b;
        statusBar()->showMessage(QString("타깃 병을 클릭하세요 (선택: %1)").arg(b->id()+1));
        b->setPen(QPen(Qt::darkGreen, 4));
        return;
    }
    if (b == selected_) {
        selected_->setPen(QPen(Qt::black, 2));
        selected_ = nullptr;
        statusBar()->showMessage("소스 병을 클릭하세요");
        return;
    }

    QColor top;
    if (!selected_->canPop()) {
        selected_->setPen(QPen(Qt::black, 2));
        selected_ = nullptr;
        statusBar()->showMessage("빈 병에서 이동 불가. 소스 병을 다시 선택.");
        return;
    }
    if (!b->canPush()) {
        selected_->setPen(QPen(Qt::black, 2));
        selected_ = nullptr;
        statusBar()->showMessage("타깃 병이 가득 참. 다른 타깃을 선택.");
        return;
    }

    selected_->pop(top);
    b->push(top);

    selected_->setPen(QPen(Qt::black, 2));
    selected_ = nullptr;
    statusBar()->showMessage("소스 병을 클릭하세요");

    checkWin();
}

void MainWindow::checkWin() {
    int uniformFull = 0;
    for (auto* b : bottles_) if (b->isUniformFull()) ++uniformFull;
    if (uniformFull >= 2) {
        QMessageBox::information(this, "승리", "두 병이 같은 색으로 3개씩 채워졌습니다. 게임 종료.");
        initLevel();
    }
}
