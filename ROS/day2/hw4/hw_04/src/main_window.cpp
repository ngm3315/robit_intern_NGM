#include "hw_04/main_window.hpp"
#include "ui_mainwindow.h"
#include <QGraphicsView>
#include <QFile>
#include <QTextStream>

static inline int clamp360(int v){ return qBound(0, v, 360); }

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow) {
    ui->setupUi(this);
    setWindowTitle("3-DOF Robot Arm");

    // QGraphicsView 초기화
    scene = new QGraphicsScene(this);
    ui->graphicsView->setScene(scene);
    ui->graphicsView->setRenderHint(QPainter::Antialiasing, true);
    scene->setSceneRect(-300,-300,600,600);
    ui->graphicsView->centerOn(0,0);

    buildArm();

    // 슬라이더 범위
    if(ui->toque1_slide) ui->toque1_slide->setRange(0,360);
    if(ui->toque2_slide) ui->toque2_slide->setRange(0,360);
    if(ui->toque3_slide) ui->toque3_slide->setRange(0,360);
    if(ui->all_toque_slide) ui->all_toque_slide->setRange(0,360);

    // 초기값
    setAngles(90,90,90);
    if(ui->toque1_slide) ui->toque1_slide->setValue(90);
    if(ui->toque2_slide) ui->toque2_slide->setValue(90);
    if(ui->toque3_slide) ui->toque3_slide->setValue(90);
    if(ui->all_toque_slide) ui->all_toque_slide->setValue(90);

    // 타이머
    timer.setInterval(30);
    connect(&timer,&QTimer::timeout,this,&MainWindow::tick);
}

MainWindow::~MainWindow(){ delete ui; }

void MainWindow::buildArm() {
    QPen pen(Qt::black,2); QBrush brush(Qt::lightGray);

    // 링크1
    link1 = new QGraphicsRectItem(0,-5,L1,10);
    link1->setPen(pen); link1->setBrush(brush);
    link1->setTransformOriginPoint(0,0);
    link1->setPos(0,0);
    scene->addItem(link1);

    // 링크2
    link2 = new QGraphicsRectItem(0,-5,L2,10, link1);
    link2->setPen(pen); link2->setBrush(brush);
    link2->setTransformOriginPoint(0,0);
    link2->setPos(L1,0);

    // 링크3
    link3 = new QGraphicsRectItem(0,-5,L3,10, link2);
    link3->setPen(pen); link3->setBrush(brush);
    link3->setTransformOriginPoint(0,0);
    link3->setPos(L2,0);

    // 조인트
    const double r=6;
    j1=new QGraphicsEllipseItem(-r,-r,2*r,2*r,link1); j1->setBrush(Qt::darkGray); j1->setPos(0,0);
    j2=new QGraphicsEllipseItem(-r,-r,2*r,2*r,link2); j2->setBrush(Qt::darkGray); j2->setPos(0,0);
    j3=new QGraphicsEllipseItem(-r,-r,2*r,2*r,link3); j3->setBrush(Qt::darkGray); j3->setPos(0,0);
}

void MainWindow::setAngles(int a1,int a2,int a3) {
    if(link1) link1->setRotation(clamp360(a1));
    if(link2) link2->setRotation(clamp360(a2));
    if(link3) link3->setRotation(clamp360(a3));
    ui->graphicsView->centerOn(0,0);
}

// ===== 슬라이더 =====
void MainWindow::on_toque1_slide_valueChanged(int v) {
    int a2 = link2 ? int(link2->rotation()) : 0;
    int a3 = link3 ? int(link3->rotation()) : 0;
    setAngles(v, a2, a3);
}
void MainWindow::on_toque2_slide_valueChanged(int v) {
    int a1 = link1 ? int(link1->rotation()) : 0;
    int a3 = link3 ? int(link3->rotation()) : 0;
    setAngles(a1, v, a3);
}
void MainWindow::on_toque3_slide_valueChanged(int v) {
    int a1 = link1 ? int(link1->rotation()) : 0;
    int a2 = link2 ? int(link2->rotation()) : 0;
    setAngles(a1, a2, v);
}
void MainWindow::on_all_toque_slide_valueChanged(int v) {
    bool b1 = ui->toque1_slide->blockSignals(true);
    bool b2 = ui->toque2_slide->blockSignals(true);
    bool b3 = ui->toque3_slide->blockSignals(true);
    if (ui->toque1_slide) ui->toque1_slide->setValue(v);
    if (ui->toque2_slide) ui->toque2_slide->setValue(v);
    if (ui->toque3_slide) ui->toque3_slide->setValue(v);
    ui->toque1_slide->blockSignals(b1);
    ui->toque2_slide->blockSignals(b2);
    ui->toque3_slide->blockSignals(b3);
    setAngles(v, v, v);
}

// ===== 자동 회전 =====
void MainWindow::updateTimer() {
    if (dir[0]==0 && dir[1]==0 && dir[2]==0) timer.stop();
    else timer.start();
}
void MainWindow::toggleDir(int idx, int want) {
    if (idx < 0 || idx > 2) return;
    dir[idx] = (dir[idx]==want) ? 0 : want;
    updateTimer();
}
void MainWindow::tick() {
    auto wrap360 = [](int x){
        x %= 360; if (x < 0) x += 360;
        return x;
    };
    int a1 = link1 ? int(link1->rotation()) : 0;
    int a2 = link2 ? int(link2->rotation()) : 0;
    int a3 = link3 ? int(link3->rotation()) : 0;

    a1 = wrap360(a1 + dir[0]*step);
    a2 = wrap360(a2 + dir[1]*step);
    a3 = wrap360(a3 + dir[2]*step);

    setAngles(a1, a2, a3);

    bool b1 = ui->toque1_slide->blockSignals(true);
    bool b2 = ui->toque2_slide->blockSignals(true);
    bool b3 = ui->toque3_slide->blockSignals(true);
    if (ui->toque1_slide) ui->toque1_slide->setValue(a1);
    if (ui->toque2_slide) ui->toque2_slide->setValue(a2);
    if (ui->toque3_slide) ui->toque3_slide->setValue(a3);
    ui->toque1_slide->blockSignals(b1);
    ui->toque2_slide->blockSignals(b2);
    ui->toque3_slide->blockSignals(b3);

    updateTimer();
}

// ===== 버튼 핸들러 =====
void MainWindow::on_A_T1_C_clicked()   { toggleDir(0, +1); }
void MainWindow::on_A_T1_C_R_clicked() { toggleDir(0, -1); }
void MainWindow::on_A_T2_C_clicked()   { toggleDir(1, +1); }
void MainWindow::on_A_T2_C_R_clicked() { toggleDir(1, -1); }
void MainWindow::on_A_T3_C_clicked()   { toggleDir(2, +1); }
void MainWindow::on_A_T3_C_R_clicked() { toggleDir(2, -1); }

void MainWindow::on_A_all_C_clicked() {
    bool allCW = (dir[0]==+1 && dir[1]==+1 && dir[2]==+1);
    dir[0]=dir[1]=dir[2]= allCW ? 0 : +1;
    updateTimer();
}
void MainWindow::on_A_all_C_R_clicked() {
    bool allCCW = (dir[0]==-1 && dir[1]==-1 && dir[2]==-1);
    dir[0]=dir[1]=dir[2]= allCCW ? 0 : -1;
    updateTimer();
}

// ===== init / save / load =====
void MainWindow::on_init_clicked() {
    const int initAng = 90;
    bool b1 = ui->toque1_slide->blockSignals(true);
    bool b2 = ui->toque2_slide->blockSignals(true);
    bool b3 = ui->toque3_slide->blockSignals(true);
    if (ui->toque1_slide) ui->toque1_slide->setValue(initAng);
    if (ui->toque2_slide) ui->toque2_slide->setValue(initAng);
    if (ui->toque3_slide) ui->toque3_slide->setValue(initAng);
    if (ui->all_toque_slide) ui->all_toque_slide->setValue(initAng);
    ui->toque1_slide->blockSignals(b1);
    ui->toque2_slide->blockSignals(b2);
    ui->toque3_slide->blockSignals(b3);

    dir[0]=dir[1]=dir[2]=0;
    setAngles(initAng, initAng, initAng);
    updateTimer();
}

void MainWindow::on_save_to_txt_clicked() {
    QFile f("arm_state.txt");
    if (!f.open(QIODevice::WriteOnly | QIODevice::Text)) return;
    QTextStream out(&f);
    const int a1 = link1 ? int(link1->rotation()) : 0;
    const int a2 = link2 ? int(link2->rotation()) : 0;
    const int a3 = link3 ? int(link3->rotation()) : 0;
    out << a1 << '\n' << a2 << '\n' << a3 << '\n';
}

void MainWindow::on_load_prev_clicked() {
    QFile f("arm_state.txt");
    if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) return;
    QTextStream in(&f);
    int a1=90, a2=90, a3=90;
    in >> a1 >> a2 >> a3;

    bool b1 = ui->toque1_slide->blockSignals(true);
    bool b2 = ui->toque2_slide->blockSignals(true);
    bool b3 = ui->toque3_slide->blockSignals(true);
    if (ui->toque1_slide) ui->toque1_slide->setValue(clamp360(a1));
    if (ui->toque2_slide) ui->toque2_slide->setValue(clamp360(a2));
    if (ui->toque3_slide) ui->toque3_slide->setValue(clamp360(a3));
    ui->toque1_slide->blockSignals(b1);
    ui->toque2_slide->blockSignals(b2);
    ui->toque3_slide->blockSignals(b3);

    setAngles(clamp360(a1), clamp360(a2), clamp360(a3));
}
