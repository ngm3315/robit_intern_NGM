#pragma once

#include <QMainWindow>
#include <QGraphicsScene>
#include <QGraphicsRectItem>
#include <QGraphicsEllipseItem>
#include <QTimer>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    // 슬라이더
    void on_toque1_slide_valueChanged(int v);
    void on_toque2_slide_valueChanged(int v);
    void on_toque3_slide_valueChanged(int v);
    void on_all_toque_slide_valueChanged(int v);

    // 자동 회전 버튼
    void on_A_T1_C_clicked();
    void on_A_T1_C_R_clicked();
    void on_A_T2_C_clicked();
    void on_A_T2_C_R_clicked();
    void on_A_T3_C_clicked();
    void on_A_T3_C_R_clicked();
    void on_A_all_C_clicked();
    void on_A_all_C_R_clicked();

    // 초기화 / 저장 / 불러오기
    void on_init_clicked();
    void on_save_to_txt_clicked();
    void on_load_prev_clicked();

private:
    Ui::MainWindow *ui;
    QGraphicsScene *scene;

    // 링크와 조인트
    QGraphicsRectItem *link1{nullptr};
    QGraphicsRectItem *link2{nullptr};
    QGraphicsRectItem *link3{nullptr};
    QGraphicsEllipseItem *j1{nullptr};
    QGraphicsEllipseItem *j2{nullptr};
    QGraphicsEllipseItem *j3{nullptr};

    // 링크 길이
    const int L1 = 120;
    const int L2 = 100;
    const int L3 = 80;

    // 자동 회전
    QTimer timer;
    int dir[3]{0,0,0};
    const int step = 2;

    // 내부 함수
    void buildArm();
    void setAngles(int a1,int a2,int a3);
    void updateTimer();
    void toggleDir(int idx,int want);
    void tick();
};
