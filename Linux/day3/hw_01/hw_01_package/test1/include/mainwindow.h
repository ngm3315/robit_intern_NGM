#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <QMainWindow>
#include <QTimer>
#include <QGraphicsScene>
#include <QGraphicsRectItem>
#include <QGraphicsEllipseItem>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent=nullptr);
    ~MainWindow();

private slots:
    // 슬라이더
    void on_toque1_slide_valueChanged(int);
    void on_toque2_slide_valueChanged(int);
    void on_toque3_slide_valueChanged(int);
    void on_all_toque_slide_valueChanged(int);

    // 버튼(자동 회전 토글)
    void on_A_T1_C_clicked();   void on_A_T1_C_R_clicked();
    void on_A_T2_C_clicked();   void on_A_T2_C_R_clicked();
    void on_A_T3_C_clicked();   void on_A_T3_C_R_clicked();
    void on_A_all_C_clicked();  void on_A_all_C_R_clicked();

    // 초기화/저장/불러오기
    void on_init_clicked();
    void on_save_to_txt_clicked();
    void on_load_prev_clicked();

private:
    Ui::MainWindow *ui;

    // 그래픽
    QGraphicsScene *scene = nullptr;
    const double L1=120, L2=90, L3=70;
    QGraphicsRectItem *link1=nullptr,*link2=nullptr,*link3=nullptr;
    QGraphicsEllipseItem *j1=nullptr,*j2=nullptr,*j3=nullptr;

    // 자동 회전
    QTimer timer;
    int dir[3]={0,0,0};   // -1 CCW, 0 stop, +1 CW
    int step=1;           // 틱당 변화량(도)

    // 내부 함수
    void buildArm();
    void setAngles(int a1,int a2,int a3);   // 0~360
    void tick();
    void toggleDir(int idx,int want);
    void updateTimer();
};
#endif
