#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QGraphicsScene>
#include <QEvent>
#include <QTimer>
#include <vector>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

struct Grid {
    int w=30, h=30;
    std::vector<unsigned char> obs;
    Grid(int W=30,int H=30):w(W),h(H),obs(W*H,0){}
    bool in(int x,int y) const { return 0<=x && x<w && 0<=y && y<h; }
    bool blocked(int x,int y) const { return obs[y*w+x]; }
    int  id(int x,int y) const { return y*w+x; }
};

// ★ 한 번만 선언
struct SearchStep { enum Kind{ Visit, Path } kind; int x, y; };

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent=nullptr);
    ~MainWindow();

private slots:
    // 페이지 전환
    void on_Map_Editing_clicked();
    void on_Map_Setting_clicked();
    void on_Path_Search_clicked();
    // Map Editing
    void on_generate_map_editing_page_clicked();
    void on_claer_map_editing_page_clicked();
    // Map Setting
    void on_apply_map_setting_page_clicked();
    void on_setStartBtn_clicked();
    void on_setGoalBtn_clicked();
    // 실행/중지 버튼
    void on_Start_clicked();
    void on_Stop_clicked();

protected:
    bool eventFilter(QObject* obj, QEvent* ev) override;

private:
    Ui::MainWindow *ui=nullptr;

    // 상태
    Grid grid{30,30};
    int sx=0, sy=0, gx=29, gy=29;
    enum class Tool { None, Start, Goal } tool_=Tool::None;

    // 장면
    QGraphicsScene *sceneAst=nullptr, *sceneDij=nullptr;
    int cell=22;

    // A* 애니메이션
    QTimer animAst; std::vector<SearchStep> stepsAst; int idxAst=0;
    // Dijkstra 애니메이션
    QTimer animDij; std::vector<SearchStep> stepsDij; int idxDij=0;

    // 그리기/탐색
    void redrawBoth();
    void drawOne(QGraphicsScene* sc);
    void drawVisit(QGraphicsScene* sc,int x,int y);
    void drawPathCell(QGraphicsScene* sc,int x,int y);
    void runAstar();
    void runDijkstra();
    void tickAstar();
    void tickDijkstra();
};
#endif
