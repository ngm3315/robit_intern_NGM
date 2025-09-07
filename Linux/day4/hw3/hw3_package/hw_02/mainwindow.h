#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QGraphicsScene>
#include <vector>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

// ★ 누락돼 있던 Grid 정의 추가
struct Grid {
    int w=30, h=30;
    std::vector<unsigned char> obs;          // 0 free, 1 blocked
    Grid(int W=30,int H=30):w(W),h(H),obs(W*H,0){}
    bool in(int x,int y) const { return 0<=x && x<w && 0<=y && y<h; }
    bool blocked(int x,int y) const { return obs[y*w+x]; }
    int id(int x,int y) const { return y*w+x; }
};

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent=nullptr);
    ~MainWindow();

private slots:
    void on_Map_Editing_clicked();
    void on_Map_Setting_clicked();
    void on_Path_Search_clicked();

    // ★ 오타 수정: claer -> clear
    void on_claer_map_editing_page_clicked();
    void on_generate_map_editing_page_clicked();
    void on_obstacle_percentage_textChanged(const QString &arg1);

    void on_Start_clicked();
    void on_Stop_clicked();
    void on_Astar_map_rubberBandChanged(const QRect &viewportRect,
                                        const QPointF &fromScenePoint,
                                        const QPointF &toScenePoint);
    void on_Dijkstra_map_rubberBandChanged(const QRect &viewportRect,
                                           const QPointF &fromScenePoint,
                                           const QPointF &toScenePoint);

    void on_apply_map_setting_page_clicked();

private:
    Ui::MainWindow *ui=nullptr;

    // 공용 맵(두 뷰 동기화)
    Grid grid{30,30};

    // 두 장면
    QGraphicsScene *sceneAst=nullptr, *sceneDij=nullptr;
    int cell=22;

    void redrawBoth();
    void drawOne(QGraphicsScene* sc);
};
#endif // MAINWINDOW_H
