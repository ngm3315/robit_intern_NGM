#include "ui_mainwindow.h"
#include "mainwindow.h"
#include <QtWidgets>
#include <random>

static inline QRectF cellRect(int x,int y,int cell){ return QRectF(x*cell, y*cell, cell, cell); }

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow) {
    ui->setupUi(this);

    // 두 장면을 각각의 뷰에 연결
    sceneAst = new QGraphicsScene(this);
    sceneDij = new QGraphicsScene(this);
    ui->Astar_map->setScene(sceneAst);
    ui->Dijkstra_map->setScene(sceneDij);
    ui->Astar_map->setRenderHint(QPainter::Antialiasing,false);
    ui->Dijkstra_map->setRenderHint(QPainter::Antialiasing,false);

    // 위젯 초기값
    ui->obstacle_percentage->setRange(0,100);
    ui->obstacle_percentage->setValue(30);
    ui->spinBox->setRange(5,100);
    ui->spinBox->setValue(grid.w);  // 정사각형 크기

    redrawBoth();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_Map_Editing_clicked(){
    ui->stackedWidget->setCurrentWidget(ui->Map_Editing_Page);
}
void MainWindow::on_Map_Setting_clicked(){
    ui->stackedWidget->setCurrentWidget(ui->Map_Setting_Page);
}
void MainWindow::on_Path_Search_clicked(){
    ui->stackedWidget->setCurrentWidget(ui->Path_Search_Page);
}



void MainWindow::on_claer_map_editing_page_clicked(){
    std::fill(grid.obs.begin(), grid.obs.end(), 0);
    redrawBoth();
}


void MainWindow::on_generate_map_editing_page_clicked(){
    const int pct = ui->obstacle_percentage->value(); // 0~100
    std::mt19937 rng(std::random_device{}());
    std::uniform_int_distribution<> d(0,99);

    std::fill(grid.obs.begin(), grid.obs.end(), 0);
    for(int y=0;y<grid.h;y++){
        for(int x=0;x<grid.w;x++){
            if(d(rng) < pct) grid.obs[grid.id(x,y)] = 1;
        }
    }
    redrawBoth(); // A*와 Dijkstra 동시 반영
}


void MainWindow::on_obstacle_percentage_textChanged(const QString &arg1)
{

}







void MainWindow::on_Start_clicked()
{

}


void MainWindow::on_Stop_clicked()
{

}


void MainWindow::on_Astar_map_rubberBandChanged(const QRect &viewportRect, const QPointF &fromScenePoint, const QPointF &toScenePoint)
{

}


void MainWindow::on_Dijkstra_map_rubberBandChanged(const QRect &viewportRect, const QPointF &fromScenePoint, const QPointF &toScenePoint)
{

}
void MainWindow::redrawBoth(){
    drawOne(sceneAst);
    drawOne(sceneDij);
}
void MainWindow::drawOne(QGraphicsScene* sc){
    sc->clear();
    sc->setSceneRect(0,0, grid.w*cell, grid.h*cell);
    for(int y=0;y<grid.h;y++){
        for(int x=0;x<grid.w;x++){
            const bool blk = grid.blocked(x,y);
            QColor fill = blk ? QColor(40,40,40) : QColor(255,255,255);
            sc->addRect(cellRect(x,y,cell), QPen(Qt::lightGray), QBrush(fill));
        }
    }
}

void MainWindow::on_apply_map_setting_page_clicked(){
    int n = ui->spinBox->value();   // n x n
    grid = Grid(n,n);               // 맵 재할당(장애물 초기화)
    redrawBoth();
}

