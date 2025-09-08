#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QtWidgets>
#include <random>
#include <queue>
#include <limits>
#include <cmath>
#include <algorithm>

static inline QRectF cellRect(int x,int y,int c){ return QRectF(x*c, y*c, c, c); }

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow) {
    ui->setupUi(this);

    // 두 장면을 각 뷰에 연결
    sceneAst = new QGraphicsScene(this);
    sceneDij = new QGraphicsScene(this);
    ui->Astar_map->setScene(sceneAst);
    ui->Dijkstra_map->setScene(sceneDij);
    ui->Astar_map->setRenderHint(QPainter::Antialiasing,false);
    ui->Dijkstra_map->setRenderHint(QPainter::Antialiasing,false);

    // 클릭 처리(두 뷰 공통)
    ui->Astar_map->viewport()->installEventFilter(this);
    ui->Dijkstra_map->viewport()->installEventFilter(this);
    ui->Astar_map->setMouseTracking(true);
    ui->Dijkstra_map->setMouseTracking(true);

    // 타이머 연결
    connect(&animAst, &QTimer::timeout, this, &MainWindow::tickAstar);
    connect(&animDij, &QTimer::timeout, this, &MainWindow::tickDijkstra);

    // 초기값
    ui->obstacle_percentage->setRange(0,100);
    ui->obstacle_percentage->setValue(30);
    ui->spinBox->setRange(5,100);
    ui->spinBox->setValue(grid.w);

    sx=0; sy=0; gx=grid.w-1; gy=grid.h-1;

    redrawBoth();
}

MainWindow::~MainWindow(){ delete ui; }

// ===== 페이지 전환 =====
void MainWindow::on_Map_Editing_clicked(){
    ui->stackedWidget->setCurrentWidget(ui->Map_Editing_Page);
}
void MainWindow::on_Map_Setting_clicked(){
    ui->stackedWidget->setCurrentWidget(ui->Map_Setting_Page);
    tool_ = Tool::Start;
    statusBar()->showMessage("시작점 셀을 클릭하세요");
}
void MainWindow::on_Path_Search_clicked(){
    ui->stackedWidget->setCurrentWidget(ui->Path_Search_Page);
}

// ===== Map Editing =====
void MainWindow::on_generate_map_editing_page_clicked(){
    const int pct = ui->obstacle_percentage->value();
    std::mt19937 rng(std::random_device{}());
    std::uniform_int_distribution<> d(0,99);

    std::fill(grid.obs.begin(), grid.obs.end(), 0);
    for(int y=0;y<grid.h;y++)
        for(int x=0;x<grid.w;x++)
            if(d(rng)<pct) grid.obs[grid.id(x,y)] = 1;

    if(grid.in(sx,sy)) grid.obs[grid.id(sx,sy)] = 0;
    if(grid.in(gx,gy)) grid.obs[grid.id(gx,gy)] = 0;

    redrawBoth();
}
void MainWindow::on_claer_map_editing_page_clicked(){
    std::fill(grid.obs.begin(), grid.obs.end(), 0);
    redrawBoth();
}

// ===== Map Setting =====
void MainWindow::on_apply_map_setting_page_clicked(){
    int n = ui->spinBox->value();     // n x n
    grid = Grid(n,n);
    sx=0; sy=0; gx=n-1; gy=n-1;
    redrawBoth();
}
void MainWindow::on_setStartBtn_clicked(){
    tool_ = Tool::Start; statusBar()->showMessage("시작점 셀을 클릭");
}
void MainWindow::on_setGoalBtn_clicked(){
    tool_ = Tool::Goal;  statusBar()->showMessage("도착점 셀을 클릭");
}

// ===== 공통 그리기 =====
void MainWindow::redrawBoth(){ drawOne(sceneAst); drawOne(sceneDij); }
void MainWindow::drawOne(QGraphicsScene* sc){
    sc->clear();
    sc->setSceneRect(0,0, grid.w*cell, grid.h*cell);
    for(int y=0;y<grid.h;y++)
        for(int x=0;x<grid.w;x++){
            QColor fill = grid.blocked(x,y)? QColor(40,40,40) : QColor(255,255,255);
            sc->addRect(cellRect(x,y,cell), QPen(Qt::lightGray), QBrush(fill));
        }
    if(grid.in(sx,sy)) sc->addRect(cellRect(sx,sy,cell), QPen(Qt::green,2));
    if(grid.in(gx,gy)) sc->addRect(cellRect(gx,gy,cell), QPen(Qt::red,2));
}
void MainWindow::drawVisit(QGraphicsScene* sc,int x,int y){
    auto r = sc->addRect(cellRect(x,y,cell), QPen(Qt::NoPen), QBrush(QColor(180,220,255)));
    r->setZValue(1);
}
void MainWindow::drawPathCell(QGraphicsScene* sc,int x,int y){
    auto r = sc->addRect(cellRect(x,y,cell), QPen(Qt::NoPen), QBrush(QColor(255,220,120)));
    r->setZValue(2);
}

// ===== 클릭 처리 =====
bool MainWindow::eventFilter(QObject* obj, QEvent* ev){
    if(ev->type()!=QEvent::MouseButtonPress) return false;

    QGraphicsView* view=nullptr;
    if(obj==ui->Astar_map->viewport())          view=ui->Astar_map;
    else if(obj==ui->Dijkstra_map->viewport())  view=ui->Dijkstra_map;
    else return false;

    auto* me = static_cast<QMouseEvent*>(ev);
    QPointF sp = view->mapToScene(me->pos());
    int x = int(sp.x())/cell, y = int(sp.y())/cell;
    if(!grid.in(x,y)) return true;

    // Map_Setting에서만 시작/도착 설정
    if(ui->stackedWidget->currentWidget()==ui->Map_Setting_Page){
        if(grid.blocked(x,y)) return true;
        if(tool_==Tool::Start){
            sx=x; sy=y;
            if(grid.in(sx,sy)) grid.obs[grid.id(sx,sy)]=0;
            tool_ = Tool::Goal;
            statusBar()->showMessage("도착점 셀을 클릭하세요");
        }else if(tool_==Tool::Goal){
            gx=x; gy=y;
            if(grid.in(gx,gy)) grid.obs[grid.id(gx,gy)]=0;
            tool_ = Tool::None;
            statusBar()->showMessage("시작/도착 설정 완료");
        }
        redrawBoth();
        return true;
    }
    return false;
}

// ===== Start/Stop =====
void MainWindow::on_Start_clicked(){
    if(!grid.in(sx,sy) || !grid.in(gx,gy) || grid.blocked(sx,sy) || grid.blocked(gx,gy)){
        statusBar()->showMessage("시작/도착 좌표가 유효하지 않습니다.");
        return;
    }
    animAst.stop(); animDij.stop();
    stepsAst.clear(); idxAst=0;
    stepsDij.clear(); idxDij=0;

    runAstar();
    runDijkstra();

    drawOne(sceneAst);
    drawOne(sceneDij);

    animAst.start(15);
    animDij.start(15);
}
void MainWindow::on_Stop_clicked(){
    animAst.stop();
    animDij.stop();
}

// ===== Dijkstra =====
void MainWindow::runDijkstra(){
    const int W=grid.w, H=grid.h;
    auto id=[&](int x,int y){ return y*W + x; };
    const int N=W*H, INF=std::numeric_limits<int>::max()/4;

    std::vector<int> dist(N, INF), px(N,-1), py(N,-1);

    struct Node{ int f,g,x,y; };
    struct Cmp{ bool operator()(const Node& a,const Node& b) const { return a.f>b.f; } };

    std::priority_queue<Node, std::vector<Node>, Cmp> pq;
    dist[id(sx,sy)] = 0;
    pq.push({0,0,sx,sy}); // f=g

    const int dx[4]={1,-1,0,0}, dy[4]={0,0,1,-1};

    while(!pq.empty()){
        Node cur = pq.top(); pq.pop();
        int cx=cur.x, cy=cur.y, gcost=cur.g;
        if(gcost!=dist[id(cx,cy)]) continue;

        stepsDij.push_back(SearchStep{SearchStep::Visit,cx,cy});

        if(cx==gx && cy==gy){
            std::vector<std::pair<int,int>> path;
            int tx=cx, ty=cy;
            while(!(tx==sx && ty==sy)){
                path.emplace_back(tx,ty);
                int ii=id(tx,ty);
                int pxv=px[ii], pyv=py[ii];
                tx=pxv; ty=pyv;
            }
            path.emplace_back(sx,sy);
            std::reverse(path.begin(), path.end());
            for(auto &p: path) stepsDij.push_back(SearchStep{SearchStep::Path,p.first,p.second});
            return;
        }

        for(int k=0;k<4;k++){
            int nx=cx+dx[k], ny=cy+dy[k];
            if(nx<0||nx>=W||ny<0||ny>=H) continue;
            if(grid.blocked(nx,ny)) continue;
            int ng=gcost+1;
            int iid=id(nx,ny);
            if(ng<dist[iid]){
                dist[iid]=ng; px[iid]=cx; py[iid]=cy;
                pq.push({ng,ng,nx,ny}); // f=g
            }
        }
    }
    statusBar()->showMessage("Dijkstra 경로 없음");
}
void MainWindow::tickDijkstra(){
    if(idxDij >= (int)stepsDij.size()){ animDij.stop(); return; }
    auto s = stepsDij[idxDij++];
    if(s.kind==SearchStep::Visit) drawVisit(sceneDij, s.x, s.y);
    else                          drawPathCell(sceneDij, s.x, s.y);
}

// ===== A* =====
static inline int manhattan(int x,int y,int tx,int ty){
    return std::abs(x-tx) + std::abs(y-ty);
}
void MainWindow::runAstar(){
    const int W=grid.w, H=grid.h;
    auto id=[&](int x,int y){ return y*W + x; };
    const int N=W*H, INF=std::numeric_limits<int>::max()/4;

    std::vector<int> dist(N, INF), px(N,-1), py(N,-1);

    struct Node{ int f,g,x,y; };
    struct Cmp{ bool operator()(const Node& a,const Node& b) const { return a.f>b.f; } };

    std::priority_queue<Node, std::vector<Node>, Cmp> pq;
    dist[id(sx,sy)] = 0;
    pq.push({manhattan(sx,sy,gx,gy),0,sx,sy});

    const int dx[4]={1,-1,0,0}, dy[4]={0,0,1,-1};

    while(!pq.empty()){
        Node cur = pq.top(); pq.pop();
        int cx=cur.x, cy=cur.y, gcost=cur.g;
        if(gcost!=dist[id(cx,cy)]) continue;

        stepsAst.push_back(SearchStep{SearchStep::Visit,cx,cy});

        if(cx==gx && cy==gy){
            std::vector<std::pair<int,int>> path;
            int tx=cx, ty=cy;
            while(!(tx==sx && ty==sy)){
                path.emplace_back(tx,ty);
                int ii=id(tx,ty);
                int pxv=px[ii], pyv=py[ii];
                tx=pxv; ty=pyv;
            }
            path.emplace_back(sx,sy);
            std::reverse(path.begin(), path.end());
            for(auto &p: path) stepsAst.push_back(SearchStep{SearchStep::Path,p.first,p.second});
            return;
        }

        for(int k=0;k<4;k++){
            int nx=cx+dx[k], ny=cy+dy[k];
            if(nx<0||nx>=W||ny<0||ny>=H) continue;
            if(grid.blocked(nx,ny)) continue;
            int ng=gcost+1;
            int iid=id(nx,ny);
            if(ng<dist[iid]){
                dist[iid]=ng; px[iid]=cx; py[iid]=cy;
                int f = ng + manhattan(nx,ny,gx,gy);
                pq.push({f,ng,nx,ny});
            }
        }
    }
    statusBar()->showMessage("A* 경로 없음");
}
void MainWindow::tickAstar(){
    if(idxAst >= (int)stepsAst.size()){ animAst.stop(); return; }
    auto s = stepsAst[idxAst++];
    if(s.kind==SearchStep::Visit) drawVisit(sceneAst, s.x, s.y);
    else                          drawPathCell(sceneAst, s.x, s.y);
}
