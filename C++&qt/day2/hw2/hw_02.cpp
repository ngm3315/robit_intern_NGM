#include "cpp_test1/hw_02.hpp"
#include <iostream>
#include <cmath>
using namespace std;

PointCalc::PointCalc(int n, int min, int max) {
    size = n;
    points = new Point[size];
      // 한 번만 호출하면 됨
    for (int i = 0; i < size; i++) {
        
        points[i].x = min + rand() % (max - min + 1);
        points[i].y = min + rand() % (max - min + 1);
    }
}
PointCalc::~PointCalc() {
    delete[] points;
}
void PointCalc::printPoints() {
    for (int i = 0; i < size; i++) {
        cout << "점" << i+1 << " = (" << points[i].x << "," << points[i].y << ")\n";
    }
}


void PointCalc::Pointsdistance_min() {
    if (size < 2) return;
    int dx1 = points[0].x - points[1].x;
    int dy1 = points[0].y - points[1].y;
    int d_origin = dx1 * dx1 + dy1 * dy1;
    int min_d2 = d_origin;
    Point A{}, B{};
    for (int i = 0; i < size; i++) {
        for (int j = i + 1; j < size; j++) {
            int dx = points[i].x - points[j].x;
            int dy = points[i].y - points[j].y;
            int d2 = dx * dx + dy * dy;
            if (d2 < min_d2) {
                min_d2 = d2;
                A = points[i];
                B = points[j];
            }
        }
    }
    cout << "최소 거리 = " << sqrt((double)min_d2)
         << " ((" << A.x << "," << A.y << ") - ("
         << B.x << "," << B.y << "))\n";
}

void PointCalc::Pointsdistance_max() {
    if (size < 2) return;
    int max_d2 = -1;
    Point A{}, B{};
    for (int i = 0; i < size; i++) {
        for (int j = i + 1; j < size; j++) {
            int dx = points[i].x - points[j].x;
            int dy = points[i].y - points[j].y;
            int d2 = dx * dx + dy * dy;
            if (d2 > max_d2) {
                max_d2 = d2;
                A = points[i];
                B = points[j];
            }
        }
    }
    cout << "최대 거리 = " << sqrt((double)max_d2)
         << " ((" << A.x << "," << A.y << ") - ("
         << B.x << "," << B.y << "))\n";
}
int main() {
    srand(static_cast<unsigned int>(time(NULL))); 
    int n, min, max;
    while(1){
        std::cout << "점 개수 ,최소값, 최대값 입력(소수를 입력해도 정수부분만 인식합니다.): ";
        std::cin >> n >> min >> max;

        if (n < 1) {
            std::cout << "N은 1 이상이어야 함\n";
            continue;
        }
        
        else if (min >max){
            std::cout<<"최소값이 최대값의 값보다 클수 없습니다\n";
        }else break;
    }

    PointCalc pc(n, min, max);
    pc.printPoints();
    pc.Pointsdistance_min();
    pc.Pointsdistance_max();

    return 0;
}
