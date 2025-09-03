#include <iostream>
#include <cpp_test1/hw_02.hpp>


#include <cmath>
#include <cstdlib>
#include <ctime>
using namespace std;

PointSet::PointSet(int n, int rangeX, int rangeY) {
    srand((unsigned)time(NULL));
    for (int i = 0; i < n; i++) {
        Point p;
        p.x = rand() % (rangeX + 1);
        p.y = rand() % (rangeY + 1);
        points.push_back(p);
    }
    minDist = 1e9;
    maxDist = -1.0;
}

double PointSet::distance(const Point& a, const Point& b) {
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

void PointSet::computeDistances() {
    for (size_t i = 0; i < points.size(); i++) {
        for (size_t j = i + 1; j < points.size(); j++) {
            double d = distance(points[i], points[j]);
            if (d < minDist) {
                minDist = d;
                minPair = {points[i], points[j]};
            }
            if (d > maxDist) {
                maxDist = d;
                maxPair = {points[i], points[j]};
            }
        }
    }
}

void PointSet::printResults() {
    cout << "최소 거리 = " << minDist << " ("
         << "(" << minPair.first.x << "," << minPair.first.y << ")"
         << " - "
         << "(" << minPair.second.x << "," << minPair.second.y << ")"
         << ")" << endl;

    cout << "최대 거리 = " << maxDist << " ("
         << "(" << maxPair.first.x << "," << maxPair.first.y << ")"
         << " - "
         << "(" << maxPair.second.x << "," << maxPair.second.y << ")"
         << ")" << endl;
}
int main() {
    int n, rangeX, rangeY;
    cout << "점 개수 입력: ";
    cin >> n;
    cout << "X 좌표 범위 입력: ";
    cin >> rangeX;
    cout << "Y 좌표 범위 입력: ";
    cin >> rangeY;

    PointSet ps(n, rangeX, rangeY);
    ps.computeDistances();
    ps.printResults();

    return 0;
}