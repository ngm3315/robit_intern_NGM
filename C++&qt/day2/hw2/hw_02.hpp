#include <vector>
#include <utility>
using namespace std;

struct Point {
    int x;
    int y;
};

class PointSet {
private:
    vector<Point> points;
    pair<Point, Point> minPair;
    pair<Point, Point> maxPair;
    double minDist;
    double maxDist;

    double distance(const Point& a, const Point& b);

public:
    PointSet(int n, int rangeX, int rangeY);
    void computeDistances();
    void printResults();
};