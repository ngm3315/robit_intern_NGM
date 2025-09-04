struct Point {
    int x;
    int y;
};

class PointCalc {
private:
    Point* points;   // 동적 배열
    int size;
    int *distance;
public:
    PointCalc(int n, int min, int max);
    ~PointCalc();
    void printPoints();
   
    void Pointsdistance_max();
    void Pointsdistance_min();
};
