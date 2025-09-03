class ArrayStats {
private:
    int* arr;
    int size;
public:
    ArrayStats(int n);
    ~ArrayStats();
    void input();
    int getMax();
    int getMin();
    int getSum();
    double getAverage();
};