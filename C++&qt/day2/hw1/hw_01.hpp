class ArrayStats {
private:
    int* arr;   // 정수 배열로 변경
    int size;
public:
    ArrayStats(int n);    // 동적 할당
    ~ArrayStats();        // 메모리 해제
    void input();         // 입력 (문자열 검사 후 정수 변환)
    int getMax();         // 최대값
    int getMin();         // 최소값
    int getSum();         // 합
    double getAverage();  // 평균
};
