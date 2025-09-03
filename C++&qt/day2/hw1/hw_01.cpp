#include "cpp_test1/hw_01.hpp"
#include <iostream>
using namespace std;

ArrayStats::ArrayStats(int n) {
    size = n;
    arr = new int[size];
}

ArrayStats::~ArrayStats() {
    delete[] arr;
}

void ArrayStats::input() {
    for(int i=0;i<size;i++) {
        cout << i+1 << "번째 데이터 입력: ";
        cin >> arr[i];
    }
}

int ArrayStats::getMax() {
    int maxVal = arr[0];
    for(int i=1;i<size;i++)
        if(arr[i] > maxVal) maxVal = arr[i];
    return maxVal;
}

int ArrayStats::getMin() {
    int minVal = arr[0];
    for(int i=1;i<size;i++)
        if(arr[i] < minVal) minVal = arr[i];
    return minVal;
}

int ArrayStats::getSum() {
    int sum = 0;
    for(int i=0;i<size;i++) sum += arr[i];
    return sum;
}

double ArrayStats::getAverage() {
    return static_cast<double>(getSum()) / size;
}

int main() {
    int n;
    while(1){
        cout << "몇 개의 원소를 할당하겠습니까? : ";
        cin >> n;
        if(n<0){
            std::cout<<"개수는 양수입니다 다시 입력하세요"<<std::endl;

        }
        else break;
    }
    

    ArrayStats stats(n);
    stats.input();

    cout << "최대값: " << stats.getMax() << endl;
    cout << "최소값: " << stats.getMin() << endl;
    cout << "전체합: " << stats.getSum() << endl;
    cout << "평균: " << stats.getAverage() << endl;

    return 0;
}