#include "cpp_test1/hw_01.hpp"
#include <iostream>
using namespace std;

// 생성자
ArrayStats::ArrayStats(int n) {
    size = n;
    arr = new int[size];
}

// 소멸자
ArrayStats::~ArrayStats() {
    delete[] arr;
}

// 입력
void ArrayStats::input() {
    for (int i = 0; i < size; i++) {
        while (true) {
            std::string temp;
            std::cout << i + 1 << "번째 정수 입력: ";
            std::cin >> temp;

            bool is_number = true;
            int start = 0;

            // 음수 허용 → 맨 앞에 '-' 있으면 건너뜀
            if (temp[0] == '-') {
                if (temp.size() == 1) is_number = false; // "-"만 입력된 경우
                start = 1;
            }

            for (int j = start; j < (int)temp.size(); j++) {
                if (temp[j] < '0' || temp[j] > '9') {
                    is_number = false;
                    break;
                }
            }

            if (is_number) {
                // 문자열 → 정수 변환 직접 구현
                int value = 0;
                for (int j = start; j < (int)temp.size(); j++) {
                    value = value * 10 + (temp[j] - '0');
                }
                if (temp[0] == '-') value = -value;

                arr[i] = value;
                break;
            } else {
                std::cout << "정수가 아닙니다. 다시 입력하세요.\n";
            }
        }
    }
}

// 최대값
int ArrayStats::getMax() {
    int maxVal = arr[0];
    for (int i = 1; i < size; i++)
        if (arr[i] > maxVal) maxVal = arr[i];
    return maxVal;
}

// 최소값
int ArrayStats::getMin() {
    int minVal = arr[0];
    for (int i = 1; i < size; i++)
        if (arr[i] < minVal) minVal = arr[i];
    return minVal;
}

// 합
int ArrayStats::getSum() {
    int sum = 0;
    for (int i = 0; i < size; i++) sum += arr[i];
    return sum;
}

// 평균
double ArrayStats::getAverage() {
    return static_cast<double>(getSum()) / size;
}
int main() {
    int n;
    while (true) {
        cout << "몇 개의 원소를 할당하겠습니까? : ";
        cin >> n;
        if (n <= 0) {
            cout << "개수는 양수입니다. 다시 입력하세요.\n";
        } else break;
    }

    ArrayStats stats(n);
    stats.input();

    cout << "최대값: " << stats.getMax() << endl;
    cout << "최소값: " << stats.getMin() << endl;
    cout << "전체합: " << stats.getSum() << endl;
    cout << "평균: " << stats.getAverage() << endl;

    return 0;
}
