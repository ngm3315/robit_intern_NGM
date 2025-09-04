#include "cpp_test1/hw_03.hpp"
using namespace std;

static const int ATTACK_POWER = 10;

Player::Player() : HP(50), MP(10), x(0), y(0) {}
Player::Player(int x_, int y_) : HP(50), MP(10), x(x_), y(y_) {}

void Player::Attack(Monster &target) {
    if (MP <= 0) {
        cout << "MP 부족!\n";
        exit(0);
    }
    MP -= 1;
    int remain = target.Be_Attacked(ATTACK_POWER);
    cout << "공격 성공! 몬스터 HP:" << remain << "\n";
    if (remain <= 0) {
        cout << "몬스터를 처치했습니다. 프로그램 종료.\n";
        exit(0);
    }
}

void Player::Show_Status() const {
    cout << "HP:" << HP << "\nMP:" << MP << "\nPosition:" << x << "," << y << "\n";
}

void Player::X_move(int move) { x += move; cout << "X Position moved! Now " << x << "\n"; }
void Player::Y_move(int move) { y += move; cout << "Y Position moved! Now " << y << "\n"; }

Monster::Monster() : HP(30), x(0), y(0) {}
Monster::Monster(int x_, int y_, int HP_) : HP(HP_), x(x_), y(y_) {}

int Monster::Be_Attacked(int damage) {
    HP -= damage;
    if (HP < 0) HP = 0;
    return HP;
}

int main() {
    Player player(0, 0);
    Monster monster(5, 4, 50);

    cout << "Type Command(A/U/D/L/R/S)\n";
    char c;
    while (cin >> c) {
        switch (c) {
            case 'A': case 'a':
                player.Attack(monster);
                break;
            case 'U': case 'u':
                player.Y_move(-1);
                break;
            case 'D': case 'd':
                player.Y_move(1);
                break;
            case 'L': case 'l':
                player.X_move(-1);
                break;
            case 'R': case 'r':
                player.X_move(1);
                break;
            case 'S': case 's':
                player.Show_Status();
                break;
            default:
                cout << "잘못된 명령\n";
        }
        cout << "Type Command(A/U/D/L/R/S)\n";
    }
    return 0;
}
