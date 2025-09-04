#include <iostream>

class Monster;

class Player {
public:
    int HP, MP, x, y;

    Player();
    Player(int x, int y);

    void Attack(Monster &target);
    void Show_Status() const;
    void X_move(int move);
    void Y_move(int move);
};

class Monster {
public:
    int HP, x, y;

    Monster();
    Monster(int x, int y, int HP);

    int Be_Attacked(int damage); // 남은 HP 반환
};