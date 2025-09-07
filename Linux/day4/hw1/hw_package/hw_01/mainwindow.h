#pragma once
class MainWindow;                // 전방선언

#include <QObject>               // 추가
#include <QMainWindow>
#include <QVector>
#include <QPushButton>
#include <QTextEdit>
#include <QTimer>
#include <QElapsedTimer>
#include <QEvent>
#include <QString>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class QPushButton;
class QTextEdit;

class MultiTapHandler : public QObject {
    Q_OBJECT
public:
    MultiTapHandler(QPushButton* btn, QTextEdit* out,
                    QChar tap1, QChar tap2, QChar longPress,
                    MainWindow* owner, QObject* parent=nullptr);

protected:
    bool eventFilter(QObject* obj, QEvent* ev) override;
private slots:
    void onTimeout();

private:
    QPushButton* btn_;
    QTextEdit* out_;
    QChar c1_, c2_, clong_;
    QTimer timer_;
    QElapsedTimer press_;
    int count_ = 0;
    MainWindow* owner_;

};
// --- 천지인 모음 합성기 ---
class VowelComposer {
public:
    // 진행 중 모음이 없으면 false
    bool hasPending() const { return !pending_.isEmpty(); }

    // 각 키 입력 처리. 반환값: 출력할 글자와 치환 여부
    struct Out { bool replaceLast; QChar ch; bool valid() const { return ch != QChar(); } };

    Out pressSunI();   // ㅣ
    Out pressDotA();   // ㆍ
    Out pressFloorU(); // ㅡ
    void reset() { pending_.clear(); }

private:
    // pending_은 'I','D','F'의 시퀀스(ㅣ,ㆍ,ㅡ)
    QString pending_;

    Out commitSingle(QChar v);                   // 새 모음 시작
    Out commitCombine(const QString& seq);       // 조합 결과 반환
    QString map2(const QString& seq) const;        // 2타 조합
};

// MainWindow 멤버에 추가


class MainWindow : public QMainWindow {
    Q_OBJECT
private:
    VowelComposer vowel_;
public:
    explicit MainWindow(QWidget *parent=nullptr);
    ~MainWindow();
    void resetVowel(){ vowel_.reset(); }
    void clearKorDotState(){ lastInsertedByDotComKor = false; }
    void insertJamo(QChar j);   // 자모 하나를 받아 음절로 합성해 넣기

private:
    Ui::MainWindow *ui;
    QVector<MultiTapHandler*> handlers;
    int dotComQueExcCountEng = 0;
    int dotComQueExcCountKor = 0;
    bool lastInsertedByDotComEng = false;
    bool lastInsertedByDotComKor = false;
    bool engCaps = false;
    QString lastT9Key;
    QElapsedTimer engTapTimer;
    int lastT9Index = -1;
    static constexpr int ENG_TAP_MS = 700;

    void handleT9(const QString& keyId,
                  const QStringList& lower, const QStringList& upper);
    // 소멸은 Qt가 parent로 정리
private slots:
    void on_space_kor_clicked();
    void on_enter_kor_clicked();
    void on_delete_kor_clicked();
    void on_dot_com_que_exc_eng_clicked();
    void on_dot_com_que_exc_kor_clicked();
    void on_kor_eng_change_kor_clicked();
    void on_kor_eng_change_eng_clicked();
    void on_shift_capslock_clicked();

};
