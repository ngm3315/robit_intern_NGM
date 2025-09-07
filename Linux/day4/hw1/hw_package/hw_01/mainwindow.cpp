// src/mainwindow.cpp
#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QPushButton>
#include <QTextEdit>
#include <QTimer>
#include <QElapsedTimer>
#include <QEvent>
// ----- VowelComposer 구현 (교체) -----
VowelComposer::Out VowelComposer::commitSingle(QChar v){
    // 단일 시작: pending_ 유지해서 다음 타 입력 시 치환 가능
    return {false, v};
}
QString VowelComposer::map2(const QString& s) const {
    if (s=="ID") return QString(u"ㅏ");
    if (s=="DI") return QString(u"ㅓ");
    if (s=="DF") return QString(u"ㅗ");
    if (s=="FD") return QString(u"ㅜ");
    if (s=="DD") return QString::fromUtf16(u"··", 2);  // 'ㆍ' 두 개
    return QString();
}


void MainWindow::on_kor_eng_change_kor_clicked() {
    // stackedWidget의 페이지를 pageEn으로 전환
    ui->stackedWidget->setCurrentWidget(ui->pageEn);
}
void MainWindow::on_kor_eng_change_eng_clicked() {
    // stackedWidget의 페이지를 pageEn으로 전환
    ui->stackedWidget->setCurrentWidget(ui->pageKo);
}
VowelComposer::Out VowelComposer::commitCombine(const QString& seq){
    // 1타 즉시 출력, 2타는 치환 출력, 일부 3타 확장
    if (seq.size()==1){
        if (seq=="I") { pending_="I"; return commitSingle(u'ㅣ'); }
        if (seq=="F") { pending_="F"; return commitSingle(u'ㅡ'); }
        if (seq=="D") { pending_="D"; return commitSingle(u'ㆍ'); }
    }
    if (seq.size()==2){
        if (QString v = map2(seq); !v.isEmpty()) {
            pending_ = seq.right(1);
            return {true, v.at(0)};   // Out은 여전히 QChar 하나만 지원
        }

    }
    // 3타 파생
    if (seq=="IDI") { pending_.clear(); return {true, u'ㅐ'}; } // ㅏ+ㅣ
    if (seq=="DII") { pending_.clear(); return {true, u'ㅔ'}; } // ㅓ+ㅣ
    if (seq=="DFI") { pending_.clear(); return {true, u'ㅚ'}; } // ㅗ+ㅣ
    if (seq=="FDI") { pending_.clear(); return {true, u'ㅟ'}; } // ㅜ+ㅣ
    if (seq=="FI")  { pending_.clear(); return {true, u'ㅢ'}; } // ㅡ+ㅣ
    if (seq=="DDF")  { pending_.clear(); return {true, u'ㅛ'}; }

    // 매칭 실패 → 마지막 키 단독 확정
    QChar last;
    if (!seq.isEmpty()){
        if (seq.back()=='I') last=u'ㅣ';
        if (seq.back()=='F') last=u'ㅡ';
        if (seq.back()=='D') last=u'ㆍ';
    }
    pending_.clear();
    return {false, last};
}
VowelComposer::Out VowelComposer::pressSunI(){
    auto s = pending_.isEmpty() ? QString("I") : pending_ + "I";
    return commitCombine(s);
}
VowelComposer::Out VowelComposer::pressDotA(){
    auto s = pending_.isEmpty() ? QString("D") : pending_ + "D";
    return commitCombine(s);
}
VowelComposer::Out VowelComposer::pressFloorU(){
    auto s = pending_.isEmpty() ? QString("F") : pending_ + "F";
    return commitCombine(s);
}



static constexpr int LONGPRESS_MS  = 280;
static constexpr int MULTITAP_MS   = 270;

MultiTapHandler::MultiTapHandler(QPushButton* btn, QTextEdit* out,
                                 QChar tap1, QChar tap2, QChar longPress,
                                 MainWindow* owner, QObject* parent)
    : QObject(parent), btn_(btn), out_(out),
    c1_(tap1), c2_(tap2), clong_(longPress), owner_(owner)
{
    timer_.setSingleShot(true);
    timer_.setInterval(MULTITAP_MS);  // 멀티탭 확정 타이머
    connect(&timer_, &QTimer::timeout, this, &MultiTapHandler::onTimeout);
    btn_->installEventFilter(this);
}


bool MultiTapHandler::eventFilter(QObject* obj, QEvent* ev) {
    if (obj != btn_) return QObject::eventFilter(obj, ev);

    if (ev->type() == QEvent::MouseButtonPress) {
        press_.start();
        return true;
    }
    if (ev->type() == QEvent::MouseButtonRelease) {
        // 길게
        if (press_.elapsed() >= LONGPRESS_MS) {
            if (owner_) owner_->insertJamo(clong_.isNull() ? c1_ : clong_);
            count_ = 0; timer_.stop();
            if (owner_) { owner_->resetVowel(); owner_->clearKorDotState(); }
        }
         else {
            ++count_;
            if (!timer_.isActive()) timer_.start();
        }
        return true;
    }
    return QObject::eventFilter(obj, ev);
}

void MultiTapHandler::onTimeout() {
    // 멀티탭 확정
    if (owner_) owner_->insertJamo(count_ == 1 ? c1_ : c2_);
    count_ = 0;
    if (owner_) { owner_->resetVowel(); owner_->clearKorDotState(); }

}

void MainWindow::handleT9(const QString& keyId,
                          const QStringList& lower, const QStringList& upper)
{
    const bool sameKey = (lastT9Key == keyId) &&
                         engTapTimer.isValid() &&
                         engTapTimer.elapsed() < ENG_TAP_MS;

    if (!sameKey) lastT9Index = -1;
    lastT9Index = (lastT9Index + 1) % lower.size();

    QString ch = engCaps ? upper.at(lastT9Index) : lower.at(lastT9Index);

    QTextCursor cur = ui->textEdit->textCursor();
    if (sameKey && !cur.atStart()) cur.deletePreviousChar(); // 치환
    cur.insertText(ch);
    ui->textEdit->setTextCursor(cur);

    lastT9Key = keyId;
    engTapTimer.restart();

    // 다른 특수버튼 연속치환 방지
    lastInsertedByDotComEng = false;
}

void MainWindow::on_shift_capslock_clicked() {
    engCaps = !engCaps;
    lastT9Key.clear();          // 새 사이클 시작
}
void MainWindow::insertJamo(QChar j)
{
    // 한글 자모 → 초중종 인덱스 매핑
    auto Lindex = [](QChar c)->int{
        static const QHash<QChar,int> m = {
            {u'ㄱ',0},{u'ㄲ',1},{u'ㄴ',2},{u'ㄷ',3},{u'ㄸ',4},{u'ㄹ',5},{u'ㅁ',6},{u'ㅂ',7},{u'ㅃ',8},
            {u'ㅅ',9},{u'ㅆ',10},{u'ㅇ',11},{u'ㅈ',12},{u'ㅉ',13},{u'ㅊ',14},{u'ㅋ',15},{u'ㅌ',16},{u'ㅍ',17},{u'ㅎ',18}
        }; return m.value(c,-1);
    };
    auto Vindex = [](QChar c)->int{
        static const QHash<QChar,int> m = {
            {u'ㅏ',0},{u'ㅐ',1},{u'ㅑ',2},{u'ㅒ',3},{u'ㅓ',4},{u'ㅔ',5},{u'ㅕ',6},{u'ㅖ',7},
            {u'ㅗ',8},{u'ㅘ',9},{u'ㅙ',10},{u'ㅚ',11},{u'ㅛ',12},{u'ㅜ',13},{u'ㅝ',14},{u'ㅞ',15},{u'ㅟ',16},
            {u'ㅠ',17},{u'ㅡ',18},{u'ㅢ',19},{u'ㅣ',20}
        }; return m.value(c,-1);
    };
    auto Tindex = [](QChar c)->int{               // 종성(받침) 인덱스 (0=없음)
        static const QHash<QChar,int> m = {
            {u'\0',0},{u'ㄱ',1},{u'ㄲ',2},{u'ㄳ',3},{u'ㄴ',4},{u'ㄵ',5},{u'ㄶ',6},{u'ㄷ',7},{u'ㄹ',8},
            {u'ㄺ',9},{u'ㄻ',10},{u'ㄼ',11},{u'ㄽ',12},{u'ㄾ',13},{u'ㄿ',14},{u'ㅀ',15},{u'ㅁ',16},{u'ㅂ',17},
            {u'ㅄ',18},{u'ㅅ',19},{u'ㅆ',20},{u'ㅇ',21},{u'ㅈ',22},{u'ㅊ',23},{u'ㅋ',24},{u'ㅌ',25},{u'ㅍ',26},{u'ㅎ',27}
        }; return m.value(c,-1);
    };
    auto make = [](int L,int V,int T)->QChar{
        int code = 0xAC00 + (L*21 + V)*28 + T;
        return QChar(code);
    };
    auto isSyll = [](QChar s)->bool{ return s.unicode()>=0xAC00 && s.unicode()<=0xD7A3; };
    auto split = [&](QChar s, int& L,int& V,int& T){
        int S = s.unicode()-0xAC00;
        L = S/588; V = (S%588)/28; T = S%28;
    };

    QTextCursor cur = ui->textEdit->textCursor();

    // 커서 앞 글자 조사
    QChar prev = QChar();
    if (!cur.atStart()) {
        QTextCursor peek = cur; peek.movePosition(QTextCursor::Left, QTextCursor::KeepAnchor, 1);
        prev = peek.selectedText().isEmpty()?QChar():peek.selectedText().at(0);
    }

    const int Li = Lindex(j), Vi = Vindex(j), Ti = Tindex(j);

    // 1) 앞에 음절이 없거나 영문/숫자/기타면: 규칙 시작
    if (!isSyll(prev)) {
        if (Li>=0) { cur.insertText(QString(j)); }                         // 초성만 입력. 다음 모음 때 합성
        else if (Vi>=0) { cur.insertText(QString(j)); }                    // 모음 단독
        else { cur.insertText(QString(j)); }                               // 기타
        ui->textEdit->setTextCursor(cur);
        return;
    }

    // 2) 앞 글자가 한글 음절인 경우 합성 규칙
    int L,V,T; split(prev, L,V,T);

    if (Vi>=0) {
        // a) 직전이 초성만 단독으로 찍혀 있었던 경우(실제론 초성+모음 없는 음절은 없음)
        // b) 직전 음절에 모음이 없을 수 없으므로: 종성이 0이면 초성+모음 치환
        if (T==0) {
            // prev는 이미 L+V 형태. 새 모음이면 새 음절 시작이 맞지만,
            // 직전 문자가 "초성 글자(ㄱ 등)로 따로 찍혀있던 경우" 처리 위해 이전 글자가 자모인지도 체크해야 한다.
            // 간단화: 직전이 자모 단독이었으면 그냥 합치고, 아니면 새 음절 시작.
            // 여기서는 가장 흔한 케이스 처리: 직전이 자음 글자였으면 합성
            // 커서 왼쪽 1글자가 자음 자모(ㄱ~ㅎ)이면
            if (Lindex(prev)!=-1) { // 자모 단독
                cur.deletePreviousChar();
                cur.insertText(QString(make(Lindex(prev), Vi, 0)));
            } else {
                cur.insertText(QString(j));
            }
        } else {
            // c) 받침이 있고 모음이 오면: 받침을 다음 음절의 초성으로 이동
            // 단일 받침만 지원(겹받침 분해는 단순화)
            QTextCursor left = cur;
            left.deletePreviousChar();                 // prev 제거
            int moveL = 0;
            // 겹받침 분해 간단화: ㄳ→ㄱ 남기고 ㅅ 이동 등은 생략. 단일 받침만 이동.
            if (Tindex(QChar())==0 && false) {}       // placeholder
            if (T > 0) {
                // 단일 받침 가정: prev = L,V,T → 남기는 음절 L,V,0
                QChar keep = make(L,V,0);
                left.insertText(QString(keep));
                moveL = (T==1)?0  : (T==2)?1  : (T==7)?3  : (T==8)?5  :
                                       (T==16)?6 : (T==17)?7 : (T==19)?9 : (T==21)?11:
                                       (T==22)?12: (T==23)?14: (T==24)?15: (T==25)?16:
                                       (T==26)?17: (T==27)?18 : -1; // 종성→초성 매핑(단일만)
            }
            if (moveL>=0) cur.insertText(QString(make(moveL, Vi, 0)));
            else          cur.insertText(QString(j));               // 매핑 실패시 모음만
        }
        ui->textEdit->setTextCursor(cur);
        return;
    }

    if (Li>=0) {
        // 자음이 왔을 때: 직전 음절의 받침으로 넣기 시도
        if (T==0) {
            int t = Tindex(j);
            if (t>0) {
                cur.deletePreviousChar();
                cur.insertText(QString(make(L,V,t)));
            } else {
                cur.insertText(QString(j)); // 받침 불가 자모는 그대로
            }
        } else {
            // 이미 받침이 있으면 새 자음 시작
            cur.insertText(QString(j));
        }
        ui->textEdit->setTextCursor(cur);
        return;
    }

    // 그 밖의 문자
    cur.insertText(QString(j));
    ui->textEdit->setTextCursor(cur);
}


// ===== MainWindow =====
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // 자음 키 배치: (단탭1, 단탭2, 길게)
    handlers += new MultiTapHandler(ui->jaum_giyeok_kieuk, ui->textEdit, u'ㄱ', u'ㅋ', u'ㄲ', this, this);
    handlers += new MultiTapHandler(ui->jaum_digut_tieut,  ui->textEdit, u'ㄷ', u'ㅌ', u'ㄸ', this, this);
    handlers += new MultiTapHandler(ui->jaum_bieup_pieup,  ui->textEdit, u'ㅂ', u'ㅍ', u'ㅃ', this, this);
    handlers += new MultiTapHandler(ui->jaum_siot_huieut,  ui->textEdit, u'ㅅ', u'ㅎ', u'ㅆ', this, this);
    handlers += new MultiTapHandler(ui->jaum_jieut_chieut, ui->textEdit, u'ㅈ', u'ㅊ', u'ㅉ', this, this);
    handlers += new MultiTapHandler(ui->jaum_nieun_rieul,  ui->textEdit, u'ㄴ', u'ㄹ', QChar(), this, this);
    handlers += new MultiTapHandler(ui->jaum_yieung_mium,  ui->textEdit, u'ㅇ', u'ㅁ', QChar(), this, this);

    // 스페이스/엔터/삭제
    connect(ui->space_kor, &QPushButton::clicked, this, &MainWindow::on_space_kor_clicked);
    connect(ui->enter_kor, &QPushButton::clicked, this, &MainWindow::on_enter_kor_clicked);
    connect(ui->delete_kor, &QPushButton::clicked, this, &MainWindow::on_delete_kor_clicked);
    // --- 모음 키 연결: Vowels_floor=ㅡ, Vowels_sun=ㆍ, Vowels_people=ㅣ
    connect(ui->Vowels_floor,  &QPushButton::clicked, [this]{ // ㅡ
        auto o = vowel_.pressFloorU();
        auto cur = ui->textEdit->textCursor();
        if (o.valid()) {
            if (o.replaceLast && !cur.atStart()) cur.deletePreviousChar();
            insertJamo(o.ch);
        }
    });

    connect(ui->Vowels_sun,    &QPushButton::clicked, [this]{ // ㆍ
        auto o = vowel_.pressDotA();
        auto cur = ui->textEdit->textCursor();
        if (o.valid()) {
            if (o.replaceLast && !cur.atStart()) cur.deletePreviousChar();
            insertJamo(o.ch);
        }
    });

    connect(ui->Vowels_people, &QPushButton::clicked, [this]{ // ㅣ
        auto o = vowel_.pressSunI();
        auto cur = ui->textEdit->textCursor();
        if (o.valid()) {
            if (o.replaceLast && !cur.atStart()) cur.deletePreviousChar();
            insertJamo(o.ch);
        }
        lastInsertedByDotComEng = false;
        lastInsertedByDotComKor = false;
    });

    // ---- ENG T9 ----
    connect(ui->ABC,  &QPushButton::clicked, [this]{ handleT9("ABC",  {"a","b","c"},          {"A","B","C"}); });
    connect(ui->DEF,  &QPushButton::clicked, [this]{ handleT9("DEF",  {"d","e","f"},          {"D","E","F"}); });
    connect(ui->GHI,  &QPushButton::clicked, [this]{ handleT9("GHI",  {"g","h","i"},          {"G","H","I"}); });
    connect(ui->JKL,  &QPushButton::clicked, [this]{ handleT9("JKL",  {"j","k","l"},          {"J","K","L"}); });
    connect(ui->MNO,  &QPushButton::clicked, [this]{ handleT9("MNO",  {"m","n","o"},          {"M","N","O"}); });
    connect(ui->PQRS, &QPushButton::clicked, [this]{ handleT9("PQRS", {"p","q","r","s"},      {"P","Q","R","S"}); });
    connect(ui->TUV,  &QPushButton::clicked, [this]{ handleT9("TUV",  {"t","u","v"},          {"T","U","V"}); });
    connect(ui->WXYZ, &QPushButton::clicked, [this]{ handleT9("WXYZ", {"w","x","y","z"},      {"W","X","Y","Z"}); });

    // delete/space/enter 는 확정 입력이므로 T9 상태 초기화
    connect(ui->delete_eng, &QPushButton::clicked, [this]{
        auto cur = ui->textEdit->textCursor();
        if (!cur.atStart()) { cur.deletePreviousChar(); ui->textEdit->setTextCursor(cur); }
        lastT9Key.clear();
        lastInsertedByDotComEng = false;
    });
    connect(ui->space_eng, &QPushButton::clicked, [this]{
        ui->textEdit->insertPlainText(" ");
        lastT9Key.clear();
        lastInsertedByDotComEng = false;
    });
    connect(ui->enter_eng, &QPushButton::clicked, [this]{
        ui->textEdit->append("");
        lastT9Key.clear();
        lastInsertedByDotComEng = false;
    });

    // 시프트/캡스
    connect(ui->shfit_capslock, &QPushButton::clicked,
            this, &MainWindow::on_shift_capslock_clicked);

    // 한/영 전환(영 → 한)
    connect(ui->kor_eng_change_eng, &QPushButton::clicked,
            this, &MainWindow::on_kor_eng_change_eng_clicked);

}

MainWindow::~MainWindow(){ delete ui; }

void MainWindow::on_space_kor_clicked()  { ui->textEdit->insertPlainText(" ");lastInsertedByDotComEng = false; lastInsertedByDotComKor = false;}
void MainWindow::on_enter_kor_clicked()  { ui->textEdit->append("");lastInsertedByDotComEng = false; lastInsertedByDotComKor = false;}
void MainWindow::on_delete_kor_clicked() {
    auto cur = ui->textEdit->textCursor();
    if (!cur.atStart()) { cur.deletePreviousChar(); ui->textEdit->setTextCursor(cur); }
    lastInsertedByDotComEng = false;
    lastInsertedByDotComKor = false;
}





void MainWindow::on_dot_com_que_exc_eng_clicked() {
    // 직전 입력이 이 버튼이 아니면 카운트 초기화
    if (!lastInsertedByDotComEng) {
        dotComQueExcCountEng = 0;
    }

    ++dotComQueExcCountEng;
    int mod = dotComQueExcCountEng % 4;

    QChar ch = (mod==1) ? u'.'
               : (mod==2) ? u','
               : (mod==3) ? u'?'
                            : u'!';

    QTextCursor cur = ui->textEdit->textCursor();

    if (lastInsertedByDotComEng && !cur.atStart()) {
        // 같은 버튼 연속 입력 → 직전 기호 삭제
        cur.deletePreviousChar();
    }

    cur.insertText(QString(ch));
    ui->textEdit->setTextCursor(cur);

    // 상태 플래그 업데이트
    lastInsertedByDotComEng = true;
}

//---------------------------------------------------------


void MainWindow::on_dot_com_que_exc_kor_clicked() {
    // 직전 입력이 이 버튼이 아니면 카운트 초기화
    if (!lastInsertedByDotComKor) {
        dotComQueExcCountKor = 0;
    }

    ++dotComQueExcCountKor;
    int mod = dotComQueExcCountKor % 4;

    QChar ch = (mod==1) ? u'.'
               : (mod==2) ? u','
               : (mod==3) ? u'?'
                            : u'!';

    QTextCursor cur = ui->textEdit->textCursor();

    if (lastInsertedByDotComKor && !cur.atStart()) {
        // 같은 버튼 연속 입력 → 직전 기호 삭제
        cur.deletePreviousChar();
    }

    cur.insertText(QString(ch));
    ui->textEdit->setTextCursor(cur);

    // 상태 플래그 업데이트
    lastInsertedByDotComKor = true;
}

