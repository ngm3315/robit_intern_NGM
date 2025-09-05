/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 6.9.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGraphicsView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QSlider *all_toque_slide;
    QSlider *toque1_slide;
    QSlider *toque2_slide;
    QPushButton *save_to_txt;
    QPushButton *load_prev;
    QTextEdit *textEdit;
    QTextEdit *textEdit_2;
    QTextEdit *textEdit_3;
    QPushButton *A_T1_C;
    QSlider *toque3_slide;
    QTextEdit *textEdit_4;
    QPushButton *A_T1_C_R;
    QPushButton *A_T2_C;
    QPushButton *A_T2_C_R;
    QPushButton *A_T3_C;
    QPushButton *A_T3_C_R;
    QPushButton *A_all_C;
    QPushButton *A_all_C_R;
    QTextEdit *textEdit_5;
    QPushButton *init;
    QGraphicsView *graphicsView;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName("MainWindow");
        MainWindow->resize(800, 600);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName("centralwidget");
        all_toque_slide = new QSlider(centralwidget);
        all_toque_slide->setObjectName("all_toque_slide");
        all_toque_slide->setGeometry(QRect(340, 390, 431, 16));
        all_toque_slide->setOrientation(Qt::Orientation::Horizontal);
        toque1_slide = new QSlider(centralwidget);
        toque1_slide->setObjectName("toque1_slide");
        toque1_slide->setGeometry(QRect(340, 420, 431, 16));
        toque1_slide->setOrientation(Qt::Orientation::Horizontal);
        toque2_slide = new QSlider(centralwidget);
        toque2_slide->setObjectName("toque2_slide");
        toque2_slide->setGeometry(QRect(340, 450, 431, 16));
        toque2_slide->setOrientation(Qt::Orientation::Horizontal);
        save_to_txt = new QPushButton(centralwidget);
        save_to_txt->setObjectName("save_to_txt");
        save_to_txt->setGeometry(QRect(330, 520, 95, 25));
        load_prev = new QPushButton(centralwidget);
        load_prev->setObjectName("load_prev");
        load_prev->setGeometry(QRect(160, 520, 95, 25));
        textEdit = new QTextEdit(centralwidget);
        textEdit->setObjectName("textEdit");
        textEdit->setGeometry(QRect(240, 380, 91, 31));
        textEdit_2 = new QTextEdit(centralwidget);
        textEdit_2->setObjectName("textEdit_2");
        textEdit_2->setGeometry(QRect(240, 410, 91, 31));
        textEdit_3 = new QTextEdit(centralwidget);
        textEdit_3->setObjectName("textEdit_3");
        textEdit_3->setGeometry(QRect(240, 440, 91, 31));
        A_T1_C = new QPushButton(centralwidget);
        A_T1_C->setObjectName("A_T1_C");
        A_T1_C->setGeometry(QRect(20, 410, 95, 25));
        toque3_slide = new QSlider(centralwidget);
        toque3_slide->setObjectName("toque3_slide");
        toque3_slide->setGeometry(QRect(340, 480, 431, 16));
        toque3_slide->setOrientation(Qt::Orientation::Horizontal);
        textEdit_4 = new QTextEdit(centralwidget);
        textEdit_4->setObjectName("textEdit_4");
        textEdit_4->setGeometry(QRect(240, 470, 91, 31));
        A_T1_C_R = new QPushButton(centralwidget);
        A_T1_C_R->setObjectName("A_T1_C_R");
        A_T1_C_R->setGeometry(QRect(130, 410, 95, 25));
        A_T2_C = new QPushButton(centralwidget);
        A_T2_C->setObjectName("A_T2_C");
        A_T2_C->setGeometry(QRect(20, 440, 95, 25));
        A_T2_C_R = new QPushButton(centralwidget);
        A_T2_C_R->setObjectName("A_T2_C_R");
        A_T2_C_R->setGeometry(QRect(130, 440, 95, 25));
        A_T3_C = new QPushButton(centralwidget);
        A_T3_C->setObjectName("A_T3_C");
        A_T3_C->setGeometry(QRect(20, 470, 95, 25));
        A_T3_C_R = new QPushButton(centralwidget);
        A_T3_C_R->setObjectName("A_T3_C_R");
        A_T3_C_R->setGeometry(QRect(130, 470, 95, 25));
        A_all_C = new QPushButton(centralwidget);
        A_all_C->setObjectName("A_all_C");
        A_all_C->setGeometry(QRect(20, 380, 95, 25));
        A_all_C_R = new QPushButton(centralwidget);
        A_all_C_R->setObjectName("A_all_C_R");
        A_all_C_R->setGeometry(QRect(130, 380, 95, 25));
        textEdit_5 = new QTextEdit(centralwidget);
        textEdit_5->setObjectName("textEdit_5");
        textEdit_5->setGeometry(QRect(340, 340, 431, 31));
        init = new QPushButton(centralwidget);
        init->setObjectName("init");
        init->setGeometry(QRect(510, 520, 95, 25));
        graphicsView = new QGraphicsView(centralwidget);
        graphicsView->setObjectName("graphicsView");
        graphicsView->setGeometry(QRect(50, 10, 701, 321));
        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName("menubar");
        menubar->setGeometry(QRect(0, 0, 800, 22));
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName("statusbar");
        MainWindow->setStatusBar(statusbar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "MainWindow", nullptr));
        save_to_txt->setText(QCoreApplication::translate("MainWindow", "save to txt", nullptr));
        load_prev->setText(QCoreApplication::translate("MainWindow", "load(prev)", nullptr));
        textEdit->setHtml(QCoreApplication::translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><meta charset=\"utf-8\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"hr { height: 1px; border-width: 0; }\n"
"li.unchecked::marker { content: \"\\2610\"; }\n"
"li.checked::marker { content: \"\\2612\"; }\n"
"</style></head><body style=\" font-family:'Ubuntu'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">all</p></body></html>", nullptr));
        textEdit_2->setHtml(QCoreApplication::translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><meta charset=\"utf-8\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"hr { height: 1px; border-width: 0; }\n"
"li.unchecked::marker { content: \"\\2610\"; }\n"
"li.checked::marker { content: \"\\2612\"; }\n"
"</style></head><body style=\" font-family:'Ubuntu'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">toque1</p></body></html>", nullptr));
        textEdit_3->setHtml(QCoreApplication::translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><meta charset=\"utf-8\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"hr { height: 1px; border-width: 0; }\n"
"li.unchecked::marker { content: \"\\2610\"; }\n"
"li.checked::marker { content: \"\\2612\"; }\n"
"</style></head><body style=\" font-family:'Ubuntu'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">toque2</p></body></html>", nullptr));
        A_T1_C->setText(QCoreApplication::translate("MainWindow", "A_T1_C", nullptr));
        textEdit_4->setHtml(QCoreApplication::translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><meta charset=\"utf-8\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"hr { height: 1px; border-width: 0; }\n"
"li.unchecked::marker { content: \"\\2610\"; }\n"
"li.checked::marker { content: \"\\2612\"; }\n"
"</style></head><body style=\" font-family:'Ubuntu'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">toque3</p></body></html>", nullptr));
        A_T1_C_R->setText(QCoreApplication::translate("MainWindow", "A_T1_C_R", nullptr));
        A_T2_C->setText(QCoreApplication::translate("MainWindow", "A_T2_C", nullptr));
        A_T2_C_R->setText(QCoreApplication::translate("MainWindow", "A_T2_C_R", nullptr));
        A_T3_C->setText(QCoreApplication::translate("MainWindow", "A_T3_C", nullptr));
        A_T3_C_R->setText(QCoreApplication::translate("MainWindow", "A_T3_C_R", nullptr));
        A_all_C->setText(QCoreApplication::translate("MainWindow", "A_all_C", nullptr));
        A_all_C_R->setText(QCoreApplication::translate("MainWindow", "A_all_C_R", nullptr));
        textEdit_5->setHtml(QCoreApplication::translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><meta charset=\"utf-8\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"hr { height: 1px; border-width: 0; }\n"
"li.unchecked::marker { content: \"\\2610\"; }\n"
"li.checked::marker { content: \"\\2612\"; }\n"
"</style></head><body style=\" font-family:'Ubuntu'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">slide dgree</p></body></html>", nullptr));
        init->setText(QCoreApplication::translate("MainWindow", "init", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
