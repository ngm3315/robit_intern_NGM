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
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QStackedWidget>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTextBrowser>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QWidget *verticalLayoutWidget;
    QVBoxLayout *verticalLayout;
    QPushButton *Path_Search;
    QPushButton *Map_Editing;
    QPushButton *Map_Setting;
    QStackedWidget *stackedWidget;
    QWidget *Path_Search_Page;
    QWidget *verticalLayoutWidget_2;
    QVBoxLayout *verticalLayout_2;
    QPushButton *Start;
    QPushButton *Stop;
    QWidget *Map_Editing_Page;
    QWidget *verticalLayoutWidget_4;
    QVBoxLayout *verticalLayout_4;
    QTextEdit *textEdit_2;
    QSpinBox *obstacle_percentage;
    QPushButton *generate_map_editing_page;
    QPushButton *claer_map_editing_page;
    QWidget *Map_Setting_Page;
    QWidget *verticalLayoutWidget_3;
    QVBoxLayout *verticalLayout_3;
    QTextEdit *textEdit;
    QSpinBox *spinBox;
    QPushButton *apply_map_setting_page;
    QTextBrowser *textBrowser;
    QGraphicsView *Astar_map;
    QGraphicsView *Dijkstra_map;
    QTextBrowser *textBrowser_2;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName("MainWindow");
        MainWindow->resize(1451, 576);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName("centralwidget");
        verticalLayoutWidget = new QWidget(centralwidget);
        verticalLayoutWidget->setObjectName("verticalLayoutWidget");
        verticalLayoutWidget->setGeometry(QRect(0, 0, 171, 361));
        verticalLayout = new QVBoxLayout(verticalLayoutWidget);
        verticalLayout->setObjectName("verticalLayout");
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        Path_Search = new QPushButton(verticalLayoutWidget);
        Path_Search->setObjectName("Path_Search");

        verticalLayout->addWidget(Path_Search);

        Map_Editing = new QPushButton(verticalLayoutWidget);
        Map_Editing->setObjectName("Map_Editing");

        verticalLayout->addWidget(Map_Editing);

        Map_Setting = new QPushButton(verticalLayoutWidget);
        Map_Setting->setObjectName("Map_Setting");

        verticalLayout->addWidget(Map_Setting);

        stackedWidget = new QStackedWidget(verticalLayoutWidget);
        stackedWidget->setObjectName("stackedWidget");
        Path_Search_Page = new QWidget();
        Path_Search_Page->setObjectName("Path_Search_Page");
        verticalLayoutWidget_2 = new QWidget(Path_Search_Page);
        verticalLayoutWidget_2->setObjectName("verticalLayoutWidget_2");
        verticalLayoutWidget_2->setGeometry(QRect(40, 30, 91, 80));
        verticalLayout_2 = new QVBoxLayout(verticalLayoutWidget_2);
        verticalLayout_2->setObjectName("verticalLayout_2");
        verticalLayout_2->setContentsMargins(0, 0, 0, 0);
        Start = new QPushButton(verticalLayoutWidget_2);
        Start->setObjectName("Start");

        verticalLayout_2->addWidget(Start);

        Stop = new QPushButton(verticalLayoutWidget_2);
        Stop->setObjectName("Stop");

        verticalLayout_2->addWidget(Stop);

        stackedWidget->addWidget(Path_Search_Page);
        Map_Editing_Page = new QWidget();
        Map_Editing_Page->setObjectName("Map_Editing_Page");
        verticalLayoutWidget_4 = new QWidget(Map_Editing_Page);
        verticalLayoutWidget_4->setObjectName("verticalLayoutWidget_4");
        verticalLayoutWidget_4->setGeometry(QRect(0, 30, 160, 131));
        verticalLayout_4 = new QVBoxLayout(verticalLayoutWidget_4);
        verticalLayout_4->setObjectName("verticalLayout_4");
        verticalLayout_4->setContentsMargins(0, 0, 0, 0);
        textEdit_2 = new QTextEdit(verticalLayoutWidget_4);
        textEdit_2->setObjectName("textEdit_2");

        verticalLayout_4->addWidget(textEdit_2);

        obstacle_percentage = new QSpinBox(verticalLayoutWidget_4);
        obstacle_percentage->setObjectName("obstacle_percentage");

        verticalLayout_4->addWidget(obstacle_percentage);

        generate_map_editing_page = new QPushButton(verticalLayoutWidget_4);
        generate_map_editing_page->setObjectName("generate_map_editing_page");

        verticalLayout_4->addWidget(generate_map_editing_page);

        claer_map_editing_page = new QPushButton(verticalLayoutWidget_4);
        claer_map_editing_page->setObjectName("claer_map_editing_page");

        verticalLayout_4->addWidget(claer_map_editing_page);

        stackedWidget->addWidget(Map_Editing_Page);
        Map_Setting_Page = new QWidget();
        Map_Setting_Page->setObjectName("Map_Setting_Page");
        verticalLayoutWidget_3 = new QWidget(Map_Setting_Page);
        verticalLayoutWidget_3->setObjectName("verticalLayoutWidget_3");
        verticalLayoutWidget_3->setGeometry(QRect(10, 30, 160, 135));
        verticalLayout_3 = new QVBoxLayout(verticalLayoutWidget_3);
        verticalLayout_3->setObjectName("verticalLayout_3");
        verticalLayout_3->setContentsMargins(0, 0, 0, 0);
        textEdit = new QTextEdit(verticalLayoutWidget_3);
        textEdit->setObjectName("textEdit");

        verticalLayout_3->addWidget(textEdit);

        spinBox = new QSpinBox(verticalLayoutWidget_3);
        spinBox->setObjectName("spinBox");

        verticalLayout_3->addWidget(spinBox);

        apply_map_setting_page = new QPushButton(verticalLayoutWidget_3);
        apply_map_setting_page->setObjectName("apply_map_setting_page");

        verticalLayout_3->addWidget(apply_map_setting_page);

        stackedWidget->addWidget(Map_Setting_Page);

        verticalLayout->addWidget(stackedWidget);

        textBrowser = new QTextBrowser(centralwidget);
        textBrowser->setObjectName("textBrowser");
        textBrowser->setGeometry(QRect(190, 0, 591, 31));
        Astar_map = new QGraphicsView(centralwidget);
        Astar_map->setObjectName("Astar_map");
        Astar_map->setGeometry(QRect(190, 30, 591, 501));
        Dijkstra_map = new QGraphicsView(centralwidget);
        Dijkstra_map->setObjectName("Dijkstra_map");
        Dijkstra_map->setGeometry(QRect(810, 30, 591, 501));
        textBrowser_2 = new QTextBrowser(centralwidget);
        textBrowser_2->setObjectName("textBrowser_2");
        textBrowser_2->setGeometry(QRect(810, 0, 591, 31));
        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName("menubar");
        menubar->setGeometry(QRect(0, 0, 1451, 22));
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName("statusbar");
        MainWindow->setStatusBar(statusbar);

        retranslateUi(MainWindow);

        stackedWidget->setCurrentIndex(2);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "MainWindow", nullptr));
        Path_Search->setText(QCoreApplication::translate("MainWindow", "Path_Search", nullptr));
        Map_Editing->setText(QCoreApplication::translate("MainWindow", "Map_Editing", nullptr));
        Map_Setting->setText(QCoreApplication::translate("MainWindow", "Map_Setting", nullptr));
        Start->setText(QCoreApplication::translate("MainWindow", "Start", nullptr));
        Stop->setText(QCoreApplication::translate("MainWindow", "Stop", nullptr));
        textEdit_2->setHtml(QCoreApplication::translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><meta charset=\"utf-8\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"hr { height: 1px; border-width: 0; }\n"
"li.unchecked::marker { content: \"\\2610\"; }\n"
"li.checked::marker { content: \"\\2612\"; }\n"
"</style></head><body style=\" font-family:'Ubuntu'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-weight:700;\">Obstacle [%]</span></p></body></html>", nullptr));
        generate_map_editing_page->setText(QCoreApplication::translate("MainWindow", "Generate", nullptr));
        claer_map_editing_page->setText(QCoreApplication::translate("MainWindow", "Clear", nullptr));
        textEdit->setHtml(QCoreApplication::translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><meta charset=\"utf-8\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"hr { height: 1px; border-width: 0; }\n"
"li.unchecked::marker { content: \"\\2610\"; }\n"
"li.checked::marker { content: \"\\2612\"; }\n"
"</style></head><body style=\" font-family:'Ubuntu'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-weight:700;\">Cell Size (pxl)</span></p></body></html>", nullptr));
        apply_map_setting_page->setText(QCoreApplication::translate("MainWindow", "Apply", nullptr));
        textBrowser->setHtml(QCoreApplication::translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><meta charset=\"utf-8\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"hr { height: 1px; border-width: 0; }\n"
"li.unchecked::marker { content: \"\\2610\"; }\n"
"li.checked::marker { content: \"\\2612\"; }\n"
"</style></head><body style=\" font-family:'Ubuntu'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-weight:700;\">A*</span></p></body></html>", nullptr));
        textBrowser_2->setHtml(QCoreApplication::translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><meta charset=\"utf-8\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"hr { height: 1px; border-width: 0; }\n"
"li.unchecked::marker { content: \"\\2610\"; }\n"
"li.checked::marker { content: \"\\2612\"; }\n"
"</style></head><body style=\" font-family:'Ubuntu'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-weight:700;\">Dijkstra</span></p></body></html>", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
