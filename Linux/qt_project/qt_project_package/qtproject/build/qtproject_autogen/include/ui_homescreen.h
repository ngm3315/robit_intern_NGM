/********************************************************************************
** Form generated from reading UI file 'homescreen.ui'
**
** Created by: Qt User Interface Compiler version 6.7.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_HOMESCREEN_H
#define UI_HOMESCREEN_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QStackedWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_HomeScreen
{
public:
    QVBoxLayout *mainLayout;
    QStackedWidget *stackedWidget;
    QWidget *Connection_page;
    QVBoxLayout *verticalLayout_2;
    QSpacerItem *verticalSpacer_2;
    QLabel *titleLabel;
    QLabel *countdownLabel;
    QSpacerItem *verticalSpacer;
    QWidget *Game_page;
    QWidget *Result_page;
    QVBoxLayout *verticalLayout_3;
    QLabel *result_text_box;
    QLabel *rematch_status_label;
    QHBoxLayout *horizontalLayout_2;
    QPushButton *rematch_button;
    QPushButton *exit_to_home_button;
    QHBoxLayout *bottomLayout;
    QLabel *status_label;
    QVBoxLayout *controlsLayout;
    QHBoxLayout *localPortLayout;
    QLabel *localPortLabel;
    QLineEdit *local_port_input_line;
    QHBoxLayout *ipLayout;
    QLabel *ipLabel;
    QLineEdit *ip_input_line;
    QLabel *portLabel;
    QLineEdit *port_input_line;
    QHBoxLayout *buttonLayout;
    QPushButton *connect_button;
    QPushButton *reset_botton;
    QPushButton *quit_button;

    void setupUi(QWidget *HomeScreen)
    {
        if (HomeScreen->objectName().isEmpty())
            HomeScreen->setObjectName("HomeScreen");
        HomeScreen->resize(800, 600);
        mainLayout = new QVBoxLayout(HomeScreen);
        mainLayout->setObjectName("mainLayout");
        stackedWidget = new QStackedWidget(HomeScreen);
        stackedWidget->setObjectName("stackedWidget");
        Connection_page = new QWidget();
        Connection_page->setObjectName("Connection_page");
        verticalLayout_2 = new QVBoxLayout(Connection_page);
        verticalLayout_2->setObjectName("verticalLayout_2");
        verticalSpacer_2 = new QSpacerItem(20, 40, QSizePolicy::Policy::Minimum, QSizePolicy::Policy::Expanding);

        verticalLayout_2->addItem(verticalSpacer_2);

        titleLabel = new QLabel(Connection_page);
        titleLabel->setObjectName("titleLabel");
        QFont font;
        font.setPointSize(24);
        font.setBold(true);
        titleLabel->setFont(font);
        titleLabel->setAlignment(Qt::AlignCenter);

        verticalLayout_2->addWidget(titleLabel);

        countdownLabel = new QLabel(Connection_page);
        countdownLabel->setObjectName("countdownLabel");
        QFont font1;
        font1.setPointSize(48);
        font1.setBold(true);
        countdownLabel->setFont(font1);
        countdownLabel->setStyleSheet(QString::fromUtf8("color: red;"));
        countdownLabel->setAlignment(Qt::AlignCenter);

        verticalLayout_2->addWidget(countdownLabel);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Policy::Minimum, QSizePolicy::Policy::Expanding);

        verticalLayout_2->addItem(verticalSpacer);

        stackedWidget->addWidget(Connection_page);
        Game_page = new QWidget();
        Game_page->setObjectName("Game_page");
        stackedWidget->addWidget(Game_page);
        Result_page = new QWidget();
        Result_page->setObjectName("Result_page");
        verticalLayout_3 = new QVBoxLayout(Result_page);
        verticalLayout_3->setObjectName("verticalLayout_3");
        result_text_box = new QLabel(Result_page);
        result_text_box->setObjectName("result_text_box");
        QFont font2;
        font2.setPointSize(36);
        result_text_box->setFont(font2);
        result_text_box->setAlignment(Qt::AlignCenter);

        verticalLayout_3->addWidget(result_text_box);

        rematch_status_label = new QLabel(Result_page);
        rematch_status_label->setObjectName("rematch_status_label");
        QFont font3;
        font3.setPointSize(16);
        rematch_status_label->setFont(font3);
        rematch_status_label->setAlignment(Qt::AlignCenter);

        verticalLayout_3->addWidget(rematch_status_label);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName("horizontalLayout_2");
        rematch_button = new QPushButton(Result_page);
        rematch_button->setObjectName("rematch_button");

        horizontalLayout_2->addWidget(rematch_button);

        exit_to_home_button = new QPushButton(Result_page);
        exit_to_home_button->setObjectName("exit_to_home_button");

        horizontalLayout_2->addWidget(exit_to_home_button);


        verticalLayout_3->addLayout(horizontalLayout_2);

        stackedWidget->addWidget(Result_page);

        mainLayout->addWidget(stackedWidget);

        bottomLayout = new QHBoxLayout();
        bottomLayout->setObjectName("bottomLayout");
        status_label = new QLabel(HomeScreen);
        status_label->setObjectName("status_label");
        status_label->setMaximumSize(QSize(16777215, 100));

        bottomLayout->addWidget(status_label);

        controlsLayout = new QVBoxLayout();
        controlsLayout->setObjectName("controlsLayout");
        localPortLayout = new QHBoxLayout();
        localPortLayout->setObjectName("localPortLayout");
        localPortLabel = new QLabel(HomeScreen);
        localPortLabel->setObjectName("localPortLabel");

        localPortLayout->addWidget(localPortLabel);

        local_port_input_line = new QLineEdit(HomeScreen);
        local_port_input_line->setObjectName("local_port_input_line");

        localPortLayout->addWidget(local_port_input_line);


        controlsLayout->addLayout(localPortLayout);

        ipLayout = new QHBoxLayout();
        ipLayout->setObjectName("ipLayout");
        ipLabel = new QLabel(HomeScreen);
        ipLabel->setObjectName("ipLabel");

        ipLayout->addWidget(ipLabel);

        ip_input_line = new QLineEdit(HomeScreen);
        ip_input_line->setObjectName("ip_input_line");

        ipLayout->addWidget(ip_input_line);

        portLabel = new QLabel(HomeScreen);
        portLabel->setObjectName("portLabel");

        ipLayout->addWidget(portLabel);

        port_input_line = new QLineEdit(HomeScreen);
        port_input_line->setObjectName("port_input_line");

        ipLayout->addWidget(port_input_line);


        controlsLayout->addLayout(ipLayout);

        buttonLayout = new QHBoxLayout();
        buttonLayout->setObjectName("buttonLayout");
        connect_button = new QPushButton(HomeScreen);
        connect_button->setObjectName("connect_button");

        buttonLayout->addWidget(connect_button);

        reset_botton = new QPushButton(HomeScreen);
        reset_botton->setObjectName("reset_botton");

        buttonLayout->addWidget(reset_botton);

        quit_button = new QPushButton(HomeScreen);
        quit_button->setObjectName("quit_button");

        buttonLayout->addWidget(quit_button);


        controlsLayout->addLayout(buttonLayout);


        bottomLayout->addLayout(controlsLayout);


        mainLayout->addLayout(bottomLayout);


        retranslateUi(HomeScreen);

        stackedWidget->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(HomeScreen);
    } // setupUi

    void retranslateUi(QWidget *HomeScreen)
    {
        HomeScreen->setWindowTitle(QCoreApplication::translate("HomeScreen", "UDP Hockey Game", nullptr));
        titleLabel->setText(QCoreApplication::translate("HomeScreen", "UDP Hockey", nullptr));
        countdownLabel->setText(QString());
        result_text_box->setText(QCoreApplication::translate("HomeScreen", "Winner / Loser", nullptr));
        rematch_status_label->setText(QString());
        rematch_button->setText(QCoreApplication::translate("HomeScreen", "Rematch", nullptr));
        exit_to_home_button->setText(QCoreApplication::translate("HomeScreen", "Exit to Home", nullptr));
        status_label->setText(QCoreApplication::translate("HomeScreen", "Status: ", nullptr));
        localPortLabel->setText(QCoreApplication::translate("HomeScreen", "Local Port:", nullptr));
        ipLabel->setText(QCoreApplication::translate("HomeScreen", "Opponent IP:", nullptr));
        portLabel->setText(QCoreApplication::translate("HomeScreen", "Opponent Port:", nullptr));
        connect_button->setText(QCoreApplication::translate("HomeScreen", "Connect", nullptr));
        reset_botton->setText(QCoreApplication::translate("HomeScreen", "Reset", nullptr));
        quit_button->setText(QCoreApplication::translate("HomeScreen", "Quit", nullptr));
    } // retranslateUi

};

namespace Ui {
    class HomeScreen: public Ui_HomeScreen {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_HOMESCREEN_H
