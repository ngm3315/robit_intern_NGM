#include <QApplication>
#include <rclcpp/rclcpp.hpp>
#include "hw_02/main_window.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);

    MainWindow w;
    w.show();

    int ret = app.exec();
    rclcpp::shutdown();
    return ret;
}

