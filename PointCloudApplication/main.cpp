#include <QtWidgets/QApplication>

#include "MainWindow.h"


int main(int argc, char* argv[]) {
    QApplication a(argc, argv);

    MainWindow mainwindow;
//    mainwindow.showMaximized();
    mainwindow.show();

    return a.exec();
}