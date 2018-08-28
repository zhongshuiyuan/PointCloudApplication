#include <QtWidgets/QApplication>

#include "MainWindow.h"


int main(int argc, char* argv[]) {
    QApplication a(argc, argv);

    MainWindow main_window;
    main_window.setGeometry(100, 100, 1500, 800);  //graphic_context!
//    mainwindow.showMaximized();
    main_window.show();

    return a.exec();
}