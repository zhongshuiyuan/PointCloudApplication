//
// Created by WuKun on 8/24/18.
// Contact me:wk707060335@gmail.com
//

#ifndef POINTCLOUDAPPLICATION_MAINWINDOW_H
#define POINTCLOUDAPPLICATION_MAINWINDOW_H

#include <QtWidgets/QMainWindow>
#include <QtWidgets/QAction>

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() final = default ;

private:
    void initUI();

    QAction* open_file_action;

signals:
    void test_signal();

private slots:
    void open();
};


#endif //POINTCLOUDAPPLICATION_MAINWINDOW_H
