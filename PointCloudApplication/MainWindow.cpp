//
// Created by WuKun on 8/24/18.
// Contact me:wk707060335@gmail.com
//
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QMessageBox>
#include <QtGui/QIcon>

#include "MainWindow.h"
#include "../OSGWidgets/OSGWidget.h"

MainWindow::MainWindow(QWidget *parent)
    :QMainWindow(parent),
    open_file_action(nullptr) {

    initUI();
}

void MainWindow::initUI() {

    open_file_action = new QAction("Open", this);
    open_file_action->setStatusTip("Open a file");
    open_file_action->setIcon(QIcon("../../resources/file_open.png"));
    connect(open_file_action, SIGNAL(triggered()), this, SLOT(open()));

    QMenu *menu = menuBar()->addMenu("File");

    menu->addAction(open_file_action);

    QToolBar *toolBar = addToolBar("Tools");
    toolBar->addAction(open_file_action);
    toolBar->addSeparator();
}

void MainWindow::open() {
    OSGWidget osgWidget;

    QMessageBox::information(this, "Open", "Open a file");
}