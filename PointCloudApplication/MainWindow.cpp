//
// Created by WuKun on 8/24/18.
// Contact me:wk707060335@gmail.com
//
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QMessageBox>
#include <QtWidgets/QDesktopWidget>
#include <QtWidgets/QStatusBar>
#include <QtGui/QIcon>
#include <QtCore/QString>
#include <QtCore/QDir>
#include <QtCore/QFileInfoList>

#include "MainWindow.h"
#include "../OSGWidgets/OSGWidget.h"
#include "../Common/tracer.h"

MainWindow::MainWindow(QWidget *parent)
    :QMainWindow(parent),
    open_file_action(nullptr) {

    initUI();
}

void MainWindow::initUI() {

    this->setWindowTitle("PointCloudApplication");

    osgwidget_ = new OSGWidget(this);
    this->setCentralWidget(osgwidget_);
    osgwidget_->init();
    createConnect();

    createMenu();
    createToolBar();
    createStatusBar();
    createDockWidget();
}

void MainWindow::open_file() {
//    QMessageBox::information(this, "Open", "Open a file");
    TRACER;
    QDir dir("/home/zhihui/.autoware/data/map/pointcloud_map");

    QStringList filters;
    filters << "*.pcd";
    dir.setNameFilters(filters);

    QFileInfoList list = dir.entryInfoList();
    for(const QFileInfo& fileInfo : list){
        std::cout << qPrintable(QString("%1 %2").arg(fileInfo.size(), 10)
                                        .arg(fileInfo.fileName()));
        std::cout << std::endl;

        osgwidget_->read_data_from_file(fileInfo);
    }

    osgwidget_->initTerrainManipulator();
}

void MainWindow::createConnect() {
    TRACER;
}

void MainWindow::createMenu() {
    TRACER;
    open_file_action = new QAction("Open", this);
    open_file_action->setStatusTip("Open a file");
    open_file_action->setIcon(QIcon("../../resources/file_open.png"));
    connect(open_file_action, SIGNAL(triggered()), this, SLOT(open_file()));

    QMenu *menu = menuBar()->addMenu("File");
    menu->addAction(open_file_action);
}

void MainWindow::createToolBar() {
    TRACER;

    QToolBar *toolBar = addToolBar("Tools");
    toolBar->addAction(open_file_action);
    toolBar->addSeparator();
}

void MainWindow::createStatusBar() {
    TRACER;

    label_ = new QLabel("ready");
    label_->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    statusBar()->addWidget(label_);
}

void MainWindow::createDockWidget() {
    TRACER;

    tree_widget_ = new QTreeWidget(this);
    tree_widget_->setColumnCount(1);
    tree_widget_->setHeaderHidden(true);
    //m_pTreeWidget->setColumnWidth(0, 100);
    //tree_widget_->setStyleSheet("QTreeWidget::item {height:25px;");

    QTreeWidgetItem *multiDataItem = new QTreeWidgetItem(tree_widget_, QStringList(QStringLiteral("数据列表")));
    multiDataItem->setExpanded(true);
    //multiDataItem->setCheckState(0, Qt::CheckState::Checked);
    {

    }

    dock_widget_ = new QDockWidget(QStringLiteral("场景数据"), this);
    dock_widget_->setFeatures(QDockWidget::AllDockWidgetFeatures);
    dock_widget_->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    dock_widget_->setWidget(tree_widget_);
    this->addDockWidget(Qt::LeftDockWidgetArea, dock_widget_);

    //QTreeWidget connect
    //connect(tree_widget_, SIGNAL(itemClicked(QTreeWidgetItem *, int)), this, SLOT(TreeWidgetClicked(QTreeWidgetItem *, int)));
    //connect(tree_widget_, SIGNAL(itemDoubleClicked(QTreeWidgetItem *, int)), this, SLOT(TreeWidgetDoubleClicked(QTreeWidgetItem *, int)));
    //connect(tree_widget_, SIGNAL(itemPressed(QTreeWidgetItem *, int)), this, SLOT(TreeWidgetRightedClicked(QTreeWidgetItem *, int)));
}