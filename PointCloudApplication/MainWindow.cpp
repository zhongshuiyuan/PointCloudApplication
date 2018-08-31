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
#include <QtCore/QDebug>
#include <QtCore/QString>
#include <QtCore/QDir>
#include <QtCore/QFileInfoList>

#include "MainWindow.h"
#include "../OSGWidgets/OSGWidget.h"
#include "../Common/tracer.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    osgwidget_(nullptr),
    dock_widget_(nullptr),
    tree_widget_(nullptr),
    label_(nullptr),
    open_file_action(nullptr),
    draw_line_action(nullptr),
    draw_trace_action(nullptr),
    save_file_action(nullptr){
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

void MainWindow::createConnect() {
    TRACER;
}

void MainWindow::createMenu() {
    TRACER;
    open_file_action = new QAction("Open", this);
    open_file_action->setIcon(QIcon("../../resources/file_open.png"));
    connect(open_file_action, SIGNAL(triggered()), this, SLOT(openFile()));

    draw_line_action = new QAction("Draw Line", this);
    draw_line_action->setIcon(QIcon("../../resources/line.png"));
    draw_line_action->setCheckable(true);
    connect(draw_line_action, SIGNAL(toggled(bool)), this, SLOT(drawLine(bool)));

    draw_trace_action = new QAction("Draw trace", this);
    draw_trace_action->setIcon(QIcon("../../resources/curve.png"));
    draw_trace_action->setCheckable(true);
    connect(draw_trace_action, SIGNAL(toggled(bool)), this, SLOT(drawTrace(bool)));


    save_file_action = new QAction("Save", this);
    save_file_action->setIcon(QIcon("../../resources/file_save.png"));
    connect(save_file_action, SIGNAL(triggered()), this, SLOT(saveFile()));
//    QMenu *menu = menuBar()->addMenu("File");
//    menu->addAction(open_file_action);
}

void MainWindow::createToolBar() {
    QToolBar *toolBar = addToolBar("Tools");
    toolBar->addAction(open_file_action);
    toolBar->addSeparator();

    toolBar->addAction(draw_line_action);
    toolBar->addAction(draw_trace_action);
    toolBar->addSeparator();

    toolBar->addAction(save_file_action);
}

void MainWindow::createStatusBar() {
    label_ = new QLabel("ready");
    label_->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
//    statusBar()->addWidget(label_);
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

void MainWindow::drawLine(bool is_active) {
    if (is_active) {
        draw_trace_action->setChecked(false);
    }

    osgwidget_->activeLineEditor(is_active);
}

void MainWindow::drawTrace(bool is_active) {
    if (is_active) {
        draw_line_action->setChecked(false);
    }

    osgwidget_->activeTraceEditor(is_active);
}

void MainWindow::openFile() {
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

        osgwidget_->readPCDataFromFile(fileInfo);
    }

    osgwidget_->initTerrainManipulator();
}

void MainWindow::saveFile() {
    TRACER;
    std::string dir_path = "../../vmap";

    osgwidget_->saveVectorMapToDir(dir_path);
}