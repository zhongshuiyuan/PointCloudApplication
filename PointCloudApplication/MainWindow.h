//
// Created by WuKun on 8/24/18.
// Contact me:wk707060335@gmail.com
//

#ifndef POINTCLOUDAPPLICATION_MAINWINDOW_H
#define POINTCLOUDAPPLICATION_MAINWINDOW_H

#include <QtWidgets/QMainWindow>
#include <QtWidgets/QProgressBar>
#include <QtWidgets/QDockWidget>
#include <QtWidgets/QTreeWidget>
#include <QtWidgets/QAction>
#include <QtWidgets/QLabel>

//forward declaration
class OSGWidget;
class EditorDialog;

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() final = default ;

    Q_DISABLE_COPY(MainWindow);
private:
    void initUI();

    void createMenu();
    void createToolBar();
    void createStatusBar();
    void createDockWidget();

    void createConnect();

    void closeEvent(QCloseEvent *event) final {};

    //core widget
    OSGWidget*      osgwidget_;

    //other widgets
    QDockWidget*  dock_widget_;
    QTreeWidget*  tree_widget_;

    //items
    QLabel*              label_;
    QAction*     clicked_action;
    QAction*   open_file_action;
    QAction*   draw_line_action;
    QAction*  draw_trace_action;
    QAction*   save_file_action;
    QAction* select_item_action;

Q_SIGNALS:
    void setSelectedItem(QStringList itemInfo);

private Q_SLOTS:
    void openFile();
    void saveFile();
    void drawLine(bool is_active);
    void drawTrace(bool is_active);
    void selectItem(bool is_active);
    void receiveItem(QStringList itemInfo);
};


#endif //POINTCLOUDAPPLICATION_MAINWINDOW_H
