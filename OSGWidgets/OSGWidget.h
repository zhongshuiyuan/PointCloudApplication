//
// Created by WuKun on 8/24/18.
// Contact me:wk707060335@gmail.com
//

#ifndef POINTCLOUDAPPLICATION_OSGWIDGET_H
#define POINTCLOUDAPPLICATION_OSGWIDGET_H

#include <iostream>

#include <QtWidgets/QMainWindow>
#include <QtGui/QPaintEvent>
#include <QtCore/QString>
#include <QtCore/QTimer>
#include <QtCore/QFileInfo>

#include <osg/Vec3d>
#include <osg/Camera>
#include <osg/Switch>
#include <osg/Geode>
#include <osgViewer/View>
#include <osgViewer/Viewer>
#include <osgViewer/CompositeViewer>
#include <osgGA/TrackballManipulator>
#include <osgGA/TerrainManipulator>
#include <osgQt/GraphicsWindowQt>

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/console/time.h>

class LineEditor;

class OSGWidget : public QWidget, public osgViewer::CompositeViewer{
    Q_OBJECT
public:
    explicit OSGWidget(QWidget* parent = nullptr);
    ~OSGWidget() final = default;

    void init();
    void initTerrainManipulator();
    void readPCDataFromFile(const QFileInfo& file_info);

    void activeLineEditor(bool is_active);

private:
    void paintEvent(QPaintEvent*) final;

    void initSceneGraph();
    void initCamera();
    void initEditor();
    void initManipulator();

    osgQt::GraphicsWindowQt* createGraphicsWindow(int x, int y, int w, int h, const std::string& name = "", bool windowDecoration = false);
    osg::Geode* addMapPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& mapPointCloud, osg::Vec3 color = osg::Vec3(1.0, 1.0, 1.0));

    osg::ref_ptr<osgViewer::View> main_view_;
    osg::ref_ptr<osg::Switch>     root_node_;

    LineEditor*   line_editor_;
    QTimer* update_timer_;

    Q_DISABLE_COPY(OSGWidget);
public Q_SLOTS:

};


#endif //POINTCLOUDAPPLICATION_OSGWIDGET_H
