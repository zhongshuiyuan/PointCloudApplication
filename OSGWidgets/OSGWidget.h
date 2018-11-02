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

namespace osgEarth {
    class MapNode;
}

class LineEditor;
class TraceEditor;
class SelectEditor;

class OSGWidget : public QWidget, public osgViewer::CompositeViewer{
    Q_OBJECT
public:
    explicit OSGWidget(QWidget* parent = nullptr);
    ~OSGWidget() final;

    Q_DISABLE_COPY(OSGWidget);
public:
    void init();
    void loadVectorMap();
    void initTerrainManipulator();
    void readPCDataFromFile(const QFileInfo& file_info);

    void activeLineEditor(bool is_active);
    void activeTraceEditor(bool is_active);
    void activeSelectEditor(bool is_active);

    void saveVectorMapToDir(const std::string& dir_path) const;
    osg::ref_ptr<SelectEditor> select_editor_;

private:
    void paintEvent(QPaintEvent*) final;

    void initSceneGraph();
    void initCamera();
    void initEditor();
    void initManipulator();
    void initVectorMap();

    osgQt::GraphicsWindowQt* createGraphicsWindow(int x, int y, int w, int h, const std::string& name = "", bool windowDecoration = false);

    osg::Geode* readPCLDataFromFile(const QFileInfo& file_info) const;
    osg::Geode* addMapPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& mapPointCloud,
            osg::Vec3 color = osg::Vec3(1.0, 1.0, 1.0)) const;

    osg::Geode* readTXTDataFromFile(const QFileInfo& file_info) const;
    osg::Geode* addIntensityPointCloud(const osg::ref_ptr<osg::Vec3Array>& vertices,
            const std::vector<int>& intensity) const;

    void drawAllLines(); //test function
    //private template function can be defined in .cpp file.
    template <class T>
    void drawTraceItems(const std::vector<T>& objects);
    template <class T>
    void drawVectorItems(const std::vector<T>& objects);

    osg::ref_ptr<osgViewer::View>  main_view_;
    osg::ref_ptr<osg::Switch>      root_node_;
    osg::ref_ptr<osgEarth::MapNode> map_node_;

    osg::ref_ptr<LineEditor>     line_editor_;
    osg::ref_ptr<TraceEditor>   trace_editor_;
    QTimer* update_timer_;

public Q_SLOTS:

};


#endif //POINTCLOUDAPPLICATION_OSGWIDGET_H
