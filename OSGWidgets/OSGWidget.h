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
#include <QtCore/QDebug>
#include <QtCore/QFileInfo>

#include <osg/ref_ptr>
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


class OSGWidget : public QWidget, public osgViewer::CompositeViewer{
    Q_OBJECT
public:
    explicit OSGWidget(QWidget* parent = nullptr);
    ~OSGWidget() final = default;

    void init();
    void initTerrainManipulator();
    void readPCDataFromFile(const QFileInfo& file_info);

private:
    void paintEvent(QPaintEvent*) final;

    void initSceneGraph();
    void initCamera();
    void initManipulator();

    osgQt::GraphicsWindowQt* createGraphicsWindow(int x, int y, int w, int h, const std::string& name = "", bool windowDecoration = false);
    osg::Geode* addMapPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& mapPointCloud, osg::Vec3 color = osg::Vec3(1.0, 1.0, 1.0));

    osg::ref_ptr<osgViewer::View> main_view_;
    osg::ref_ptr<osg::Switch>     root_node_;

    QTimer* update_timer_;

    Q_DISABLE_COPY(OSGWidget);
public Q_SLOTS:

};

std::ostream& operator<<(std::ostream& os, const osg::Vec3d& point);

class PickHandler : public osgGA::GUIEventHandler
{

public:
    PickHandler() :
        _mx(0),
        _my(0) {
    }
    ~PickHandler() final = default;

    bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa) final{
        auto view = dynamic_cast<osgViewer::View*>(&aa);
        if (!view) return false;

        switch (ea.getEventType()){
            case(osgGA::GUIEventAdapter::MOVE):
            {
                _mx = ea.getX();
                _my = ea.getY();
                return false;
            }
            case(osgGA::GUIEventAdapter::RELEASE):
            {
                if (_mx == ea.getX() && _my == ea.getY())
                {
                    // only do a pick if the mouse hasn't moved
                    pick(ea,view);
                }
                return true;
            }

            default:
                return false;
        }
    }

    void pick(const osgGA::GUIEventAdapter& ea, osgViewer::View* view) {
        if (ea.getButton() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON) {

            double w = 3.0f;
            double h = 3.0f;
            osg::ref_ptr<osgUtil::PolytopeIntersector> picker = new osgUtil::PolytopeIntersector(
                    osgUtil::Intersector::WINDOW, _mx - w, _my - h, _mx + w, _my + h);
            osgUtil::IntersectionVisitor iv(picker);

            view->getCamera()->accept(iv);

            if (picker->containsIntersections()) {
                osg::NodePath all_node_path = picker->getFirstIntersection().nodePath;
                for(auto node_path : all_node_path)
                {
                    qDebug() << node_path->libraryName() << node_path->className() << node_path->getName().c_str();
                }
            }
        }
    }

protected:
    float _mx,_my;
};



#endif //POINTCLOUDAPPLICATION_OSGWIDGET_H
