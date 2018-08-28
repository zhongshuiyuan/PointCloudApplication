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

namespace osgQt
{ class GraphicsWindowQt; };


class OSGWidget : public QWidget, public osgViewer::CompositeViewer{
    Q_OBJECT
public:
    explicit OSGWidget(QWidget* parent = nullptr);
    ~OSGWidget() final = default;

    void init();
    void initTerrainManipulator();
    bool read_data_from_file(const QFileInfo& file_path);

private:
    void paintEvent(QPaintEvent* event) final;

    void initSceneGraph();
    void initCamera();
    void initManipulator();

    osg::ref_ptr<osg::Switch> root_node_;
    osg::ref_ptr<osgViewer::Viewer> main_view_;

    QTimer* update_timer_;

    Q_DISABLE_COPY(OSGWidget);


public Q_SLOTS:

};


std::ostream& operator<<(std::ostream& os, const osg::Vec3d& point);

class PickHandler : public osgGA::GUIEventHandler
{

public:
    PickHandler(){}
    ~PickHandler(){}

    bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa) {
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

//            osg::ref_ptr< osgUtil::LineSegmentIntersector > picker =
//                    new osgUtil::LineSegmentIntersector(osgUtil::Intersector::WINDOW, ea.getX(), ea.getY());
//            osgUtil::IntersectionVisitor iv(picker);

            view->getCamera()->accept(iv);

            {
                auto mani = view->getCameraManipulator();
                auto mani2 = dynamic_cast<osgGA::TrackballManipulator*>(mani);
//
//                osg::Vec3d eye, up, center;
//                view->getCameraManipulator()->getHomePosition(eye, center, up);
                std::cout << "manipulator position: "<< mani2->getCenter() << " " << mani2->getDistance() << std::endl;
            }

            if (picker->containsIntersections()) {
                osg::NodePath nodepath = picker->getFirstIntersection().nodePath;
                for (int i = 0; i < nodepath.size(); i++)
                {
                	qDebug() << nodepath[i]->libraryName() << nodepath[i]->className() << nodepath[i]->getName().c_str();
                    //printf("十六进制  %x\n", nodepath[i]->getNodeMask());

                    if (nodepath[i]->className() == "Geode") {
                        auto boundingbox = nodepath[i]->getBound();
                        qDebug() << boundingbox._center.x()<< boundingbox._center.y()<< boundingbox._center.z() << boundingbox.radius();
                        nodepath[i]->setNodeMask(0);
                    }
                }
            }
        }
    }

protected:
    float _mx,_my;
};



#endif //POINTCLOUDAPPLICATION_OSGWIDGET_H
