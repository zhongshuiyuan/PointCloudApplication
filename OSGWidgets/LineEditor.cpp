//
// Created by WuKun on 8/29/18.
// Contact me:wk707060335@gmail.com
//

#include "LineEditor.h"
#include "common.h"

LineEditor::LineEditor() :
        _mx(0),
        _my(0) {
}

bool LineEditor::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa){
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

void LineEditor::pick(const osgGA::GUIEventAdapter& ea, osgViewer::View* view) {
    if (ea.getButton() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON) {

        double w = 3.0f;
        double h = 3.0f;
        osg::ref_ptr<osgUtil::PolytopeIntersector> picker = new osgUtil::PolytopeIntersector(
                osgUtil::Intersector::WINDOW, _mx - w, _my - h, _mx + w, _my + h);
        osgUtil::IntersectionVisitor iv(picker);
        view->getCamera()->accept(iv);

        if (picker->containsIntersections()) {
            osg::NodePath all_node_path = picker->getFirstIntersection().nodePath;
            qDebug() << "node path" << all_node_path.size();
            for(auto node_path : all_node_path)
            {
                qDebug() << node_path->libraryName() << node_path->className() << node_path->getName().c_str();
            }

            typedef osgUtil::PolytopeIntersector::Intersections Inters;
            auto iter = picker->getIntersections().begin();
            if (iter != picker->getIntersections().end())
            {
                osg::Vec3 pt;
                osg::Vec3 selectedPoint(iter->localIntersectionPoint.x(), iter->localIntersectionPoint.y(), iter->localIntersectionPoint.z());

                pt = selectedPoint;
                //此时iter->primitiveIndex对应selectedPoints节点中的index！
                int index = iter->primitiveIndex;

                qDebug() << "index:" << index << "x:" << pt.x() << "y:" << pt.y() << "z:" << pt.z();
            }
        }
    }
}