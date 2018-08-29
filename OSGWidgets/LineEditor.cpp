//
// Created by WuKun on 8/29/18.
// Contact me:wk707060335@gmail.com
//

#include "LineEditor.h"
#include "common.h"
#include "NodeTreeSearch.h"
#include "../Common/tracer.h"

LineEditor::LineEditor(osg::Switch* root_node) :
    root_node_(root_node),
    line_node_(nullptr),
    temp_node_(nullptr),
    _mx(0),
    _my(0) {

    //find related nodes
    line_node_ = dynamic_cast<osg::Switch*>(findNodeWithName(root_node, line_node_name));

    temp_node_ = dynamic_cast<osg::Switch*>(findNodeWithName(root_node, temp_node_name));
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
            std::cout << "node path: " << all_node_path.size() << std::endl;
            for(auto node_path : all_node_path)
            {
                std::cout << node_path->libraryName() << " " << node_path->className() << " " << node_path->getName().c_str() << std::endl;
            }

            auto iter = picker->getIntersections().begin();
            if (iter != picker->getIntersections().end())
            {
                osg::Vec3d pt(iter->localIntersectionPoint);

                std::cout << "point: " << pt << std::endl;
            }
        }
    }
}

std::ostream& operator<<(std::ostream& os, const osg::Vec3d& point){
    os << point.x() << ","
       << point.y() << ","
       << point.z();
    return os;
}
