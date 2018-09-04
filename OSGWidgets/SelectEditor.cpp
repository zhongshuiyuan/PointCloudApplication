//
// Created by WuKun on 9/4/18.
// Contact me:wk707060335@gmail.com
//

#include <vector>
#include <iomanip>
#include <algorithm> // for copy
#include <iterator> // for ostream_iterator

#include <osg/ValueObject>
#include <osg/ShapeDrawable>
#include <osgFX/Outline>

#include "SelectEditor.h"
#include "common.h"
#include "NodeTreeSearch.h"
#include "../Common/tracer.h"
#include "../Common/VectorMapSingleton.h"
#include "DataStructure.h"

using m_map::Point;
using m_map::Line;
using m_map::Area;
using m_map::Node;
using m_map::Lane;
using m_map::dtLane;

SelectEditor::SelectEditor(osg::Switch *root) :
    root_node_(root){
    vmap_node_ = dynamic_cast<osg::Switch*>(NodeTreeSearch::findNodeWithName(root_node_, vmap_node_name));
    temp_node_ = dynamic_cast<osg::Switch*>(NodeTreeSearch::findNodeWithName(root_node_, temp_node_name));
}

SelectEditor::~SelectEditor() {

}


bool SelectEditor::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa){
    auto view = dynamic_cast<osgViewer::View*>(&aa);
    if (!view) return false;

    switch (ea.getEventType()){
        case(osgGA::GUIEventAdapter::MOVE):
        {
            _mx = ea.getX();
            _my = ea.getY();

            return true;
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



void SelectEditor::pick(const osgGA::GUIEventAdapter& ea, osgViewer::View* view) {

    if (ea.getButton() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON) {
        double w = 1.5f;
        double h = 1.5f;

        osg::ref_ptr<osgUtil::PolytopeIntersector> picker = new osgUtil::PolytopeIntersector(
                osgUtil::Intersector::WINDOW, _mx - w, _my - h, _mx + w, _my + h);
        osgUtil::IntersectionVisitor iv(picker);

        //only intersect with vmap_node;
        root_node_->setSingleChildOn(root_node_->getChildIndex(vmap_node_));
        view->getCamera()->accept(iv);
        root_node_->setAllChildrenOn();

        //intersection check
        cleanUp();
        bool has_intersected = false;
        if (picker->containsIntersections()) {
            auto node_path = picker->getFirstIntersection().nodePath;

            std::string prefix("item_");
            for (const auto& node : node_path) {
                std::string nodeName = node->getName();

                if (!nodeName.compare(0, prefix.size(), prefix)) {
                    std::cout << node->libraryName() << " " << node->className() << " " << node->getName().c_str() << std::endl;

                    //outline effect
                    {
                        osg::ref_ptr<osgFX::Outline> outline = new osgFX::Outline;
                        temp_node_->addChild(outline.get());
                        outline->setWidth(6);
                        outline->addChild(node);
                    }

                    //start, end line check
                    std::string vector_prefix("vector");
                    if (!nodeName.compare(prefix.size(), vector_prefix.size(), vector_prefix))
                    {
                        int start_line_id, end_line_id;
                        start_line_id = end_line_id = 0;

                        node->getUserValue("start_line_id", start_line_id);
                        node->getUserValue("end_line_id", end_line_id);

                        std::cout << node->getName() << " start at: " << start_line_id<< " end at: " << end_line_id << std::endl;
                    }

                    //start, end lane check
                    std::string trace_prefix("trace");
                    if (!node->getName().compare(prefix.size(), trace_prefix.size(), trace_prefix))
                    {
                        int start_lane_id, end_lane_id;
                        start_lane_id = end_lane_id = 0;

                        node->getUserValue("start_lane_id", start_lane_id);
                        node->getUserValue("end_lane_id", end_lane_id);

                        std::cout << node->getName() << " start at: " << start_lane_id<< " end at: " << end_lane_id << std::endl;
                    }

                    has_intersected = true;
                }
            }
        }

        if (!has_intersected) {
            cleanUp();
        }
    }
}

void SelectEditor::cleanUp() {
    temp_node_->removeChildren(0, temp_node_->getNumChildren());
}