//
// Created by WuKun on 8/31/18.
// Contact me:wk707060335@gmail.com
//

#ifndef POINTCLOUDAPPLICATION_TEXTCONTROLLER_H
#define POINTCLOUDAPPLICATION_TEXTCONTROLLER_H

#include <iostream>

#include <osgGA/GUIEventHandler>
#include <osgGA/GUIEventAdapter>
#include <osgGA/GUIActionAdapter>
#include <osgViewer/View>

#include "common.h"
#include "NodeTreeSearch.h"
#include "../Common/tracer.h"

class TextController : public osgGA::GUIEventHandler
{
public:
    explicit TextController(osg::ref_ptr<osg::Switch> root) :
        root_(root) {
        point_text_node_  = dynamic_cast<osg::Switch*>(NodeTreeSearch::findNodeWithName(root_, point_text_node_name));
        line_text_node_   = dynamic_cast<osg::Switch*>(NodeTreeSearch::findNodeWithName(root_, line_text_node_name));
        area_text_node_   = dynamic_cast<osg::Switch*>(NodeTreeSearch::findNodeWithName(root_, area_text_node_name));
        node_text_node_   = dynamic_cast<osg::Switch*>(NodeTreeSearch::findNodeWithName(root_, node_text_node_name));
        lane_text_node_   = dynamic_cast<osg::Switch*>(NodeTreeSearch::findNodeWithName(root_, lane_text_node_name));
        dtlane_text_node_ = dynamic_cast<osg::Switch*>(NodeTreeSearch::findNodeWithName(root_, dtlane_text_node_name));
    }
    ~TextController() final = default;

    bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa) final {
        auto view = dynamic_cast<osgViewer::View*>(&aa);
        if (!view) return false;

        switch (ea.getEventType()){
            case (osgGA::GUIEventAdapter::KEYDOWN):
            {
                if (ea.getKey() == '1'){
                    reverseNodeMask(point_text_node_);
                }
                if (ea.getKey() == '2'){
                    reverseNodeMask(line_text_node_);
                }
                if (ea.getKey() == '3'){
                    reverseNodeMask(area_text_node_);
                }
                if (ea.getKey() == '4'){
                    reverseNodeMask(node_text_node_);
                }
                if (ea.getKey() == '5'){
                    reverseNodeMask(lane_text_node_);
                }
                if (ea.getKey() == '6'){
                    reverseNodeMask(dtlane_text_node_);
                }
            }
            default:
                return false;
        }
    }

private:
    inline void reverseNodeMask(osg::ref_ptr<osg::Switch> node) {
        if (!node.valid() || node->getNumChildren() == 0) return;

        auto node_mask = node->getNodeMask();
        auto new_mask = node_mask == 0 ? 1 : 0;
        node->setNodeMask(new_mask);
    }

    osg::ref_ptr<osg::Switch> root_;
    osg::ref_ptr<osg::Switch> point_text_node_;
    osg::ref_ptr<osg::Switch> line_text_node_;
    osg::ref_ptr<osg::Switch> area_text_node_;
    osg::ref_ptr<osg::Switch> node_text_node_;
    osg::ref_ptr<osg::Switch> lane_text_node_;
    osg::ref_ptr<osg::Switch> dtlane_text_node_;

};

#endif //POINTCLOUDAPPLICATION_TEXTCONTROLLER_H
