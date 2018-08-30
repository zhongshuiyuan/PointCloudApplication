//
// Created by WuKun on 8/27/18.
// Contact me:wk707060335@gmail.com
//

#ifndef POINTCLOUDAPPLICATION_NODETREEINFO_H
#define POINTCLOUDAPPLICATION_NODETREEINFO_H

#include <iostream>

#include <osg/NodeVisitor>
#include <osg/Switch>
#include <osg/Geode>
#include <osg/Node>
#include <osg/Camera>
#include <osgGA/GUIEventHandler>
#include <osgViewer/Viewer>


class NodeTreeInfo : public osg::NodeVisitor
{
public:
    NodeTreeInfo():osg::NodeVisitor(TRAVERSE_ALL_CHILDREN),
                   indent_(0)
    {
        std::cout << "--------------------------------------" << std::endl;
        std::cout << "Print NodeTreeInfo:" << std::endl;
    }

    void apply(osg::Switch& node) override
    {
        for (int i = 0; i < indent_; i++) std::cout << "  ";
        std::cout << "[" << indent_ + 1 << "]" << node.libraryName() << "::" << node.className() << "::" << node.getName() << std::endl;

        indent_++;
        traverse(node);
        indent_--;

    }
    void apply(osg::Geode& node) override
    {
        for (int i = 0; i < indent_; i++) std::cout << "  ";
        std::cout << "[" << indent_ + 1 << "]" << node.libraryName() << "::" <<
            node.className() << "::" << node.getName() << std::endl;

//        for (unsigned int n = 0; n < node.getNumDrawables(); n++){
//        	osg::Drawable* drawable = node.getDrawable(n);
//        	if (!drawable) continue;
//
//        	for (int i = 0; i <= indent_; i++)std::cout << "  ";
//        	std::cout << drawable->libraryName() << "::" << drawable->className() << "::" << drawable->getName() << std::endl;
//        }

        indent_++;
        traverse(node);
        indent_--;
    }

    void apply(osg::Camera& node) override
    {
        for (int i = 0; i < indent_; i++) std::cout << "  ";
        std::cout << "[" << indent_ + 1 << "]" << node.libraryName() << "::" << node.className() << "::" << node.getName() << std::endl;

        indent_++;
        traverse(node);
        indent_--;
    }

protected:
    int indent_;
};

class NodeTreeHandler : public osgGA::GUIEventHandler {
public:
    explicit NodeTreeHandler(osg::Switch* node):root_node_(node){}

    bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa) override
    {
        auto myview = dynamic_cast<osgViewer::View*>(&aa);
        if (!myview) return false;

        if (ea.getHandled()) return false;

        switch (ea.getEventType()) {
            case(osgGA::GUIEventAdapter::KEYDOWN) :
                if (ea.getKey() == 'n')
                {
                    NodeTreeInfo nodeTreeInfo;
                    if (root_node_) root_node_->accept(nodeTreeInfo);
                    return true;
                }
            default:
                break;
        }

        return false;

    }

protected:
    osg::ref_ptr<osg::Switch> root_node_;
};


#endif //POINTCLOUDAPPLICATION_NODETREEINFO_H
