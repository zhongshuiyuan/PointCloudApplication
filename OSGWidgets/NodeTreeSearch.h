//
// Created by WuKun on 8/29/18.
// Contact me:wk707060335@gmail.com
//

#ifndef POINTCLOUDAPPLICATION_NODETREESEARCH_H
#define POINTCLOUDAPPLICATION_NODETREESEARCH_H

#include <string>
#include <iostream>

#include <osg/Node>
#include <osg/Switch>
#include <osg/NodeVisitor>

class NodeTreeSearch: public osg::NodeVisitor {
public:
    explicit NodeTreeSearch(const char name[]):
            osg::NodeVisitor(TRAVERSE_ALL_CHILDREN),
            node_(nullptr),
            name_(name) {
    }

    void apply(osg::Switch &search_node) override
    {
        if(search_node.getName() == name_)
        {
            node_ = &search_node;
        }
        traverse(search_node);
    }

    void apply(osg::Node &search_node) override
    {
        if(search_node.getName() == name_)
        {
            node_ = &search_node;
        }
        traverse(search_node);
    }

    osg::Node* getNode()
    {
        return node_;
    }

    static osg::Node* findNodeWithName(osg::Switch* root_node, const char name[]) {
        auto visitor= new NodeTreeSearch(name);
        root_node->accept(*visitor);
        auto node = visitor->getNode();

        if (!node) std::cout << "aha?" << std::endl;

        return node;
    }

private:
    osg::Node*  node_;
    std::string name_;
};

#endif //POINTCLOUDAPPLICATION_NODETREESEARCH_H
