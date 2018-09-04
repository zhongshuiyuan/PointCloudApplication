//
// Created by WuKun on 9/4/18.
// Contact me:wk707060335@gmail.com
//

#ifndef POINTCLOUDAPPLICATION_SELECTEDITOR_H
#define POINTCLOUDAPPLICATION_SELECTEDITOR_H

#include <iostream>
#include <string>

#include <osg/Vec3d>
#include <osg/Vec4f>
#include <osg/Geode>
#include <osgViewer/View>
#include <osgGA/GUIEventHandler>
#include <osgGA/GUIEventAdapter>
#include <osgGA/GUIActionAdapter>

class SelectEditor : public osgGA::GUIEventHandler {
public:
    explicit SelectEditor(osg::Switch* root);
    ~SelectEditor() final;

    bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa) final;
    void pick(const osgGA::GUIEventAdapter& ea, osgViewer::View* view);

private:
    void cleanUp();

    osg::ref_ptr<osg::Switch> root_node_;
    osg::ref_ptr<osg::Switch> vmap_node_;
    osg::ref_ptr<osg::Switch> temp_node_;

    float _mx,_my;
};


#endif //POINTCLOUDAPPLICATION_SELECTEDITOR_H
