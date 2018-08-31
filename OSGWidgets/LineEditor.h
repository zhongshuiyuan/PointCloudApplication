//
// Created by WuKun on 8/29/18.
// Contact me:wk707060335@gmail.com
//

#ifndef POINTCLOUDAPPLICATION_LINEEDITOR_H
#define POINTCLOUDAPPLICATION_LINEEDITOR_H

#include <iostream>
#include <string>

#include <osg/Vec3d>
#include <osg/Vec4f>
#include <osg/Geode>
#include <osgViewer/View>
#include <osgGA/GUIEventHandler>
#include <osgGA/GUIEventAdapter>
#include <osgGA/GUIActionAdapter>

class LineEditor : public osgGA::GUIEventHandler
{
public:
    explicit LineEditor(osg::Switch* root_node);
    ~LineEditor() final;

    bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa) final;
    void pick(const osgGA::GUIEventAdapter& ea, osgViewer::View* view);

private:
    void updateIndex();
    void cleanUp(bool all = true);
    osg::ref_ptr<osg::Geode> drawTextGeode(const osg::Vec3d& pos,
            const std::string& content, const osg::Vec4f& color) const;

    osg::ref_ptr<osg::Switch> root_node_;
    osg::ref_ptr<osg::Switch> temp_node_;
    osg::ref_ptr<osg::Geode>  temp_line_geode_;

    std::vector<std::pair<size_t, osg::Vec3d>> selected_points;
    size_t cur_point_index;

protected:
    float _mx,_my;
};

std::ostream& operator<<(std::ostream& os, const osg::Vec3d& point);

#endif //POINTCLOUDAPPLICATION_LINEEDITOR_H
