//
// Created by WuKun on 8/30/18.
// Contact me:wk707060335@gmail.com
//

#ifndef POINTCLOUDAPPLICATION_TRACEEDITOR_H
#define POINTCLOUDAPPLICATION_TRACEEDITOR_H


#include <iostream>

#include <osg/Vec3d>
#include <osgViewer/View>
#include <osgGA/GUIEventHandler>
#include <osgGA/GUIEventAdapter>
#include <osgGA/GUIActionAdapter>
namespace m_map {
    class Lane;
    class Node;
}

class TraceEditor : public osgGA::GUIEventHandler
{
    typedef std::vector<std::pair<size_t, osg::Vec3d>> point_pair_vec;

public:
    explicit TraceEditor(osg::Switch* root_node);
    ~TraceEditor() final;

    bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa) final;
    void pick(const osgGA::GUIEventAdapter& ea, osgViewer::View* view);

private:
    void updateIndex();
    void cleanUp(bool all = true);
    bool isCurveLine(const point_pair_vec& points) const;
    std::vector<osg::Vec3d> calculateInterpolationPoints(const osg::Vec3d& start_point, const osg::Vec3d& end_point) const;
    bool clockwiseJudgement(const m_map::Lane& lane, const m_map::Node& node);
    bool isClockWiseOrNot(const osg::Vec3d& p1, const osg::Vec3d& p2, const osg::Vec3d& p3) const;

    osg::ref_ptr<osg::Switch> root_node_;
    osg::ref_ptr<osg::Switch> temp_node_;
    osg::ref_ptr<osg::Geode>  temp_line_geode_;

    point_pair_vec selected_points;
    size_t cur_point_index;

protected:
    float _mx,_my;
};


#endif //POINTCLOUDAPPLICATION_TRACEEDITOR_H
