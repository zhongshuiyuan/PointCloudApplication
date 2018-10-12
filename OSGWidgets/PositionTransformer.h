//
// Created by WuKun on 10/11/18.
// Contact me:wk707060335@gmail.com
//

#ifndef POINTCLOUDAPPLICATION_POSITIONTRANSFORMER_H
#define POINTCLOUDAPPLICATION_POSITIONTRANSFORMER_H

#include <osg/Vec3d>
#include <osg/ref_ptr>
#include <osgEarth/MapNode>

#include "ENUCoorConv.hpp"
using geodetic_converter::GeodeticConverter;

class PositionTransformer {
private:
    PositionTransformer();
    ~PositionTransformer() = default;

    static PositionTransformer* instance;
    GeodeticConverter* geodetic_converter_;
    osg::ref_ptr<osgEarth::MapNode> map_node_;
public:
    void setMapNode(osg::ref_ptr<osgEarth::MapNode>& map_node);
    void printVec3d(const osg::Vec3d& point);

    static PositionTransformer* getInstance();

    osg::Vec3d convertXYZ2ENU(const osg::Vec3d& xyz_point);
};


#endif //POINTCLOUDAPPLICATION_POSITIONTRANSFORMER_H
