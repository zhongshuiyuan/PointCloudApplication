//
// Created by WuKun on 10/11/18.
// Contact me:wk707060335@gmail.com
//

#include "NodeNames.h"
#include "PositionTransformer.h"


PositionTransformer* PositionTransformer::instance= new PositionTransformer;

PositionTransformer *PositionTransformer::getInstance() {
    return instance;
}

osg::Vec3d PositionTransformer::convertXYZ2ENU(const osg::Vec3d& xyz_point) {
    if(!map_node_.valid()) return xyz_point;

    osg::Vec3d lla;
    map_node_->getMap()->getSRS()->getEllipsoid()->convertXYZToLatLongHeight(xyz_point.x(), xyz_point.y(), xyz_point.z(),
                                                                            lla.x(), lla.y(), lla.z());
    osg::Vec3d lla_degrees = osg::Vec3d(osg::RadiansToDegrees(lla.y()), osg::RadiansToDegrees(lla.x()), lla.z());

    osg::Vec3d enu_xyz;
    geodetic_converter_->geodetic2Enu(lla_degrees.y(), lla_degrees.x(), lla_degrees.z(), &enu_xyz.x(), &enu_xyz.y(), &enu_xyz.z());

    return enu_xyz;
}

PositionTransformer::PositionTransformer()
{
    geodetic_converter_ = new GeodeticConverter;
    geodetic_converter_->initialiseReference(ground_center_location.x(), ground_center_location.y(), ground_center_location.z());

    std::cout << "ground_center_location: " << ground_center_location.x() << " "
              << ground_center_location.y() << " " << ground_center_location.z() << std::endl;
}

void PositionTransformer::setMapNode(osg::ref_ptr<osgEarth::MapNode> &map_node) {
    map_node_ = map_node;
}

void PositionTransformer::printVec3d(const osg::Vec3d &point) {
    std::cout << point.x() << " "
              << point.y() << " " << point.z() << std::endl;
}

