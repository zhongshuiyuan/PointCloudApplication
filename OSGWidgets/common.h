//
// Created by WuKun on 8/29/18.
// Contact me:wk707060335@gmail.com
//

#ifndef POINTCLOUDAPPLICATION_COMMON_H
#define POINTCLOUDAPPLICATION_COMMON_H

#include <iostream>

#include <QDebug>

#include <osg/Vec3d>

std::ostream& operator<<(std::ostream& os, const osg::Vec3d& point){
    os << point.x() << ","
       << point.y() << ","
       << point.z();
    return os;
}

#endif //POINTCLOUDAPPLICATION_COMMON_H
