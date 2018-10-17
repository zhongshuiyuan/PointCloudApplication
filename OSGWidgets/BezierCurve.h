//
// Created by WuKun on 10/16/18.
// Contact me:wk707060335@gmail.com
//

#ifndef POINTCLOUDAPPLICATION_BEZIER_H
#define POINTCLOUDAPPLICATION_BEZIER_H

#include <vector>

#include <osg/Vec3d>

#include "bezier.h"
#include "bezier_adapter.h"

namespace Util{
class BezierCurveFactory {
public:
    BezierCurveFactory() = delete;
    ~BezierCurveFactory() = delete;

    static std::vector<osg::Vec3d> make(const std::vector<osg::Vec3d>& seleted_points, int size = 100);
private:
    static std::vector<Bezier::Point> generate(const std::vector<Bezier::Point>& ctrl_points, int size);
};
}



#endif //POINTCLOUDAPPLICATION_BEZIER_H
