//
// Created by WuKun on 10/16/18.
// Contact me:wk707060335@gmail.com
//

#include <iostream>
#include <algorithm>
#include <limits.h>

#include "BezierCurve.h"


std::vector<Bezier::Point> Util::BezierCurveFactory::generate(const std::vector<Bezier::Point> &ctrl_points, int size) {
    BezierAdapter bezierAdapter(ctrl_points);
    std::vector<Bezier::Point> bezierPoints = bezierAdapter.getBezierPoints(size);

    return bezierPoints;
}

std::vector<osg::Vec3d> Util::BezierCurveFactory::make(const std::vector<osg::Vec3d> &seleted_points, int size) {
    //generate control points(2D)
    std::vector<Bezier::Point> control_points;
    control_points.reserve(seleted_points.size());
    for(const auto& point : seleted_points) {
        control_points.emplace_back(point.x(), point.y());
    }
    std::cout << "selected_points size: " << seleted_points.size() << std::endl;

    //get bezier points(2D)
    std::vector<Bezier::Point> bezier_points = generate(control_points, size);

    //generate result
    std::vector<osg::Vec3d> result;
    result.reserve(bezier_points.size());
    std::cout << "bezier_points size: " << bezier_points.size() << std::endl;
    for (int i = 0; i < bezier_points.size(); ++i) {
        const auto& bezier_point = bezier_points[i];

        //calculate height(linear interpolation)
        double h = 0;
        {
            int x = i / size;
            int y = i % size;
            double h1 = seleted_points[x].z();
            double h2 = seleted_points[x + 1].z();
            h = h1 + (h2 - h1) * (static_cast<double>(y) / static_cast<double>(size));
        }

        result.emplace_back(bezier_point.x, bezier_point.y, h);
    }

    return result;
}
