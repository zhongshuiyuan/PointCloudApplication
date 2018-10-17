//
// Created by WuKun on 8/30/18.
// Contact me:wk707060335@gmail.com
//

#include <vector>
#include <iomanip>   // for std::flags
#include <algorithm> // for copy
#include <iterator>  // for ostream_iterator
#include <math.h>    // for acos

#include <osg/ValueObject>
#include <osg/ShapeDrawable>
#include <osg/PositionAttitudeTransform>
#include <osgText/Text>

#include "NodeNames.h"
#include "BezierCurve.h"
#include "TraceEditor.h"
#include "VMapDrawable.h"
#include "DataStructure.h"
#include "NodeTreeSearch.h"
#include "PositionTransformer.h"
#include "../Common/tracer.h"
#include "../Common/VectorMapSingleton.h"
using m_map::Point;
using m_map::Node;
using m_map::Lane;
using m_map::dtLane;

#define PI 3.14159265

TraceEditor::TraceEditor(osg::Switch* root_node) :
        root_node_(root_node),
        temp_node_(nullptr),
        temp_line_geode_(nullptr),
        _mx(0),
        _my(0) {

    temp_node_ = dynamic_cast<osg::Switch*>(NodeTreeSearch::findNodeWithName(root_node, temp_node_name));
}

TraceEditor::~TraceEditor() {
    cleanUp();
}

bool TraceEditor::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa){
    auto view = dynamic_cast<osgViewer::View*>(&aa);
    if (!view) return false;

    switch (ea.getEventType()){
        case (osgGA::GUIEventAdapter::KEYDOWN):
        {
            if (ea.getKey() == osgGA::GUIEventAdapter::KEY_Escape)
            {
                cleanUp();
                return true;
            }
            break;
        }
        case(osgGA::GUIEventAdapter::MOVE):
        {
            _mx = ea.getX();
            _my = ea.getY();

            if (selected_points.empty()) return false;

            //draw mouse follow line
            osg::ref_ptr<osgUtil::LineSegmentIntersector> picker = new osgUtil::LineSegmentIntersector
                    (osgUtil::Intersector::PROJECTION, ea.getXnormalized(), ea.getYnormalized());
            osgUtil::IntersectionVisitor iv(picker.get());

            view->getCamera()->accept(iv);

            if (picker->containsIntersections())
            {
                auto intersection = picker->getFirstIntersection();
                osg::Vec3d curPoint = intersection.getWorldIntersectPoint(); //xyz
                osg::Vec3d firstPoint = std::get<1>(selected_points.back()); //enu
                curPoint = PositionTransformer::getInstance()->convertXYZ2ENU(curPoint); //enu

                osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
                vertices->push_back(firstPoint);
                vertices->push_back(curPoint);

                if (nullptr == temp_line_geode_)
                {
                    temp_line_geode_ = new osg::Geode;
                    temp_line_geode_->setName("tmpLineGeode");
                    temp_node_->addChild(temp_line_geode_);
                }
                else
                {
                    temp_line_geode_->removeDrawables(0, temp_line_geode_->getNumDrawables());
                }

                osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
                geom->setName("tmpLineGeom");
                temp_line_geode_->addDrawable(geom);

                geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, vertices->size()));

                osg::ref_ptr<osg::Vec3Array> colors = new osg::Vec3Array;
                colors->push_back(osg::Vec3(1.0, 0.0, 0.0));

                geom->setColorArray(colors.get());
                geom->setColorBinding(osg::Geometry::BIND_OVERALL);
                geom->setVertexArray(vertices.get());
            } //end draw

            break;
        }
        case(osgGA::GUIEventAdapter::RELEASE):
        {
            if (_mx == ea.getX() && _my == ea.getY())
            {
                // only do a pick if the mouse hasn't moved
                pick(ea,view);
                return true;
            }
            break;
        }
        default:
            break;
    }

    return false;
}

void TraceEditor::pick(const osgGA::GUIEventAdapter& ea, osgViewer::View* view) {

    if (selected_points.empty()) {
        updateIndex();
        cleanUp();
    }

    if (ea.getButton() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON) {
        osg::ref_ptr<osgUtil::LineSegmentIntersector> picker = new osgUtil::LineSegmentIntersector
                (osgUtil::Intersector::PROJECTION, ea.getXnormalized(), ea.getYnormalized());
        osgUtil::IntersectionVisitor iv(picker.get());

        view->getCamera()->accept(iv);

        //intersection check
        if (picker->containsIntersections()) {
            osg::Vec3d local_point;
            int local_point_index = 0;

            bool is_connected = false;
            auto all_intersection = picker->getIntersections();
            for (const auto& intersection : all_intersection) {
                auto child_node = intersection.nodePath.back();
                if (child_node->getName() == "node") {
//                    std::cout << "connect!" << std::endl;
                    is_connected = true;
                    child_node->getUserValue("pos", local_point);
                    child_node->getUserValue("id", local_point_index);
                    break;
                }
            }
            if (!is_connected) {
                osg::Vec3d intersectPoint = picker->getIntersections().begin()->getWorldIntersectPoint();
                local_point = PositionTransformer::getInstance()->convertXYZ2ENU(intersectPoint);
                local_point_index = cur_point_index++;

                osg::ref_ptr<osg::Geode> node_geode = new osg::Geode;
                node_geode->setName("node");
                node_geode->setUserValue("pos", local_point);
                node_geode->setUserValue("id", local_point_index);

                osg::ref_ptr<osg::ShapeDrawable> node_sphere = new osg::ShapeDrawable(new osg::Sphere(local_point, 0.1f));
                node_sphere->setColor(osg::Vec4f(1.0, 1.0, 0.0, 1.0));
                node_geode->addDrawable(node_sphere);
                temp_node_->addChild(node_geode);

            }
            std::cout << "local point: " << local_point_index << " " << local_point.x() << " "
                << local_point.y() << " " << local_point.z() << std::endl;
            selected_points.emplace_back(std::make_pair(local_point_index, local_point));
        }

        //draw temp line
        if (selected_points.size() >= 2)
        {
            osg::Vec3d penultimate_point = std::get<1>(*(selected_points.end() - 2));
            osg::Vec3d last_point = std::get<1>(selected_points.back());

            osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
            vertices->push_back(penultimate_point);
            vertices->push_back(last_point);

            osg::ref_ptr<osg::Vec3Array> colors = new osg::Vec3Array;
            colors->push_back(osg::Vec3(1.0, 1.0, 0.0));

            osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
            geom->setName("solidLineGeom");
            geom->setVertexArray(vertices.get());
            geom->setColorArray(colors.get());
            geom->setColorBinding(osg::Geometry::BIND_OVERALL);
            geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, 2));

            osg::ref_ptr<osg::Geode> geode = new osg::Geode;
            geode->setName("solidLineGeode");
            geode->addDrawable(geom);

            temp_node_->addChild(geode);
        } // end draw
    } //left button

    //right button
    if (ea.getButton() == osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON && selected_points.size() >= 2)
    {
        //bezier interpolation
        if(false)
        {
            int interval = 10;
            std::vector<osg::Vec3d> points;
            for (const auto& selected_point : selected_points) {
                points.push_back(std::get<1>(selected_point));
            }

            auto bezier_points = Util::BezierCurveFactory::make(points, interval);

            point_pair_vec dense_selected_points;
            for (int i = 0; i < selected_points.size() - 1; ++i) {
                dense_selected_points.push_back(selected_points[i]);
                for (int j = 1; j < interval - 1; ++j) {
                    osg::Vec3d point = bezier_points[i * interval + j];
                    dense_selected_points.emplace_back(std::make_pair(cur_point_index++, point));
                }
            }
            dense_selected_points.push_back(selected_points.back());
            dense_selected_points.swap(selected_points);
        }

        //equally spaced interpolation
        if(true)
        {
            point_pair_vec tmp_selected_points;
            for (int i = 0; i < selected_points.size() - 1; ++i) {
                const osg::Vec3d& start_point = std::get<1>(selected_points[i]);
                const osg::Vec3d& end_point = std::get<1>(selected_points[i + 1]);

                std::vector<osg::Vec3d> interpolated_points = calculateInterpolationPoints(start_point, end_point);

                point_pair_vec dense_selected_points;
                dense_selected_points.push_back(selected_points[i]);
                for (auto point: interpolated_points) {
                    dense_selected_points.emplace_back(std::make_pair(cur_point_index++, point));
                }
                tmp_selected_points.insert(tmp_selected_points.end(), dense_selected_points.begin(), dense_selected_points.end());
            }
            tmp_selected_points.push_back(selected_points.back());
            tmp_selected_points.swap(selected_points);
        }

        //successively output
        if (false)
        {
            std::ofstream ofs("test.txt", std::ios::app);

            ofs.setf(std::ios::fixed);;
            for (const auto& pair : selected_points) {
                const osg::Vec3d& p = std::get<1>(pair);
                ofs << std::setprecision(2) << p.x() << "," << p.y() << "," << p.z() << std::endl;
            }
            ofs.close();
        }

        //update point, node
        std::vector<Point> points;
        std::vector<Node> nodes;
        {
            size_t cur_min_point_index = VectorMapSingleton::getInstance()->getMaxPointIndex() + 1;

            for (const auto& pair : selected_points) {
                size_t node_id = std::get<0>(pair);
                size_t point_id = cur_min_point_index++; //TODO don't judge point's exist
                osg::Vec3d pos = std::get<1>(pair);

                //reverse x y
                points.emplace_back(point_id, pos.y(), pos.x(), pos.z());
                nodes.emplace_back(node_id, point_id);
            }
            VectorMapSingleton::getInstance()->update(points);
            VectorMapSingleton::getInstance()->update(nodes);
        }

        //update lane, dtlane
        //lane
        std::vector<Lane> lanes;
        std::vector<dtLane> dtlanes;
        std::vector<Lane> update_lanes; //former lanes
        {
            bool is_curve = isCurveLine(selected_points);
            int dir = 0;

            size_t cur_min_lane_index = VectorMapSingleton::getInstance()->getMaxLaneIndex() + 1;
            for (int i = 0; i < nodes.size() - 1; ++i) {
                const Node& backward_node = nodes[i];
                const Node& forward_node = nodes[i + 1];

                size_t backward_lane_id = (i == 0 ? 0 : cur_min_lane_index - 1);
                size_t forward_lane_id  = (i == nodes.size() - 2 ? 0 :cur_min_lane_index + 1);

                //backward connection
                if (i == 0) {
                    //search connected lane
                    std::vector<Lane> connected_lanes = VectorMapSingleton::getInstance()->findByFilter([&](const Lane& lane) {
                        return lane.fnid == backward_node.nid;
                    });

                    //update connected lane
                    if (!connected_lanes.empty()) {
                        Lane& connected_lane = connected_lanes.back();

                        //clockwise judgement
                        size_t jct = connected_lane.jct;
                        {
                            bool is_clockwise = clockwiseJudgement(connected_lane, forward_node);
                            if (is_clockwise) {
                                if (jct == 1) jct = 5;
                                else jct = 2;
                            } else {
                                if (jct == 2) jct = 5;
                                else jct = 1;
                            }
                        }

                        if (connected_lane.flid == 0) {
                            connected_lane.flid = cur_min_lane_index;
                        }
                        else if (connected_lane.flid2 == 0) {
                            connected_lane.flid2 = cur_min_lane_index;
                            connected_lane.jct = jct;
                        }
                        else {
                            connected_lane.flid3 = cur_min_lane_index;
                            connected_lane.jct = jct;
                        }

                        backward_lane_id = connected_lane.lnid;
                        std::cout << "connected lane: " << connected_lane << std::endl;
                        update_lanes.push_back(connected_lane);
                    }
                }

                //forward connection
                if (i == nodes.size() - 2) {
                    std::vector<Lane> connected_lanes = VectorMapSingleton::getInstance()->findByFilter([&](const Lane& lane) {
                        return lane.bnid == forward_node.nid;
                    });

                    if (!connected_lanes.empty()) {
                        Lane& connected_lane = connected_lanes.back();
                        if (connected_lane.blid == 0) { connected_lane.blid = cur_min_lane_index; }
                        else if (connected_lane.blid2 == 0) { connected_lane.blid2 = cur_min_lane_index; }
                        else { connected_lane.blid3 = cur_min_lane_index; }

                        //clockwise judgement
                        size_t jct = connected_lane.jct;
                        {
                            bool is_clockwise = clockwiseJudgement(connected_lane, backward_node);
                            if (is_clockwise) {
                                if (jct == 3) jct = 5;
                                else jct = 4;
                            } else {
                                if (jct == 4) jct = 5;
                                else jct = 3;
                            }
                        }

                        connected_lane.jct = jct;
                        forward_lane_id = connected_lane.lnid;
                        std::cout << "connected lane: " << connected_lane << std::endl;
                        update_lanes.push_back(connected_lane);
                    }
                }

                Lane cur_lane(cur_min_lane_index, cur_min_lane_index, backward_lane_id, forward_lane_id, backward_node.nid, forward_node.nid);
                if (i == 0) cur_lane.start_end_tag = 1;
                else if (i == nodes.size() - 2) cur_lane.start_end_tag = 2;
                lanes.push_back(cur_lane);

                //TODO update connected dtlanes
                double apara = 0.0;
                double r = RADIUS_MAX;
                if (is_curve)  r = 9.9;

                dtlanes.emplace_back(cur_min_lane_index, dir++, backward_node.pid, apara, r);
                cur_min_lane_index++;
            }

            VectorMapSingleton::getInstance()->update(lanes);
            VectorMapSingleton::getInstance()->update(dtlanes);
            VectorMapSingleton::getInstance()->update(update_lanes);
        }

        //redraw node, lane
        {
            VMapDrawable vMapDrawable(root_node_);
            vMapDrawable.drawTraceNode(lanes, Lane());
        }

        //debug
        if (false) {
            std::cout << "--------result---------" << std::endl;
            for (auto node : nodes)
                std::cout << "node: " << node << std::endl;
            for (auto lane : lanes)
                std::cout << "lane: " << lane << std::endl;
            for (auto dtlane : dtlanes)
                std::cout << "dtlane: " << dtlane << std::endl;
        }

        cleanUp();
    } //right button
}

void TraceEditor::cleanUp() {
    if (temp_line_geode_)
    {
        temp_line_geode_->removeDrawables(0, temp_line_geode_->getNumDrawables());
        temp_line_geode_ = nullptr;
    }
    temp_node_->removeChildren(0, temp_node_->getNumChildren());

    selected_points.clear();
    selected_points.shrink_to_fit();
}

void TraceEditor::updateIndex() {
    //update cur_point_index in case lineEditor add new points!
    cur_point_index = VectorMapSingleton::getInstance()->getMaxNodeIndex() + 1;
}

std::vector<osg::Vec3d> TraceEditor::calculateInterpolationPoints(const osg::Vec3d& start_point, const osg::Vec3d& end_point) const {
    std::vector<osg::Vec3d> points;

    osg::Vec3d direction = end_point - start_point;
    //direction.z() = 0;
    double distance = direction.length();
    direction.normalize();

    if (distance > 1.0) {
        for (int i = 1; i < distance; ++i) {
            osg::Vec3d point = start_point + direction * i;
            points.push_back(point);
        }
    }

    return points;
}

bool TraceEditor::isCurveLine(const point_pair_vec& points) const {
    if (points.size() < 3) return false;

    osg::Vec3d p1 = std::get<1>(*points.begin());
    osg::Vec3d p2 = std::get<1>(*(points.begin() + 1));
    osg::Vec3d dir1 = p2 - p1;
    dir1.normalize();

    osg::Vec3d p3 = std::get<1>(*(points.end() - 2));
    osg::Vec3d p4 = std::get<1>(*(points.end() - 1));
    osg::Vec3d dir2 = p4 - p3;
    dir2.normalize();

    double alpha = dir1 * dir2;
    alpha = (alpha > 0.999) ? 0.999 : (alpha < -0.999) ? -0.999 : alpha;

    double angle = std::acos(alpha) * 180 / PI;
    printf ("The arc cosine of %f is %f degrees.\n", alpha, angle);

    return (angle > 30 or angle < -30);

}

bool TraceEditor::isClockWiseOrNot(const osg::Vec3d &p1, const osg::Vec3d &p2, const osg::Vec3d &p3) const {
    osg::Vec3d p12 = p2 - p1;
    osg::Vec3d p13 = p3 - p1;
    osg::Vec3d normal = p12 ^ p13;

    //x - y is reversed!
    return normal.z() > 0;
}

bool TraceEditor::clockwiseJudgement(const m_map::Lane &lane, const m_map::Node &node) {
    bool is_clockwise = true;

    Node n1 = VectorMapSingleton::getInstance()->findByID(Key<Node>(lane.bnid));
    Node n2 = VectorMapSingleton::getInstance()->findByID(Key<Node>(lane.fnid));
    Node n3 = node;

    Point p1 = VectorMapSingleton::getInstance()->findByID(Key<Point>(n1.pid));
    Point p2 = VectorMapSingleton::getInstance()->findByID(Key<Point>(n2.pid));
    Point p3 = VectorMapSingleton::getInstance()->findByID(Key<Point>(n3.pid));

    if (p1.pid > 0 and p2.pid > 0 and p3.pid > 0) {
        osg::Vec3d v1(p1.bx, p1.ly, p1.h);
        osg::Vec3d v2(p2.bx, p2.ly, p2.h);
        osg::Vec3d v3(p3.bx, p3.ly, p3.h);

        is_clockwise = isClockWiseOrNot(v1, v2, v3);
    }

    return is_clockwise;
}
