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
#include <osgText/Text>

#include "TraceEditor.h"
#include "common.h"
#include "NodeTreeSearch.h"
#include "../Common/tracer.h"
#include "../Common/VectorMapSingleton.h"
#include "DataStructure.h"
using m_map::Point;

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
            double w = 1.5f;
            double h = 1.5f;

            osg::ref_ptr<osgUtil::PolytopeIntersector> picker = new osgUtil::PolytopeIntersector(osgUtil::Intersector::WINDOW, _mx - w, _my - h, _mx + w, _my + h);
            osgUtil::IntersectionVisitor iv(picker);
            view->getCamera()->accept(iv);

            if (picker->containsIntersections())
            {
                auto iter = picker->getIntersections().begin();
                if (iter != picker->getIntersections().end())
                {
                    osg::Vec3 curPoint(iter->localIntersectionPoint);

                    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
                    vertices->push_back(std::get<1>(selected_points.back()));
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

                    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_LOOP, 0, vertices->size()));

                    osg::ref_ptr<osg::Vec3Array> colors = new osg::Vec3Array;
                    colors->push_back(osg::Vec3(1.0, 0.0, 0.0));

                    geom->setColorArray(colors.release());
                    geom->setColorBinding(osg::Geometry::BIND_OVERALL);
                    geom->setVertexArray(vertices.get());
                }
            } //end draw

            return false;
        }
        case(osgGA::GUIEventAdapter::RELEASE):
        {
            if (_mx == ea.getX() && _my == ea.getY())
            {
                // only do a pick if the mouse hasn't moved
                pick(ea,view);
            }
            return true;
        }

        default:
            return false;
    }
}

void TraceEditor::pick(const osgGA::GUIEventAdapter& ea, osgViewer::View* view) {

    if(selected_points.empty()) updateIndex();

    if (ea.getButton() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON) {
        double w = 3.0f;
        double h = 3.0f;

        osg::ref_ptr<osgUtil::PolytopeIntersector> picker = new osgUtil::PolytopeIntersector(
                osgUtil::Intersector::WINDOW, _mx - w, _my - h, _mx + w, _my + h);
        osgUtil::IntersectionVisitor iv(picker);
        view->getCamera()->accept(iv);

        //intersection check
        if (picker->containsIntersections()) {
            osg::NodePath all_node_path = picker->getFirstIntersection().nodePath;

            //first intersection
            auto iter = picker->getIntersections().begin();
            if (iter != picker->getIntersections().end())
            {
                osg::Vec3d local_point(iter->localIntersectionPoint);
                int local_point_index = 0;

                //Is it connected with former node ?
                auto child_node = all_node_path.back();
                if (child_node->getName() == "node") {
                    child_node->getUserValue("pos", local_point);
                    child_node->getUserValue("id", local_point_index);
                }
                else
                {
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

                std::cout << "local point: " << local_point_index << std::endl;
                selected_points.emplace_back(std::make_pair(local_point_index, local_point));
            }
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
        //interpolation
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

        //update point, node
        std::vector<Point> points;
        std::vector<Node> nodes;
        {
            size_t cur_min_point_index = VectorMapSingleton::getInstance()->getMaxPointIndex() + 1;

            for (const auto& pair : selected_points) {
                size_t node_id = std::get<0>(pair);
                size_t point_id = cur_min_point_index++; //TODO don't judge point's exist
                osg::Vec3d pos = std::get<1>(pair);

                points.emplace_back(point_id, pos.x(), pos.y(), pos.z());
                nodes.emplace_back(node_id, point_id);
            }
            VectorMapSingleton::getInstance()->update(points);
            VectorMapSingleton::getInstance()->update(nodes);
        }

        //update lane, dtlane
        //lane
        std::vector<Lane> lanes;
        std::vector<dtLane> dtlanes;
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
                        if (lane.fnid == backward_node.nid) return true;
                        else return false;
                    });

                    //update connected lane
                    if (!connected_lanes.empty()) {
                        Lane& connected_lane = connected_lanes.back();
                        if (connected_lane.flid == 0) { connected_lane.flid = cur_min_lane_index; }
                        else if (connected_lane.flid2 == 0) { connected_lane.flid2 = cur_min_lane_index; }
                        else { connected_lane.flid3 = cur_min_lane_index; }

                        backward_lane_id = connected_lane.lnid;
                        std::cout << "connected lane: " << connected_lane << std::endl;
                        lanes.push_back(connected_lane);
                    }
                }

                //forward connection
                if (i == nodes.size() - 2) {
                    std::vector<Lane> connected_lanes = VectorMapSingleton::getInstance()->findByFilter([&](const Lane& lane) {
                        if (lane.bnid == forward_node.nid) return true;
                        else return false;
                    });

                    if (!connected_lanes.empty()) {
                        Lane& connected_lane = connected_lanes.back();
                        if (connected_lane.blid == 0) { connected_lane.blid = cur_min_lane_index; }
                        else if (connected_lane.blid2 == 0) { connected_lane.blid2 = cur_min_lane_index; }
                        else { connected_lane.blid3 = cur_min_lane_index; }

                        forward_lane_id = connected_lane.lnid;
                        std::cout << "connected lane: " << connected_lane << std::endl;
                        lanes.push_back(connected_lane);
                    }
                }

                lanes.emplace_back(cur_min_lane_index, cur_min_lane_index, backward_lane_id, forward_lane_id, backward_node.nid, forward_node.nid);

                //TODO update connected dtlanes
                double apara = 0.0;
                double r = RADIUS_MAX;
                if (is_curve)  r = 9.9;

                dtlanes.emplace_back(cur_min_lane_index, dir++, backward_node.pid, apara, r);
                cur_min_lane_index++;
            }

            VectorMapSingleton::getInstance()->update(lanes);
            VectorMapSingleton::getInstance()->update(dtlanes);
        }

        //redraw line, point
        {
            osg::ref_ptr<osg::Switch> trace_item_node = dynamic_cast<osg::Switch*>(NodeTreeSearch::findNodeWithName(root_node_, trace_item_node_name));

            static int node_index = 0;
            osg::ref_ptr<osg::Switch> oneDrawNode = new osg::Switch;
            oneDrawNode->setName("trace" + std::to_string(node_index++));
            trace_item_node->addChild(oneDrawNode);
            //line
            {
                osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
                for (const auto& point : points) {
                    vertices->push_back(osg::Vec3d(point.bx, point.ly, point.h));
                }

                osg::ref_ptr<osg::Vec3Array> colors = new osg::Vec3Array;
                colors->push_back(osg::Vec3(1.0, 1.0, 0.0));

                osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
                geom->setVertexArray(vertices.get());
                geom->setColorArray(colors.get());
                geom->setColorBinding(osg::Geometry::BIND_OVERALL);
                geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP, 0, vertices->size()));

                osg::ref_ptr<osg::Geode> geode = new osg::Geode;
                geode->setName("solidLineGeode");
                geode->addDrawable(geom);

                oneDrawNode->addChild(geode);
            }

            //point
            {
                for (int i = 0; i < nodes.size(); ++i) {
                    int local_point_index = nodes[i].nid;
                    const Point& point = points[i];
                    osg::Vec3d local_point(point.bx, point.ly, point.h);

                    osg::ref_ptr<osg::Geode> point_geode = new osg::Geode;
                    point_geode->setName("node");
                    point_geode->setUserValue("id", local_point_index);
                    point_geode->setUserValue("pos", local_point);

                    osg::ref_ptr<osg::ShapeDrawable> point_sphere = new osg::ShapeDrawable(new osg::Sphere(local_point, 0.1f));
                    point_sphere->setColor(osg::Vec4f(1.0, 1.0, 0.0, 1.0));
                    point_geode->addDrawable(point_sphere);
                    oneDrawNode->addChild(point_geode);
                }
            }
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

        //text
        {
            osg::ref_ptr<osg::Switch> node_text_node = dynamic_cast<osg::Switch*>(NodeTreeSearch::findNodeWithName(root_node_, node_text_node_name));
            for (const auto& node: nodes) {
                const auto& point = VectorMapSingleton::getInstance()->findByID(Key<Point>(node.pid));

                osg::Vec3d pos(point.bx, point.ly, point.h);
                std::string name = std::to_string(node.nid);
                osg::Vec4f color(0.0, 1.0, 0.0, 0.5);

                node_text_node->addChild(drawTextGeode(pos, name, color));
            }

            osg::ref_ptr<osg::Switch> lane_text_node = dynamic_cast<osg::Switch*>(NodeTreeSearch::findNodeWithName(root_node_, lane_text_node_name));
            for (const auto& lane : lanes) {
                const auto& backward_node = VectorMapSingleton::getInstance()->findByID(Key<Node>(lane.bnid));
                const auto& forward_node = VectorMapSingleton::getInstance()->findByID(Key<Node>(lane.fnid));

                const auto& backward_point = VectorMapSingleton::getInstance()->findByID(Key<Point>(backward_node.pid));
                const auto& forward_point = VectorMapSingleton::getInstance()->findByID(Key<Point>(forward_node.pid));

                osg::Vec3d pos1(backward_point.bx, backward_point.ly, backward_point.h);
                osg::Vec3d pos2(forward_point.bx, forward_point.ly, forward_point.h);

                osg::Vec3d pos = ( pos1 + pos2 ) / 2;
                std::string name = std::to_string(lane.lnid);
                osg::Vec4f color(1.0, 0.0, 0.0, 0.5);

                lane_text_node->addChild(drawTextGeode(pos, name, color));
            }
        }

        //emit
        {
        }

        cleanUp(true);
    } //right button
}

void TraceEditor::cleanUp(bool all) {
    if (temp_line_geode_)
    {
        temp_line_geode_->removeDrawables(0, temp_line_geode_->getNumDrawables());
        temp_line_geode_ = nullptr;
    }
    if (all) temp_node_->removeChildren(0, temp_node_->getNumChildren());

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

    if (angle < 30 and angle > -30) return false;

    return true;
}


osg::ref_ptr<osg::Geode> TraceEditor::drawTextGeode(const osg::Vec3d& pos,
                                       const std::string& content, const osg::Vec4f& color) const {
    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    geode->setName(content);

    osg::ref_ptr<osgText::Text> text = new osgText::Text;
    text->setCharacterSize(0.2);
    text->setAxisAlignment( osgText::TextBase::XY_PLANE );
    text->setPosition(pos);
    text->setText(content);
    text->setColor(color);

    geode->addDrawable(text);
    return geode.release();
}