//
// Created by WuKun on 9/6/18.
// Contact me:wk707060335@gmail.com
//

#ifndef POINTCLOUDAPPLICATION_VMAPDRAWABLE_H
#define POINTCLOUDAPPLICATION_VMAPDRAWABLE_H

#include <vector>

#include <QStringList>

#include <osg/Geode>
#include <osg/Switch>
#include <osg/Geometry>
#include <osg/ShapeDrawable>

#include "NodeNames.h"
#include "NodeTreeSearch.h"
#include "../Common/DataStructure.h"
#include "../Common/VectorMapSingleton.h"

using m_map::Point;
using m_map::Line;
using m_map::Node;
using m_map::Lane;
using m_map::RoadEdge;
using m_map::StopLine;
using m_map::CrossWalk;

class VMapDrawable {

public:
    explicit VMapDrawable(osg::Switch* root);
    ~VMapDrawable() = default;

    //public template function must be defined in .h file.
    //meanwhile the function definition must see the parameters' definition(so only forward declaration is inadequate)
    template <class T>
    void drawVectorNode(const std::vector<Line>& lines, const T& object) {
        if (lines.empty()) return;

        osg::ref_ptr<osg::Switch> vector_item_node =
                dynamic_cast<osg::Switch*>(NodeTreeSearch::findNodeWithName(root_node_, vector_item_node_name));

        //points
        std::vector<Point> points;
        for (const Line& line : lines) {
            Point start_point = VectorMapSingleton::getInstance()->findByID(Key<Point>(line.bpid));
            points.push_back(start_point);
        }
        Point end_point = VectorMapSingleton::getInstance()->findByID(Key<Point>(lines.back().fpid));
        points.push_back(end_point);

        //vector
        int vector_index = vector_item_node->getNumChildren();
        int start_line_id = lines[0].lid;
        int end_line_id = lines.back().lid;
        std::string type = "Uncertain";

        osg::ref_ptr<osg::Switch> vectorNode = new osg::Switch;
        vectorNode->setName(vector_item_name + std::to_string(vector_index++));
        vectorNode->setUserValue("start_id", start_line_id);
        vectorNode->setUserValue("end_id", end_line_id);
        vectorNode->setUserValue("item_type", type);
        //line
        {
            osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
            for (const auto& point : points) {
                //reverse x y
                vertices->push_back(osg::Vec3d(point.ly, point.bx, point.h));
            }

            osg::ref_ptr<osg::Vec3Array> colors = new osg::Vec3Array;
            colors->push_back(osg::Vec3(1.0, 1.0, 1.0));

            osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
            geom->setVertexArray(vertices.get());
            geom->setColorArray(colors.get());
            geom->setColorBinding(osg::Geometry::BIND_OVERALL);
            geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP, 0, vertices->size()));

            osg::ref_ptr<osg::Geode> geode = new osg::Geode;
            geode->setName("Line");
            geode->addDrawable(geom);

            vectorNode->addChild(geode);
        }

        //point
        {
            for (const auto& point : points) {
                int local_point_index = point.pid;
                //reverse x y
                osg::Vec3d local_point(point.ly, point.bx, point.h);

                osg::ref_ptr<osg::Geode> point_geode = new osg::Geode;
                point_geode->setName("point");
                point_geode->setUserValue("id", local_point_index);
                point_geode->setUserValue("pos", local_point);

                osg::ref_ptr<osg::ShapeDrawable> point_sphere = new osg::ShapeDrawable(new osg::Sphere(local_point, 0.15f));
                point_geode->addDrawable(point_sphere);
                vectorNode->addChild(point_geode);
            }
        }

        //text
        bool draw_text = false;
        if (draw_text)
        {
            osg::ref_ptr<osg::Switch> point_text_node = dynamic_cast<osg::Switch*>(NodeTreeSearch::findNodeWithName(root_node_, point_text_node_name));
            for (const auto& point: points) {
                //reverse x y
                osg::Vec3d pos(point.ly, point.bx, point.h);
                std::string name = std::to_string(point.pid);
                osg::Vec4f color(0.0, 1.0, 0.0, 0.5);

                point_text_node->addChild(drawTextGeode(pos, name, color));
            }

//        osg::ref_ptr<osg::Switch> area_text_node = dynamic_cast<osg::Switch*>(NodeTreeSearch::findNodeWithName(root_node_, area_text_node_name));
//        for (const auto& area: areas) {
//            const auto& start_line = VectorMapSingleton::getInstance()->findByID(Key<Line>(area.slid));
//            const auto& first_point = VectorMapSingleton::getInstance()->findByID(Key<Point>(start_line.bpid));
//
//            //reverse x y
//            osg::Vec3d pos(first_point.ly, first_point.bx, first_point.h);
//            std::string name = std::to_string(area.aid);
//            osg::Vec4f color(0.0, 1.0, 1.0, 0.5);
//
//            area_text_node->addChild(drawTextGeode(pos, name, color));
//        }

            osg::ref_ptr<osg::Switch> line_text_node = dynamic_cast<osg::Switch*>(NodeTreeSearch::findNodeWithName(root_node_, line_text_node_name));
            for (const auto& line : lines) {

                const auto& backward_point = VectorMapSingleton::getInstance()->findByID(Key<Point>(line.bpid));
                const auto& forward_point = VectorMapSingleton::getInstance()->findByID(Key<Point>(line.fpid));
                //reverse x y
                osg::Vec3d pos1(backward_point.ly, backward_point.bx, backward_point.h);
                osg::Vec3d pos2(forward_point.ly, forward_point.bx, forward_point.h);

                osg::Vec3d pos = ( pos1 + pos2 ) / 2;
                std::string name = std::to_string(line.lid);
                osg::Vec4f color(1.0, 0.0, 0.0, 0.5);

                line_text_node->addChild(drawTextGeode(pos, name, color));
            }
        }

        setNodeValue(object, vectorNode);
        vector_item_node->addChild(vectorNode);
    }
    template <class T>
    void drawTraceNode(const std::vector<Lane>& lanes, const T& object)  {
        if (lanes.empty()) return;

        osg::ref_ptr<osg::Switch> trace_item_node  =
                dynamic_cast<osg::Switch*>(NodeTreeSearch::findNodeWithName(root_node_, trace_item_node_name));

        //nodes
        std::vector<Node> nodes;
        for (const Lane& lane : lanes) {
            Node node = VectorMapSingleton::getInstance()->findByID(Key<Node>(lane.bnid));
            nodes.push_back(node);
        }
        Node end_node = VectorMapSingleton::getInstance()->findByID(Key<Node>(lanes.back().fnid));
        nodes.push_back(end_node);

        //points
        std::vector<Point> points;
        for (const Node& node : nodes) {
            Point point = VectorMapSingleton::getInstance()->findByID(Key<Point>(node.pid));
            points.push_back(point);
        }

        //trace
        int trace_index = trace_item_node->getNumChildren();
        int start_lane_id = lanes[0].lnid;
        int end_lane_id = lanes.back().lnid;
        std::string type = "Lane";

        osg::ref_ptr<osg::Switch> traceNode = new osg::Switch;
        traceNode->setName(trace_item_name + std::to_string(trace_index++));
        traceNode->setUserValue("start_id", start_lane_id);
        traceNode->setUserValue("end_id", end_lane_id);
        traceNode->setUserValue("item_type", type);
        {
            osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
            for (const auto& point : points) {
                //reverse x y
                vertices->push_back(osg::Vec3d(point.ly, point.bx, point.h));
            }

            osg::ref_ptr<osg::Vec3Array> colors = new osg::Vec3Array;
            colors->push_back(osg::Vec3(1.0, 1.0, 0.0));

            osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
            geom->setVertexArray(vertices.get());
            geom->setColorArray(colors.get());
            geom->setColorBinding(osg::Geometry::BIND_OVERALL);
            geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP, 0, vertices->size()));

            osg::ref_ptr<osg::Geode> geode = new osg::Geode;
            geode->setName("Lane");
            geode->addDrawable(geom);

            traceNode->addChild(geode);
        }

        //node
        for (int i = 0; i < nodes.size(); ++i) {
            int local_node_index = nodes[i].nid;
            const Point& point = points[i];
            //reverse x y
            osg::Vec3d local_node(point.ly, point.bx, point.h);

            osg::ref_ptr<osg::Geode> node_geode = new osg::Geode;
            node_geode->setName("node");
            node_geode->setUserValue("id", local_node_index);
            node_geode->setUserValue("pos", local_node);

            osg::ref_ptr<osg::ShapeDrawable> point_sphere = new osg::ShapeDrawable(new osg::Sphere(local_node, 0.1f));
            point_sphere->setColor(osg::Vec4f(1.0, 1.0, 0.0, 1.0));
            node_geode->addDrawable(point_sphere);
            traceNode->addChild(node_geode);
        }

        //text
        bool draw_text = false;
        if(draw_text)
        {
            osg::ref_ptr<osg::Switch> node_text_node = dynamic_cast<osg::Switch*>(NodeTreeSearch::findNodeWithName(root_node_, node_text_node_name));
            for (const auto& node: nodes) {
                const auto& point = VectorMapSingleton::getInstance()->findByID(Key<Point>(node.pid));

                //reverse x y
                osg::Vec3d pos(point.ly, point.bx, point.h);
                std::string name = std::to_string(node.nid);
                osg::Vec4f color(0.0, 1.0, 0.0, 0.5);

                //node_text_node->addChild(drawTextGeode(pos, name, color, 0.2));
            }

            osg::ref_ptr<osg::Switch> lane_text_node = dynamic_cast<osg::Switch*>(NodeTreeSearch::findNodeWithName(root_node_, lane_text_node_name));
            for (const auto& lane : { lanes.front(), lanes.back()} ) {
                const auto& backward_node = VectorMapSingleton::getInstance()->findByID(Key<Node>(lane.bnid));
                const auto& forward_node = VectorMapSingleton::getInstance()->findByID(Key<Node>(lane.fnid));

                const auto& backward_point = VectorMapSingleton::getInstance()->findByID(Key<Point>(backward_node.pid));
                const auto& forward_point = VectorMapSingleton::getInstance()->findByID(Key<Point>(forward_node.pid));

                osg::Vec3d pos1(backward_point.ly, backward_point.bx, backward_point.h);
                osg::Vec3d pos2(forward_point.ly, forward_point.bx, forward_point.h);

                osg::Vec3d pos = ( pos1 + pos2 ) / 2;
                int id = static_cast<int>(lane.lnid);
                std::string name = std::to_string(id);
                osg::Vec4f color(1.0, 0.0, 0.0, 0.5);

                lane_text_node->addChild(drawTextGeode(pos, name, color, 0.2));
            }
        }

        setNodeValue(object, traceNode);
        trace_item_node->addChild(traceNode);
    }

    template <class T>
    void setNodeValue(const T& object, osg::Node* node) {
       setValue(object, node);
    }

    static QStringList getNodeValue(osg::Node* node);

private:
    osg::ref_ptr<osg::Geode> drawTextGeode(const osg::Vec3d& pos, const std::string& content,
            const osg::Vec4f& color, float size = 0.5);

    void setValue(const m_map::RoadEdge& object, osg::Node* node);
    void setValue(const m_map::StopLine& object, osg::Node* node);
    void setValue(const m_map::CrossWalk& object, osg::Node* node);
    void setValue(const m_map::Lane& object, osg::Node* node);
    void setValue(const m_map::Line& object, osg::Node* node);

    osg::ref_ptr<osg::Switch> root_node_;
};


#endif //POINTCLOUDAPPLICATION_VMAPDRAWABLE_H
