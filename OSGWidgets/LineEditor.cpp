//
// Created by WuKun on 8/29/18.
// Contact me:wk707060335@gmail.com
//

#include <vector>
#include <iomanip>
#include <algorithm> // for copy
#include <iterator> // for ostream_iterator

#include <osg/ValueObject>
#include <osg/ShapeDrawable>

#include "LineEditor.h"
#include "common.h"
#include "NodeTreeSearch.h"
#include "../Common/tracer.h"
#include "../Common/VectorMapSingleton.h"
#include "DataStructure.h"
using m_map::Point;


LineEditor::LineEditor(osg::Switch* root_node) :
    root_node_(root_node),
    temp_node_(nullptr),
    line_node_(nullptr),
    temp_line_geode_(nullptr),
    _mx(0),
    _my(0) {

    //find related nodes
    line_node_ = dynamic_cast<osg::Switch*>(findNodeWithName(root_node, line_node_name));
    temp_node_ = dynamic_cast<osg::Switch*>(findNodeWithName(root_node, temp_node_name));

    cur_point_index = cur_min_point_index_ = VectorMapSingleton::getInstance()->getMaxPointIndex() + 1;
}

LineEditor::~LineEditor() {
    cleanUp();
}

bool LineEditor::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa){
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

void LineEditor::pick(const osgGA::GUIEventAdapter& ea, osgViewer::View* view) {

    if (ea.getButton() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON) {
        double w = 3.0f;
        double h = 3.0f;

        osg::ref_ptr<osgUtil::PolytopeIntersector> picker = new osgUtil::PolytopeIntersector(
                osgUtil::Intersector::WINDOW, _mx - w, _my - h, _mx + w, _my + h);
        osgUtil::IntersectionVisitor iv(picker);
        view->getCamera()->accept(iv);

        if (picker->containsIntersections()) {

            osg::NodePath all_node_path = picker->getFirstIntersection().nodePath;
//            std::cout << "node path: " << all_node_path.size() << std::endl;
//            for(auto node_path : all_node_path)
//            {
//                std::cout << node_path->libraryName() << " " << node_path->className() << " " << node_path->getName() << std::endl;
//            }

            auto iter = picker->getIntersections().begin();
            if (iter != picker->getIntersections().end())
            {
                osg::Vec3d local_point(iter->localIntersectionPoint);
                int local_point_index = cur_point_index++;

                auto child_node = all_node_path.back();
                if (child_node->getName() == "point") {  //connection
                    child_node->getUserValue("pos", local_point);
                    child_node->getUserValue("id", local_point_index);
                }
                else
                {
                    osg::ref_ptr<osg::Geode> point_geode = new osg::Geode;
                    point_geode->setName("point");
                    point_geode->setUserValue("pos", local_point);
                    point_geode->setUserValue("id", local_point_index);

                    osg::ref_ptr<osg::ShapeDrawable> point_sphere = new osg::ShapeDrawable(new osg::Sphere(local_point, 0.1f));
                    point_geode->addDrawable(point_sphere);
                    temp_node_->addChild(point_geode);
                }
//                std::cout << "point: " << local_point_index << " " << local_point << std::endl;
                selected_points.emplace_back(std::make_pair(local_point_index, local_point));
            }
        }

        //draw line
        if (selected_points.size() >= 2)
        {
            osg::Vec3d penultimate_point = std::get<1>(*(selected_points.end() - 2));
            osg::Vec3d last_point = std::get<1>(selected_points.back());

            osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
            vertices->push_back(penultimate_point);
            vertices->push_back(last_point);

            osg::ref_ptr<osg::Vec3Array> colors = new osg::Vec3Array;
            colors->push_back(osg::Vec3(1.0, 1.0, 1.0));

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

    if (ea.getButton() == osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON && selected_points.size() >= 2)
    {
        //recalculate id
        std::vector<Point> points;
        {
            for (const auto& pair : selected_points) {
                size_t index = std::get<0>(pair);

                if (index >= cur_min_point_index_) index = cur_min_point_index_++;

                osg::Vec3d pos = std::get<1>(pair);

                Point p(index, pos.x(), pos.y(), pos.z());
                points.emplace_back(p);
            }

            std::cout << "result: " << std::endl;
            for (auto point : points)
                std::cout << point << std::endl;
//            std::copy(points.begin(), points.end(), std::ostream_iterator<Point>(std::cout, "\n"));

            VectorMapSingleton::getInstance()->update(points);
        }

        //redraw line & point
        {
            osg::ref_ptr<osg::Switch> oneDrawNode = new osg::Switch;
            oneDrawNode->setName("test");
            line_node_->addChild(oneDrawNode);
            //line
            {
                osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
                for (const auto& point : points) {
                    vertices->push_back(osg::Vec3d(point.bx, point.ly, point.h));
                }

                osg::ref_ptr<osg::Vec3Array> colors = new osg::Vec3Array;
                colors->push_back(osg::Vec3(1.0, 1.0, 1.0));

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
                for (const auto& point : points) {
                    int local_point_index = point.pid;
                    osg::Vec3d local_point(point.bx, point.ly, point.h);

                    osg::ref_ptr<osg::Geode> point_geode = new osg::Geode;
                    point_geode->setName("point");
                    point_geode->setUserValue("id", local_point_index);
                    point_geode->setUserValue("pos", local_point);

                    osg::ref_ptr<osg::ShapeDrawable> point_sphere = new osg::ShapeDrawable(new osg::Sphere(local_point, 0.1f));
                    point_geode->addDrawable(point_sphere);
                    oneDrawNode->addChild(point_geode);
                }
            }
        }


        cleanUp(true);
    } //right button
}

void LineEditor::cleanUp(bool all) {
    if (temp_line_geode_)
    {
        temp_line_geode_->removeDrawables(0, temp_line_geode_->getNumDrawables());
        temp_line_geode_ = nullptr;
    }
    if (all) temp_node_->removeChildren(0, temp_node_->getNumChildren());

    selected_points.clear();
    selected_points.shrink_to_fit();

    cur_point_index = cur_min_point_index_ = VectorMapSingleton::getInstance()->getMaxPointIndex() + 1;
}

std::ostream& operator<<(std::ostream& os, const osg::Vec3d& point){
    os.flags(std::ios::right);
    os << std::setw(8) << point.x() << ","
       << std::setw(8) << point.y() << ","
       << std::setw(8) << point.z();
    return os;
}
