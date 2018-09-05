//
// Created by WuKun on 9/4/18.
// Contact me:wk707060335@gmail.com
//

#include <vector>
#include <iomanip>
#include <algorithm> // for copy
#include <iterator> // for ostream_iterator

#include <osg/ValueObject>
#include <osg/ShapeDrawable>
#include <osgFX/Outline>

#include "SelectEditor.h"
#include "NodeNames.h"
#include "NodeTreeSearch.h"
#include "../Common/common.h"
#include "../Common/tracer.h"
#include "../Common/DataStructure.h"
#include "../Common/VectorMapSingleton.h"

using m_map::Point;
using m_map::Line;
using m_map::Area;
using m_map::Node;
using m_map::Lane;
using m_map::dtLane;

SelectEditor::SelectEditor(osg::Switch *root) :
    root_node_(root),
    selected_node_(nullptr){
    vmap_node_ = dynamic_cast<osg::Switch*>(NodeTreeSearch::findNodeWithName(root_node_, vmap_node_name));
    temp_node_ = dynamic_cast<osg::Switch*>(NodeTreeSearch::findNodeWithName(root_node_, temp_node_name));
}

SelectEditor::~SelectEditor() {

}


bool SelectEditor::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa){
    auto view = dynamic_cast<osgViewer::View*>(&aa);
    if (!view) return false;

    switch (ea.getEventType()){
        case(osgGA::GUIEventAdapter::MOVE):
        {
            _mx = ea.getX();
            _my = ea.getY();

            return true;
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



void SelectEditor::pick(const osgGA::GUIEventAdapter& ea, osgViewer::View* view) {

    if (ea.getButton() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON) {
        double w = 1.5f;
        double h = 1.5f;

        osg::ref_ptr<osgUtil::PolytopeIntersector> picker = new osgUtil::PolytopeIntersector(
                osgUtil::Intersector::WINDOW, _mx - w, _my - h, _mx + w, _my + h);
        osgUtil::IntersectionVisitor iv(picker);

        //only intersect with vmap_node;
        root_node_->setSingleChildOn(root_node_->getChildIndex(vmap_node_));
        view->getCamera()->accept(iv);
        root_node_->setAllChildrenOn();

        //intersection check
        cleanUp();
        bool has_intersected = false;
        if (picker->containsIntersections()) {
            auto node_path = picker->getFirstIntersection().nodePath;

            std::string prefix("item_");
            for (const auto& node : node_path) {
                std::string nodeName = node->getName();

                if (!nodeName.compare(0, prefix.size(), prefix)) {
                    std::cout << node->libraryName() << " " << node->className() << " " << node->getName().c_str() << std::endl;

                    //record target node
                    selected_node_ = node;

                    //outline effect
                    {
                        osg::ref_ptr<osgFX::Outline> outline = new osgFX::Outline;
                        outline->setWidth(6);
                        outline->addChild(node);
                        temp_node_->addChild(outline);
                    }

                    //fields
                    std::string item_type = "Uncertain";
                    node->getUserValue("item_type", item_type);

                    QStringList itemInfo;
                    itemInfo.append(QString::fromStdString(item_type));

                    if (item_type == "CrossWalk") {
                        int type = 0;
                        node->getUserValue("type", type);
                        itemInfo.append(QString::number(type));

                        int bdid = 0;
                        node->getUserValue("bdid", bdid);
                        itemInfo.append(QString::number(bdid));
                    } else if (item_type == "StopLine") {
                        int tlid = 0;
                        node->getUserValue("tlid", tlid);
                        itemInfo.append(QString::number(tlid));

                        int signid = 0;
                        node->getUserValue("signid", signid);
                        itemInfo.append(QString::number(signid));
                    } else if (item_type == "Lane") {
                        int lcnt = 0;
                        node->getUserValue("lcnt", lcnt);
                        itemInfo.append(QString::number(lcnt));

                        int lno = 0;
                        node->getUserValue("lno", lno);
                        itemInfo.append(QString::number(lno));
                    }

                    emit selectItem(itemInfo);
                    has_intersected = true;
                }
            }
        }

        if (!has_intersected) {
            cleanUp();
        }
    }
}

void SelectEditor::cleanUp() {
    temp_node_->removeChildren(0, temp_node_->getNumChildren());
    selected_node_ = nullptr;
}

void SelectEditor::getItemInfo(QStringList itemInfo) {
    //set value
    std::string item_type = itemInfo[0].toStdString();
    int field1 = itemInfo[1].toInt();
    int field2 = itemInfo[2].toInt();
    int field3 = itemInfo[3].toInt();

    std::cout << "getItemInfo: " << item_type << " " << field1 << " " << field2 << " " << field3 << std::endl;

    if (item_type == "CrossWalk") {
        selected_node_->setUserValue("item_type", item_type);
        selected_node_->setUserValue("type", field1);
        selected_node_->setUserValue("bdid", field2);
    } else if (item_type == "StopLine") {
        selected_node_->setUserValue("item_type", item_type);
        selected_node_->setUserValue("tlid", field1);
        selected_node_->setUserValue("signid", field2);
    } else if (item_type == "Lane") {
        selected_node_->setUserValue("item_type", item_type);
        selected_node_->setUserValue("lcnt", field1);
        selected_node_->setUserValue("lno", field2);
    }

    //update
    int start_id, end_id;
    start_id = end_id = 0;
    selected_node_->getUserValue("start_id", start_id);
    selected_node_->getUserValue("end_id", end_id);
    if (item_type == "Uncertain") return;

    std::cout << "start id: " << start_id << " end id: " << end_id << std::endl;

    if (item_type == "Lane") {
        std::vector<Lane> update_lanes;
        size_t lnid = start_id;
        while (lnid <= end_id && lnid != 0) {
            Lane lane = VectorMapSingleton::getInstance()->findByID(Key<Lane>(lnid));
            lane.lcnt = field1;
            lane.lno = field2;
            update_lanes.push_back(lane);

            lnid = lane.flid;
        }

        VectorMapSingleton::getInstance()->update(update_lanes);
        return;
    }

    //calculate linkid for vector items
    std::vector<Lane> lanes = VectorMapSingleton::getInstance()->findByFilter([](const Lane& lane) { return true; });

    if (item_type == "StopLine") {
        size_t index = VectorMapSingleton::getInstance()->getMaxStopLineIndex();

        std::vector<StopLine> stop_lines = generate<StopLine, Lane>(start_id, end_id, index, lanes);
        for(auto& obj : stop_lines) {
            obj.tlid = field1;
            obj.signid = field2;
        }
        VectorMapSingleton::getInstance()->update(stop_lines);
    }

    if (item_type == "RoadEdge") {
        size_t index = VectorMapSingleton::getInstance()->getMaxRoadEdgeIndex();

        std::vector<RoadEdge> road_edges = generate<RoadEdge, Lane>(start_id, end_id, index, lanes);
        VectorMapSingleton::getInstance()->update(road_edges);
    }
}


template <class T, class U>
std::vector<T> SelectEditor::generate(size_t start_id, size_t end_id, size_t index, const std::vector<U>& lanes) const {
    std::vector<T> objects;
    size_t lid = start_id;
    while (lid <= end_id) {
        T object(index++, lid);
        size_t linkid = calculateLinkID<T, U>(object, lanes);
        object.linkid = linkid;

        objects.push_back(object);
        std::cout << "object:" << object << std::endl;
        //TODO:calculate next id
        lid++;
    }

    return objects;
}


template <class T>
size_t SelectEditor::nextID(const T& obj) const {
    return 1;
    Line line = VectorMapSingleton::getInstance()->findByID(Key<Line>(obj.lid));
}

template <class T, class U>
size_t SelectEditor::calculateLinkID(const T& obj, const std::vector<U>& lanes) const {
    size_t lid = obj.lid;

    Line line = VectorMapSingleton::getInstance()->findByID(Key<Line>(lid));
    Point point1 = VectorMapSingleton::getInstance()->findByID(Key<Point>(line.bpid));
    Point point2 = VectorMapSingleton::getInstance()->findByID(Key<Point>(line.fpid));
    osg::Vec3d p1(point1.bx, point1.ly, 0);
    osg::Vec3d p2(point2.bx, point2.ly, 0);
    double min_distance = INT_MAX;
    size_t linkId = 0;

    for (const auto& lane : lanes) {
        Node node1 = VectorMapSingleton::getInstance()->findByID(Key<Node>(lane.bnid));
        Node node2 = VectorMapSingleton::getInstance()->findByID(Key<Node>(lane.fnid));
        Point _point1 = VectorMapSingleton::getInstance()->findByID(Key<Point>(node1.pid));
        Point _point2 = VectorMapSingleton::getInstance()->findByID(Key<Point>(node2.pid));
        osg::Vec3d q1(_point1.bx, _point1.ly, 0);
        osg::Vec3d q2(_point2.bx, _point2.ly, 0);

        double distance = distanceBewteen2DLineSegment(p1, p2, q1, q2);
        if (distance < min_distance) {
            min_distance = distance;
            linkId = lane.lnid;
        }
    }

    return linkId;
}


double SelectEditor::distanceBewteen2DLineSegment(const osg::Vec3d& p1, const osg::Vec3d& p2,
        const osg::Vec3d& q1, const osg::Vec3d& q2) const {
    osg::Vec3d d1 = p2 - p1;
    osg::Vec3d d2 = q2 - q1;

    double a, b, c ,d, e;
    a = d1 * d1;
    b = d1 * d2;
    c = d2 * d2;
    d = d1 * (p1 - q1);
    e = d2 * (p1 - q1);

    double t_num, t_den, s_num, s_den;
    t_num = t_den = s_num = s_den = 0;

    double delta = a * c - b * b;
    t_den = s_den = delta;

    //parallel
    if (delta == 0) {
        t_num = 0; s_num = -e; s_den = b;
    } else {
        s_num = b * e - c * d; t_num = a * e - b * d;
    }

    if (s_num < 0) {
        s_num = 0; t_num = d; t_den = b;
    } else if (s_num > s_den) {
        s_num = s_den; t_num = a + d; t_den = b;
    }

    if (t_num < 0) {
        t_num = 0; s_num = -d; s_den = a;
        if (s_num < 0) {
            s_num = 0;
        } else if (s_num > s_den) {
            s_num = s_den = 1;
        }
    } else if (t_num > t_den) {
        t_num = t_den; s_num = c - e; s_den = b;
        if (s_num < 0) {
            s_num = 0;
        }

        if (s_num > s_den) {
            s_num = s_den = 1;
        }
    }
    double t, s;
    t = s = 0;
    if (s_den != 0) s = s_num / s_den;
    if (t_den != 0) t = t_num / t_den;


    osg::Vec3d p = p1 + d1 * s;
    osg::Vec3d q = q1 + d2 * t;
    osg::Vec3d dis = p - q;
    double distance = dis.length();

    return distance;
}