//
// Created by WuKun on 9/6/18.
// Contact me:wk707060335@gmail.com
//
#include <osg/ValueObject>
#include <osgText/Text>

#include "VMapDrawable.h"

VMapDrawable::VMapDrawable(osg::Switch *root) :
        root_node_(root) {
}

osg::ref_ptr<osg::Geode> VMapDrawable::drawTextGeode(const osg::Vec3d& pos, const std::string& content,
        const osg::Vec4f& color, float size) {
    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    geode->setName(content);

    osg::ref_ptr<osgText::Text> text = new osgText::Text;
    text->setCharacterSize(size);
    text->setAxisAlignment( osgText::TextBase::XY_PLANE );
    text->setPosition(pos);
    text->setText(content);
    text->setColor(color);

    geode->addDrawable(text);
    return geode.release();
}

void VMapDrawable::setRoadEdgeNodeValue(const RoadEdge &object, osg::Node* node) {
    std::string item_type = "RoadEdge";
    node->setUserValue("item_type", item_type);
}

void VMapDrawable::setStopLineNodeValue(const StopLine &object, osg::Node* node) {
    std::string item_type = "StopLine";
    int tlid = object.tlid;
    int signid = object.signid;

    node->setUserValue("item_type", item_type);
    node->setUserValue("tlid", tlid);
    node->setUserValue("signid", signid);
}

void VMapDrawable::setCrossWalkNodeValue(const CrossWalk &object, osg::Node* node) {
    std::string item_type = "CrossWalk";
    int bdid = object.bdid;
    int type = object.type;

    node->setUserValue("item_type", item_type);
    node->setUserValue("bdid", bdid);
    node->setUserValue("type", type);
}

void VMapDrawable::setLaneNodeValue(const m_map::Lane& object, osg::Node* node) {
    std::string item_type = "Lane";
    int lcnt = object.lcnt;
    int lno = object.lno;

    node->setUserValue("item_type", item_type);
    node->setUserValue("lcnt", lcnt);
    node->setUserValue("lno", lno);
}

void VMapDrawable::setNullNodeValue(osg::Node* node){
    std::string item_type = "Uncertain";
    node->setUserValue("item_type", item_type);
}

QStringList VMapDrawable::getNodeValue(osg::Node *node) {
    if (!node) return QStringList();

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

    return itemInfo;
}


