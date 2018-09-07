//
// Created by WuKun on 9/4/18.
// Contact me:wk707060335@gmail.com
//

#ifndef POINTCLOUDAPPLICATION_SELECTEDITOR_H
#define POINTCLOUDAPPLICATION_SELECTEDITOR_H

#include <iostream>
#include <string>

#include <QtCore/QObject>

#include <osg/Vec3d>
#include <osg/Vec4f>
#include <osg/Geode>
#include <osgViewer/View>
#include <osgGA/GUIEventHandler>
#include <osgGA/GUIEventAdapter>
#include <osgGA/GUIActionAdapter>


class SelectEditor : public QObject, public osgGA::GUIEventHandler {
    Q_OBJECT
public:
    explicit SelectEditor(osg::Switch* root);
    ~SelectEditor() final = default;

    bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa) final;
    void pick(const osgGA::GUIEventAdapter& ea, osgViewer::View* view);

private:
    template <class T, class U>
    std::vector<T> generate(size_t start_id, size_t end_id, size_t index, const std::vector<U>& lanes) const;
    template <class T, class U>
    size_t calculateLinkID(const T& obj, const std::vector<U>& lanes) const;
    template <class T>
    size_t getObjectId(const T& obj) const;

    double distanceBewteen2DLineSegment(const osg::Vec3d& p1, const osg::Vec3d& p2,
            const osg::Vec3d& q1, const osg::Vec3d& q2) const;
    void cleanUp();

    osg::ref_ptr<osg::Switch> root_node_;
    osg::ref_ptr<osg::Switch> vmap_node_;
    osg::ref_ptr<osg::Switch> temp_node_;

    osg::ref_ptr<osg::Node> selected_node_;
    float _mx,_my;

Q_SIGNALS:
    void selectItem(QStringList itemInfo);

public Q_SLOTS:
    void receiveItemInfo(QStringList itemInfo);
};


#endif //POINTCLOUDAPPLICATION_SELECTEDITOR_H
