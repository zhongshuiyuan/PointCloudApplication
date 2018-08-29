//
// Created by WuKun on 8/29/18.
// Contact me:wk707060335@gmail.com
//

#ifndef POINTCLOUDAPPLICATION_LINEEDITOR_H
#define POINTCLOUDAPPLICATION_LINEEDITOR_H

#include <osgViewer/View>
#include <osgGA/GUIEventHandler>
#include <osgGA/GUIEventAdapter>
#include <osgGA/GUIActionAdapter>


class LineEditor : public osgGA::GUIEventHandler
{

public:
    LineEditor();
    ~LineEditor() final = default;

    bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa) final;
    void pick(const osgGA::GUIEventAdapter& ea, osgViewer::View* view);

protected:
    float _mx,_my;
};


#endif //POINTCLOUDAPPLICATION_LINEEDITOR_H
