//
// Created by WuKun on 8/24/18.
// Contact me:wk707060335@gmail.com
//

#include <iostream>

#include <QtWidgets/QGridLayout>

#include <OpenThreads/Thread>
#include <OpenThreads/Condition>
#include <OpenThreads/ScopedLock>
#include <osg/Geometry>
#include <osg/Material>
#include <osg/ShapeDrawable>
#include <osgDB/ReadFile>
#include <osgQt/GraphicsWindowQt>
#include <osgViewer/ViewerEventHandlers>

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/console/time.h>


#include "OSGWidget.h"
#include "NodeTreeInfo.h"
#include "../Common/VectorMapSingleton.h"
#include "../Common/tracer.h"

// forward declarations
osgQt::GraphicsWindowQt* createGraphicsWindow(int x, int y, int w, int h, const std::string& name = "", bool windowDecoration = false);
osg::Geode* addMapPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr mapPointCloud, osg::Vec3 color = osg::Vec3(1.0, 1.0, 1.0));
osg::Geode* createQuade();


OSGWidget::OSGWidget(QWidget* parent)
    : QWidget(parent){
}

void OSGWidget::init(){
    TRACER;

    initSceneGraph();
    initCamera();

    update_timer_ = new QTimer;
    connect(update_timer_, SIGNAL(timeout()), this, SLOT(update()));
    update_timer_->start(10);
}

void OSGWidget::paintEvent(QPaintEvent *event) {
    frame();
}

void OSGWidget::initSceneGraph() {
    TRACER;

    root_node_ = new osg::Switch;
    root_node_->setName("root_node");

    //test
    {
        auto node = osgDB::readNodeFile("../../resources/data/cow.osg");
        if(node)
        {
            qDebug() << "node valid!";
            root_node_->addChild(node);
        }
    }

//    osg::ref_ptr<osg::StateSet> ss = root_node_->getOrCreateStateSet();
//    ss->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
}

void OSGWidget::initCamera() {
    TRACER;

    osgViewer::ViewerBase::ThreadingModel threadingModel = osgViewer::ViewerBase::SingleThreaded;
    this->setThreadingModel(threadingModel);
    this->setKeyEventSetsDone(0);

    auto graphic_window = createGraphicsWindow(200, 200, 1000, 800);
    auto traits = graphic_window->getTraits();

    main_view_ = new osgViewer::Viewer;
    auto camera = main_view_->getCamera();
    camera->setGraphicsContext(graphic_window);
    camera->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OVERRIDE | osg::StateAttribute::ON);
    camera->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::ON);
    camera->setProjectionMatrixAsPerspective(30.f, static_cast<double>(traits->width) / static_cast<double>(traits->height), 1.0, 1000.0);
    camera->setRenderOrder(osg::Camera::PRE_RENDER);
//    camera->setClearColor(osg::Vec4(0.84313, 0.84313, 0.89804, 1.0));
    camera->setClearMask(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
    camera->setViewport(new osg::Viewport(0, 0, traits->width, traits->height));
//    qDebug() << "traits:" << traits->width << traits->height;

    main_view_->addEventHandler(new osgViewer::StatsHandler);

    osg::ref_ptr<NodeTreeHandler> nodeTreeHandler = new NodeTreeHandler(root_node_.get());
    main_view_->addEventHandler(nodeTreeHandler.get());

    osg::ref_ptr<PickHandler> pickHandler = new PickHandler;
    main_view_->addEventHandler(pickHandler);

    main_view_->setSceneData(root_node_.get());
    main_view_->setCameraManipulator(new osgGA::TrackballManipulator);
    addView(main_view_.get());

    QWidget *widget = graphic_window->getGLWidget();
    auto grid = new QGridLayout;
    grid->addWidget(widget);
    this->setLayout(grid);
}

void OSGWidget::initTerrainManipulator(){
    this->initManipulator();
}

void OSGWidget::initManipulator() {
    osg::ref_ptr<osgGA::TerrainManipulator> trackballManip = new osgGA::TerrainManipulator;
    trackballManip->setHomePosition(root_node_->getBound().center() + osg::Vec3(0.0, 0.0, 10000.0),
                                    root_node_->getBound().center(), osg::Vec3(0, 1, 0));
    main_view_->setCameraManipulator( trackballManip );
}

osgQt::GraphicsWindowQt* createGraphicsWindow(int x, int y, int w, int h, const std::string& name, bool windowDecoration)
{
    osg::DisplaySettings* ds = osg::DisplaySettings::instance().get();
    osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
    traits->windowName = name;
    traits->windowDecoration = windowDecoration;
    traits->x = x;
    traits->y = y;
    traits->width = w;
    traits->height = h;
    traits->doubleBuffer = true;
    traits->alpha = ds->getMinimumNumAlphaBits();
    traits->stencil = ds->getMinimumNumStencilBits();
    traits->sampleBuffers = ds->getMultiSamples();
    traits->samples = ds->getNumMultiSamples();

    return new osgQt::GraphicsWindowQt(traits.get());
}

std::ostream& operator<<(std::ostream& os, const osg::Vec3d& point){
    os << point.x() << ","
       << point.y() << ","
       << point.z();
    return os;
}

osg::Geode* createQuade(){
    osg::ref_ptr<osg::Geometry> geom=new osg::Geometry();//定义一个几何体
    //首先定义四个点
    osg::ref_ptr<osg::Vec3Array> v=new osg::Vec3Array();//定义一个几何体坐标集合
    v->push_back(osg::Vec3(-1.0,0.0,-1.0));//左下角坐标点
    v->push_back(osg::Vec3(1.0,0.0,-1.0));//右下角坐标点
    v->push_back(osg::Vec3(1.0,0.0,1.0));//右上角坐标点
    v->push_back(osg::Vec3(-1.0,0.0,1.0));//左上角坐标点
    geom->setVertexArray(v.get());//将坐标设置到几何体节点中
    //定义颜色数组
    osg::ref_ptr<osg::Vec4Array> c=new osg::Vec4Array();//定义一个颜色数组颜色
    c->push_back(osg::Vec4(1.0,0.0,0.0,1.0));//数组的四个参数分别为RGBA，其中A表示透明度
    c->push_back(osg::Vec4(0.0,1.0,0.0,1.0));
    c->push_back(osg::Vec4(0.0,0.0,1.0,1.0));
    c->push_back(osg::Vec4(1.0,1.0,1.0,1.0));
    geom->setColorArray(c.get());//与几何体中进行关联
    geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);//设置绑定方式为逐点绑定。
    //定义法线
    osg::ref_ptr<osg::Vec3Array> n=new osg::Vec3Array();//定义了一个法线绑定到该四方体中
    n->push_back(osg::Vec3(0.0,-1.0,0.0));//法线为指向Y轴负半轴
    geom->setNormalArray(n.get());//添加法线到几何体中
    geom->setNormalBinding(osg::Geometry::BIND_OVERALL);//将法线进行绑定
    //设置顶点的关联方式，这里是Quad方式，总共有这么些方式：POINTS,LINES,LINE_STRIP,LINE_LOOP,TRIANGLES,TRIANGLE_STRIP,TRIANGLE_FAN,QUADS,QUAD_STRIP,POLYGON
    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUADS,0,4));

    //定义一个几何组结点，并把几何体结点加入到几何组结点当中
    osg::ref_ptr<osg::Geode> geode=new osg::Geode();
    geode->setName("Rect");
    geode->addDrawable(geom.get());

    return geode.release();
}

bool OSGWidget::read_data_from_file(const QFileInfo& file_info){

//    osg::ref_ptr<osg::TessellationHints> hints = new osg::TessellationHints;
//    hints->setDetailRatio(0.5f);
//
//    osg::Vec4d color(1.0f, 0.0f, 0.0f, 0.5f);
//
//    osg::ref_ptr<osg::Geode> unitSphere = new osg::Geode;
//    unitSphere->setName("sphere");
//    osg::ref_ptr<osg::Sphere> sphere = new osg::Sphere(osg::Vec3(0,0,0), 10.0f);
//    osg::ref_ptr<osg::ShapeDrawable> shapeDrawable = new osg::ShapeDrawable(sphere.get(), hints.get());
//    shapeDrawable->setColor(color);
//    unitSphere->addDrawable(shapeDrawable.get());
//
//    root_node_->addChild(unitSphere.get(), true);

    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCDReader reader;
    reader.read(file_info.filePath().toStdString(), *point_cloud);
    osg::ref_ptr<osg::Geode> geode = addMapPointCloud(point_cloud, osg::Vec3(0.4, 0.4, 0.4));
    geode->setName(file_info.fileName().toStdString());

    root_node_->addChild(geode.get());
}

osg::Geode* addMapPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr mapPointCloud, osg::Vec3 color){

    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
    osg::ref_ptr<osg::Vec3Array> vertexs = new osg::Vec3Array;
    osg::ref_ptr<osg::Vec3Array> colors = new osg::Vec3Array;
    for (int i = 0; i < mapPointCloud->points.size(); i++)
    {
        vertexs->push_back(osg::Vec3(mapPointCloud->points[i].x,
                                     mapPointCloud->points[i].y,
                                     mapPointCloud->points[i].z));
    }
    colors->push_back(color);
    geom->setVertexArray(vertexs);
    geom->setColorArray(colors);
    geom->setColorBinding(osg::Geometry::BIND_OVERALL);
    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS,0,vertexs->size()));
    geode->addDrawable(geom.get());

    return geode.release();
}