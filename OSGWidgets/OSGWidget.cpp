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
#include <osg/Light>
#include <osg/Point>
#include <osg/ShapeDrawable>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgGA/StateSetManipulator>
#include <osgViewer/ViewerEventHandlers>

#include "OSGWidget.h"
#include "NodeTreeInfo.h"
#include "../Common/VectorMapSingleton.h"
#include "../Common/tracer.h"

OSGWidget::OSGWidget(QWidget* parent) :
    QWidget(parent),
    main_view_(nullptr),
    root_node_(nullptr),
    update_timer_(nullptr) {
}

void OSGWidget::init() {
    TRACER;

    initSceneGraph();
    initCamera();

    update_timer_ = new QTimer();
    QObject::connect(update_timer_, SIGNAL(timeout()), this, SLOT(update()));
    update_timer_->start(30);
}

void OSGWidget::paintEvent(QPaintEvent *) {
    frame();
}

void OSGWidget::initSceneGraph() {
    TRACER;

    root_node_ = new osg::Switch;
    root_node_->setName("root_node");

    {
        //离散对象节点光照
        osg::ref_ptr<osg::Light> pclight = new osg::Light;
        pclight->setAmbient(osg::Vec4(.8f, .8f, .8f, .8f));
        osg::ref_ptr<osg::Point> pcps = new osg::Point(1.0f);

        osg::ref_ptr<osg::StateSet> pcss = new osg::StateSet;
        pcss->setMode(GL_LIGHTING, osg::StateAttribute::ON);
        pcss->setAttribute(pclight, osg::StateAttribute::ON);
        pcss->setMode(GL_BLEND, osg::StateAttribute::ON);
        pcss->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
        pcss->setMode(GL_MULTISAMPLE_ARB, osg::StateAttribute::ON);
        pcss->setAttribute(pcps, osg::StateAttribute::ON);

        root_node_->setStateSet(pcss.get());
    }
}

void OSGWidget::initCamera() {
    TRACER;

    osgViewer::ViewerBase::ThreadingModel threadingModel = osgViewer::ViewerBase::SingleThreaded;
    this->setThreadingModel(threadingModel);
    this->setKeyEventSetsDone(0);

    auto graphic_window = createGraphicsWindow(0, 0, 2000, 2000);
    auto traits = graphic_window->getTraits();

    main_view_ = new osgViewer::View;
    this->addView(main_view_.get());

    auto camera = main_view_->getCamera();
    camera->setGraphicsContext(graphic_window);
    camera->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OVERRIDE | osg::StateAttribute::ON);
    camera->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    camera->setClearMask(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
    camera->setViewport(new osg::Viewport(0, 0, traits->width, traits->height));
    camera->setProjectionMatrixAsPerspective(30.f, static_cast<double>(traits->width) / static_cast<double>(traits->height), 1.0, 1000.0);
    camera->setNearFarRatio(0.0000002);
    camera->setComputeNearFarMode(osg::CullSettings::COMPUTE_NEAR_FAR_USING_BOUNDING_VOLUMES);
//    camera->setClearColor(osg::Vec4(0.84313, 0.84313, 0.89804, 1.0));
//    qDebug() << "traits:" << traits->width << traits->height;

    main_view_->addEventHandler(new osgViewer::StatsHandler);
    main_view_->addEventHandler(new osgGA::StateSetManipulator(camera->getStateSet()));

    osg::ref_ptr<NodeTreeHandler> nodeTreeHandler = new NodeTreeHandler(root_node_.get());
    main_view_->addEventHandler(nodeTreeHandler.get());

    osg::ref_ptr<PickHandler> pickHandler = new PickHandler;
    main_view_->addEventHandler(pickHandler);

    main_view_->setSceneData(root_node_.get());
    main_view_->setCameraManipulator(new osgGA::TrackballManipulator);

    QWidget *widget = graphic_window->getGLWidget();
    auto grid = new QGridLayout;
    grid->addWidget(widget);
    this->setLayout(grid);
}

void OSGWidget::initTerrainManipulator(){
    this->initManipulator();
}

void OSGWidget::initManipulator() {
    osg::ref_ptr<osgGA::TerrainManipulator> terrain_manipulator = new osgGA::TerrainManipulator;
    terrain_manipulator->setHomePosition(root_node_->getBound().center() + osg::Vec3(0.0, 0.0, 5000.0),
                                    root_node_->getBound().center(), osg::Vec3(0, 1, 0));
    main_view_->setCameraManipulator(terrain_manipulator.get());
}

osgQt::GraphicsWindowQt* OSGWidget::createGraphicsWindow(int x, int y, int w, int h, const std::string& name, bool windowDecoration)
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

void OSGWidget::readPCDataFromFile(const QFileInfo& file_info){
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCDReader reader;
    reader.read(file_info.filePath().toStdString(), *point_cloud);

    osg::ref_ptr<osg::Geode> geode = addMapPointCloud(point_cloud, osg::Vec3(0.4, 0.4, 0.4));
    geode->setName(file_info.fileName().toStdString());

    root_node_->addChild(geode.get());
}

osg::Geode* OSGWidget::addMapPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& mapPointCloud, osg::Vec3 color){

    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;

    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
//    auto center = mapPointCloud->points.front();
    for(const auto point : mapPointCloud->points)
    {
        vertices->push_back(osg::Vec3(point.x, point.y, point.z));
//        vertices->push_back(osg::Vec3(point.x - center.x, point.y - center.y, point.z - center.z));
    }
    geom->setVertexArray(vertices);

    osg::ref_ptr<osg::Vec3Array> colors = new osg::Vec3Array;
    colors->push_back(color);
    geom->setColorArray(colors);
    geom->setColorBinding(osg::Geometry::BIND_OVERALL);

    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS,0,vertices->size()));
    geode->addDrawable(geom.get());

    return geode.release();
}