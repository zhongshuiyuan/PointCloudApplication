//
// Created by WuKun on 8/24/18.
// Contact me:wk707060335@gmail.com
//

#include <iostream>

#include <QtWidgets/QGridLayout>

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
#include "common.h"
#include "../Common/VectorMapSingleton.h"
#include "../Common/tracer.h"

OSGWidget::OSGWidget(QWidget* parent) :
    QWidget(parent),
    main_view_(nullptr),
    root_node_(nullptr),
    line_editor_(nullptr),
    update_timer_(nullptr) {

}

void OSGWidget::init() {
    TRACER;

    initSceneGraph();
    initCamera();
    initEditor();

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
    root_node_->setName(root_node_name);

    osg::ref_ptr<osg::Switch> vmap_node = new osg::Switch;
    vmap_node->setName(vmap_node_name);
    root_node_->addChild(vmap_node);
    {
        osg::ref_ptr<osg::Switch> point_cloud_node = new osg::Switch;
        point_cloud_node->setName(point_cloud_node_name);
        vmap_node->addChild(point_cloud_node);

        osg::ref_ptr<osg::Switch> vector_item_node = new osg::Switch;
        vector_item_node->setName(vector_item_node_name);
        vmap_node->addChild(vector_item_node);
        {
            osg::ref_ptr<osg::Switch> point_node = new osg::Switch;
            point_node->setName(point_node_name);
            vector_item_node->addChild(point_node);

            osg::ref_ptr<osg::Switch> line_node = new osg::Switch;
            line_node->setName(line_node_name);
            vector_item_node->addChild(line_node);
        }


        osg::ref_ptr<osg::Switch> trace_item_node = new osg::Switch;
        trace_item_node->setName(trace_item_node_name);
        vmap_node->addChild(trace_item_node);
        {
            osg::ref_ptr<osg::Switch> lane_node = new osg::Switch;
            lane_node->setName(lane_node_name);
            trace_item_node->addChild(lane_node);
        }
    }

    osg::ref_ptr<osg::Switch> text_node = new osg::Switch;
    text_node->setName(text_node_name);
    root_node_->addChild(text_node);

    osg::ref_ptr<osg::Switch> temp_node = new osg::Switch;
    temp_node->setName(temp_node_name);
    root_node_->addChild(temp_node);

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

        vmap_node->setStateSet(pcss.get());
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

    main_view_->addEventHandler(new osgViewer::StatsHandler);
    main_view_->addEventHandler(new osgGA::StateSetManipulator(camera->getStateSet()));

    osg::ref_ptr<NodeTreeHandler> nodeTreeHandler = new NodeTreeHandler(root_node_.get());
    main_view_->addEventHandler(nodeTreeHandler.get());

//    osg::ref_ptr<PickHandler> pickHandler = new PickHandler;
//    main_view_->addEventHandler(pickHandler);

    main_view_->setSceneData(root_node_.get());
    main_view_->setCameraManipulator(new osgGA::TrackballManipulator);

    QWidget *widget = graphic_window->getGLWidget();
    auto grid = new QGridLayout;
    grid->addWidget(widget);
    this->setLayout(grid);
}

void OSGWidget::initEditor() {
    line_editor_ = new LineEditor(root_node_);

}

void OSGWidget::initTerrainManipulator(){
    this->initManipulator();
}

void OSGWidget::initManipulator() {
    osg::ref_ptr<osgGA::TerrainManipulator> terrain_manipulator = new osgGA::TerrainManipulator;
    terrain_manipulator->setHomePosition(root_node_->getBound().center() + osg::Vec3(0.0, 0.0, 200.0), //5000
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

void OSGWidget::readPCDataFromFile(const QFileInfo& file_info){
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCDReader reader;
    reader.read(file_info.filePath().toStdString(), *point_cloud);

    osg::ref_ptr<osg::Geode> geode = addMapPointCloud(point_cloud, osg::Vec3(0.4, 0.4, 0.4));
    geode->setName(file_info.fileName().toStdString());

    auto vmap_node = dynamic_cast<osg::Switch*>(root_node_->getChild(0));
    auto point_cloud_node = dynamic_cast<osg::Switch*>(vmap_node->getChild(0));

    point_cloud_node->addChild(geode.get());
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

void OSGWidget::activeLineEditor(bool is_active) {
    if (is_active) {
        main_view_->addEventHandler(line_editor_);
    } else {
        main_view_->removeEventHandler(line_editor_);
    }
}