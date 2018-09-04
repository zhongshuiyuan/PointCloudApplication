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

#include "common.h"
#include "OSGWidget.h"
#include "NodeTreeInfo.h"
#include "TextController.h"
#include "../Common/VectorMapSingleton.h"
#include "../Common/tracer.h"

OSGWidget::OSGWidget(QWidget* parent) :
    QWidget(parent),
    main_view_(nullptr),
    root_node_(nullptr),
    line_editor_(nullptr),
    trace_editor_(nullptr),
    select_editor_(nullptr),
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

    osg::ref_ptr<osg::Switch> point_cloud_node = new osg::Switch;
    point_cloud_node->setName(point_cloud_node_name);
    root_node_->addChild(point_cloud_node);

    osg::ref_ptr<osg::Switch> vmap_node = new osg::Switch;
    vmap_node->setName(vmap_node_name);
    root_node_->addChild(vmap_node);
    {
        osg::ref_ptr<osg::Switch> vector_item_node = new osg::Switch;
        vector_item_node->setName(vector_item_node_name);
        vmap_node->addChild(vector_item_node);

        osg::ref_ptr<osg::Switch> trace_item_node = new osg::Switch;
        trace_item_node->setName(trace_item_node_name);
        vmap_node->addChild(trace_item_node);
    }

    osg::ref_ptr<osg::Switch> text_node = new osg::Switch;
    text_node->setName(text_node_name);
    root_node_->addChild(text_node);
    {
        osg::ref_ptr<osg::Switch> point_text_node = new osg::Switch;
        point_text_node->setName(point_text_node_name);
        text_node->addChild(point_text_node);

        osg::ref_ptr<osg::Switch> line_text_node = new osg::Switch;
        line_text_node->setName(line_text_node_name);
        text_node->addChild(line_text_node);

        osg::ref_ptr<osg::Switch> area_text_node = new osg::Switch;
        area_text_node->setName(area_text_node_name);
        text_node->addChild(area_text_node);

        osg::ref_ptr<osg::Switch> node_text_node = new osg::Switch;
        node_text_node->setName(node_text_node_name);
        text_node->addChild(node_text_node);

        osg::ref_ptr<osg::Switch> lane_text_node = new osg::Switch;
        lane_text_node->setName(lane_text_node_name);
        text_node->addChild(lane_text_node);

        osg::ref_ptr<osg::Switch> dtlane_text_node = new osg::Switch;
        dtlane_text_node->setName(dtlane_text_node_name);
        text_node->addChild(dtlane_text_node);
    }

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

        point_cloud_node->setStateSet(pcss.get());
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

    //for outline effects
    {
        osg::DisplaySettings::instance()->setMinimumNumStencilBits(1);
        unsigned int clearMask = camera->getClearMask();
        camera->setClearMask(clearMask | GL_STENCIL_BUFFER_BIT);
        camera->setClearStencil(0);
    }

    main_view_->addEventHandler(new osgViewer::StatsHandler);
    main_view_->addEventHandler(new osgGA::StateSetManipulator(camera->getStateSet()));

    osg::ref_ptr<NodeTreeHandler> nodeTreeHandler = new NodeTreeHandler(root_node_.get());
    main_view_->addEventHandler(nodeTreeHandler.get());

    osg::ref_ptr<TextController> textController = new TextController(root_node_.get());
    main_view_->addEventHandler(textController);

    main_view_->setSceneData(root_node_.get());
    main_view_->setCameraManipulator(new osgGA::TrackballManipulator);

    QWidget *widget = graphic_window->getGLWidget();
    auto grid = new QGridLayout;
    grid->addWidget(widget);
    this->setLayout(grid);
}

void OSGWidget::initEditor() {
    line_editor_ = new LineEditor(root_node_);
    trace_editor_ = new TraceEditor(root_node_);
    select_editor_ = new SelectEditor(root_node_);
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

    osg::ref_ptr<osg::Switch> point_cloud_node =
            dynamic_cast<osg::Switch*>(NodeTreeSearch::findNodeWithName(root_node_, point_cloud_node_name));

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

void OSGWidget::activeTraceEditor(bool is_active) {
    if (is_active) {
        main_view_->addEventHandler(trace_editor_);
    } else {
        main_view_->removeEventHandler(trace_editor_);
    }
}

void OSGWidget::activeSelectEditor(bool is_active) {
    if (is_active) {
        main_view_->addEventHandler(select_editor_);
    } else {
        main_view_->removeEventHandler(select_editor_);
    }
}

double distanceBewteen2DLineSegment(const osg::Vec3d& p1, const osg::Vec3d& p2, const osg::Vec3d& q1, const osg::Vec3d& q2) {
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

void OSGWidget::saveVectorMapToDir(const std::string dir_path) const {

    //calculate closest lane foreach line
    std::vector<Line> lines = VectorMapSingleton::getInstance()->findByFilter([](const Line& line) { return true; });
    std::vector<Lane> lanes = VectorMapSingleton::getInstance()->findByFilter([](const Lane& lane) { return true; });

    for(Line& line : lines) {
        Point point1 = VectorMapSingleton::getInstance()->findByID(Key<Point>(line.bpid));
        Point point2 = VectorMapSingleton::getInstance()->findByID(Key<Point>(line.fpid));
        osg::Vec3d p1(point1.bx, point1.ly, 0);
        osg::Vec3d p2(point2.bx, point2.ly, 0);
        double min_distance = INT_MAX;
        size_t linkId = 0;

        for (const Lane& lane : lanes) {
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

        std::cout << line.lid << " closest lane id: " << linkId << std::endl;
    }

    VectorMapSingleton::getInstance()->saveToDir(dir_path);
}