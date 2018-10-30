//
// Created by WuKun on 8/24/18.
// Contact me:wk707060335@gmail.com
//

#include <iostream>
#include <fstream>
#include <bitset>
#include <iomanip>

#include <QtWidgets/QGridLayout>
#include <QtCore/QDir>
#include <QtCore/QString>
#include <QtCore/QFileInfoList>
#include <QtCore/QTextStream>

#include <osg/Light>
#include <osg/Point>
#include <osg/Material>
#include <osg/Geometry>
#include <osg/ValueObject>
#include <osg/ShapeDrawable>
#include <osg/PositionAttitudeTransform>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgGA/StateSetManipulator>
#include <osgViewer/ViewerEventHandlers>

#include <osgEarth/MapNode>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/ObjectLocator>
#include <osgEarthUtil/Sky>

#include "NodeNames.h"
#include "OSGWidget.h"
#include "LineEditor.h"
#include "TraceEditor.h"
#include "SelectEditor.h"
#include "NodeTreeInfo.h"
#include "VMapDrawable.h"
#include "TextController.h"
#include "PositionTransformer.h"
#include "../Common/tracer.h"
#include "../Common/VectorMapSingleton.h"


OSGWidget::OSGWidget(QWidget* parent) :
    QWidget(parent),
    main_view_(nullptr),
    root_node_(nullptr),
    line_editor_(nullptr),
    trace_editor_(nullptr),
    select_editor_(nullptr),
    update_timer_(nullptr) {

}

OSGWidget::~OSGWidget()  = default;

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

    //earth
    std::string earth_file = "../../resources/gisms.earth";
    osg::ref_ptr<osg::Node> node = osgDB::readNodeFile(earth_file);
    map_node_ = osgEarth::MapNode::findMapNode(node.get());
    map_node_->setName(earth_node_name);
    PositionTransformer::getInstance()->setMapNode(map_node_);

    //sky----the only entrance of the whole scene node tree!
    osg::ref_ptr<osgEarth::Util::SkyNode> sky_node = osgEarth::Util::SkyNode::create(map_node_.get());
    sky_node->setName("Sky");
    sky_node->setDateTime(osgEarth::DateTime(2018, 10, 11, 6));
    osg::ref_ptr<osgEarth::Util::Ephemeris> ephemeris = new osgEarth::Util::Ephemeris;
    sky_node->setEphemeris(ephemeris.get());
    sky_node->attach(main_view_, 0);
    sky_node->setLighting(true);
    sky_node->getSunLight()->setAmbient(osg::Vec4(0.2, 0.2, 0.2, 0.0));
    sky_node->addChild(map_node_.get());
    root_node_->addChild(sky_node);

    //locator
    osg::ref_ptr<osgEarth::Util::ObjectLocatorNode> locator = new osgEarth::Util::ObjectLocatorNode(map_node_->getMap());
    locator->setName("locator");
    locator->getLocator()->setPosition(osg::Vec3d(ground_center_location.y(), ground_center_location.x(), ground_center_location.z() + 1.8));
    sky_node->addChild(locator);

    //others
    osg::ref_ptr<osg::Switch> point_cloud_node = new osg::Switch;
    point_cloud_node->setName(point_cloud_node_name);
    locator->addChild(point_cloud_node);

    osg::ref_ptr<osg::Switch> vmap_node = new osg::Switch;
    vmap_node->setName(vmap_node_name);
    locator->addChild(vmap_node);
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
    locator->addChild(text_node);
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
    locator->addChild(temp_node);

    {
        //光照
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

    osg::ref_ptr<osgEarth::Util::EarthManipulator> em = new osgEarth::Util::EarthManipulator;
    em->setHomeViewpoint(osgEarth::Viewpoint(ground_center_location.y(), ground_center_location.x(),
                                             ground_center_location.z(), 0, -90, 1000));

    main_view_->setCameraManipulator(em);
    //main_view_->setCameraManipulator(new osgGA::TrackballManipulator);

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

void OSGWidget::loadVectorMap() {
    QDir dir("../../vmap");

    QStringList filters;
    filters << "*.csv";
    dir.setNameFilters(filters);

    category_t category = Category::NONE;
    QFileInfoList list = dir.entryInfoList();
    for(const QFileInfo& fileInfo : list){
        std::string file_path = fileInfo.filePath().toStdString();
        std::string file_name = fileInfo.fileName().toStdString();

        if (file_name == "point.csv") {
            category |= Category::POINT;

            std::vector<Point> points = m_map::parse<Point>(file_path);
            VectorMapSingleton::getInstance()->update(points);
        }
        else if (file_name == "line.csv") {
            category |= Category::LINE;

            std::vector<Line> all_lines = m_map::parse<Line>(file_path);
            VectorMapSingleton::getInstance()->update(all_lines);
        }
        else if (file_name == "area.csv") {
            category |= Category::AREA;

            std::vector<Area> areas = m_map::parse<Area>(file_path);
            VectorMapSingleton::getInstance()->update(areas);
        }
        else if (file_name == "dtlane.csv") {
            category |= Category::DTLANE;

            std::vector<dtLane> dtlanes = m_map::parse<dtLane>(file_path);
            VectorMapSingleton::getInstance()->update(dtlanes);
        }
        else if (file_name == "node.csv") {
            category |= Category::NODE;

            std::vector<Node> nodes = m_map::parse<Node>(file_path);
            VectorMapSingleton::getInstance()->update(nodes);
        }
        else if (file_name == "lane.csv") {
            category |= Category::LANE;

            std::vector<Lane> lanes = m_map::parse<Lane>(file_path);
            VectorMapSingleton::getInstance()->update(lanes);
        }
        else if (file_name == "roadedge.csv") {
            category |= Category::ROAD_EDGE;

            std::vector<RoadEdge> road_edges = m_map::parse<RoadEdge>(file_path);
            VectorMapSingleton::getInstance()->update(road_edges);
        }
        else if (file_name == "stopline.csv") {
            category |= Category::STOP_LINE;

            std::vector<StopLine> stop_lines = m_map::parse<StopLine>(file_path);
            VectorMapSingleton::getInstance()->update(stop_lines);
        }
        else if (file_name == "crosswalk.csv") {
            category |= Category::CROSS_WALK;

            std::vector<CrossWalk> cross_walks = m_map::parse<CrossWalk>(file_path);
            VectorMapSingleton::getInstance()->update(cross_walks);
        }
    }

    std::cout << "load vmap: " << std::bitset<32>(category) << " done!"<< std::endl;

    initVectorMap();
}

void OSGWidget::readPCDataFromFile(const QFileInfo& file_info){
    TRACER;
    static osg::ref_ptr<osg::Switch> point_cloud_node =
            dynamic_cast<osg::Switch*>(NodeTreeSearch::findNodeWithName(root_node_, point_cloud_node_name));

//    osg::ref_ptr<osg::Geode> geode = readPCLDataFromFile(file_info);
    osg::ref_ptr<osg::Geode> geode = readTXTDataFromFile(file_info);

    point_cloud_node->addChild(geode);
}

osg::Geode *OSGWidget::readPCLDataFromFile(const QFileInfo& file_info) const {
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCDReader reader;
    reader.read(file_info.filePath().toStdString(), *point_cloud);

    osg::ref_ptr<osg::Geode> geode = addMapPointCloud(point_cloud, osg::Vec3(0.4, 0.4, 0.4));
    geode->setName(file_info.fileName().toStdString());

    return geode.release();
}

osg::Geode* OSGWidget::addMapPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& mapPointCloud,
                                        osg::Vec3 color) const{

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

osg::Geode* OSGWidget::readTXTDataFromFile(const QFileInfo& file_info) const {
    QFile f(file_info.filePath());
    if(!f.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        std::cout << "Open failed." << std::endl;
        return nullptr;
    }

    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
    std::vector<int> intensity;

    QTextStream in(&f);
    QString lineStr;
    while(!in.atEnd())
    {
        lineStr = in.readLine();
        QStringList sections = lineStr.trimmed().split(QRegExp("[, \t]"));
        double x, y, z;
        int i;
        x = sections[0].toDouble();
        y = sections[1].toDouble();
        z = sections[2].toDouble();
        i = sections[3].toInt();
        vertices->push_back(osg::Vec3d(x, y, z));
        intensity.push_back(i);
    }
    f.close();

    osg::ref_ptr<osg::Geode> geode = addIntensityPointCloud(vertices, intensity);
    geode->setName(file_info.fileName().toStdString());

    return geode.release();
}


osg::Geode *OSGWidget::addIntensityPointCloud(const osg::ref_ptr<osg::Vec3Array>& vertices,
                                              const std::vector<int> &intensity) const {

    osg::ref_ptr<osg::Geode> geode = new osg::Geode;

    osg::ref_ptr<osg::Vec3Array> colors = new osg::Vec3Array();
    colors->reserve(intensity.size());

    //sampling raw data to speed up
    std::vector<int> mini_intensity;
    for (int i = 0; i < intensity.size(); i += 100) {
        mini_intensity.push_back(intensity[i]);
    }

    //ascending order
    std::sort(mini_intensity.begin(), mini_intensity.end());

    int min_intensity = mini_intensity[0];
    int max_intensity = mini_intensity.back();

    std::vector<osg::Vec3d> color_maps;
    float maxi = 30.0;
    float step = 1.0;
    for (float i = 1.0; i <= maxi; i += step)
    {
        color_maps.emplace_back(i / maxi, i / maxi, i / maxi);
        step += 0.5;
    }

    //section
    int section_range = mini_intensity.size() / color_maps.size(); //区间长度
    std::vector<int> intensity_ranges;
    for (int i = section_range; i < mini_intensity.size(); i += section_range)
    {
        intensity_ranges.push_back(mini_intensity[i]);
    }

    //calculate color
    for (const auto& intens : intensity)
    {
        auto iter = std::lower_bound(intensity_ranges.begin(), intensity_ranges.end(), intens);
        auto dis = std::distance(intensity_ranges.begin(), iter);
//        dis = dis == intensity_ranges.size() ? dis - 1 : dis;

        colors->push_back(color_maps[dis]);
    }


    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
    geom->setVertexArray(vertices);

    geom->setColorArray(colors);
    geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS,0,vertices->size()));
    geode->addDrawable(geom);

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


void OSGWidget::saveVectorMapToDir(const std::string& dir_path) const {
    std::vector<Point> points = VectorMapSingleton::getInstance()->findByFilter([](const Point& point) { return true; });

    geodetic_converter::GeodeticConverter gc;
    gc.initialiseReference(22.5485150000, 114.0661120000, 0);
    for(auto& point : points) {
        double x, y, h;
        x = point.ly;
        y = point.bx;
        h = point.h;
//        std::cout << "point:" << point << std::endl;
        double latitude, longitude, altitude;
        latitude = longitude = altitude = 0;
        gc.enu2Geodetic(x, y, h, &latitude, &longitude, &altitude);

//        std::cout.setf(std::ios::fixed);;
//        std::cout << std::setprecision(9) << "latitude: " << latitude << " longitude: " << longitude << " altitude: " << altitude << std::endl;

        point.b = latitude;
        point.l = longitude;
    }
    VectorMapSingleton::getInstance()->update(points);

    VectorMapSingleton::getInstance()->saveToDir(dir_path);
}

void OSGWidget::initVectorMap() {
    TRACER;

    std::vector<Line> lines = VectorMapSingleton::getInstance()->findByFilter([](const Line& object) { return true;} );
    if (!lines.empty()) drawVectorItems<Line>(lines);

    std::vector<RoadEdge> road_edges = VectorMapSingleton::getInstance()->findByFilter([](const RoadEdge& object) { return true;} );
    if (!road_edges.empty()) drawVectorItems<RoadEdge>(road_edges);

    std::vector<StopLine> stop_lines = VectorMapSingleton::getInstance()->findByFilter([](const StopLine& object) { return true;} );
    if (!stop_lines.empty()) drawVectorItems<StopLine>(stop_lines);

    std::vector<Lane> lanes = VectorMapSingleton::getInstance()->findByFilter([](const Lane& object) { return true;} );
    if (!lanes.empty()) drawTraceItems<Lane>(lanes);
}

void OSGWidget::drawAllLines() {

    std::vector<Line> all_lines = VectorMapSingleton::getInstance()->findByFilter([](const Line& line){ return true; });
    VMapDrawable vMapDrawable(root_node_);

    auto iter = all_lines.begin();
    while (iter != all_lines.end()) {
        //start_line is end_line
        if ((*iter).blid == 0 && (*iter).flid == 0) {
            ++iter;
            continue;
        }

        //start_line
        auto start_iter = std::find_if(iter, all_lines.end(), [](const Line& line) {
            return line.blid == 0;
        });

        //end_line
        auto end_iter = std::find_if(start_iter, all_lines.end(), [](const Line& line) {
            return line.flid == 0;
        });

        //move on
        iter = ++end_iter;

        //draw lines
        {
            std::vector<Line> lines;
            for (auto it = start_iter; it != end_iter + 1 && it != lines.end(); ++it) {
                lines.push_back(*it);
            }

            std::vector<Point> points;
            for (const Line& line : lines) {
                Point start_point = VectorMapSingleton::getInstance()->findByID(Key<Point>(line.bpid));
                points.push_back(start_point);
            }
            Point end_point = VectorMapSingleton::getInstance()->findByID(Key<Point>(lines.back().fpid));
            points.push_back(end_point);

//            std::cout << "***" << std::endl;
//            for (auto line : lines) {
//                std::cout << line << std::endl;;
//            }
//            std::cout << "---" << std::endl;
//            for (auto point : points) {
//                std::cout << point << std::endl;
//            }
            vMapDrawable.drawVectorNode(lines, Line());
        }
    }
}

template<class T>
void OSGWidget::drawVectorItems(const std::vector<T> &objects) {
    if (objects.empty()) return;

    std::vector<Line> all_lines;
    for (const T& object : objects) {
        Line line = VectorMapSingleton::getInstance()->findByID(Key<Line>(object.lid));
        all_lines.push_back(line);
    }
    if (all_lines.size() != objects.size()) return;
    VMapDrawable vMapDrawable(root_node_);

    auto iter = all_lines.begin();
    while (iter != all_lines.end()) {
        //start_line is end_line
        if ((*iter).blid == 0 && (*iter).flid == 0) {
            std::vector<Line> lines = { *iter };

            Point start_point = VectorMapSingleton::getInstance()->findByID(Key<Point>((*iter).bpid));
            Point end_point = VectorMapSingleton::getInstance()->findByID(Key<Point>((*iter).fpid));
            std::vector<Point> points = { start_point, end_point };

            auto index = std::distance(all_lines.begin(), iter);
            const T& object = objects[index];

            vMapDrawable.drawVectorNode(lines, object);

            ++iter;
            continue;
        }

        //start_line
        auto start_iter = std::find_if(iter, all_lines.end(), [](const Line& line) {
            return line.blid == 0;
        });

        //end_line
        auto end_iter = std::find_if(start_iter, all_lines.end(), [](const Line& line) {
            return line.flid == 0;
        });

        //move on
        iter = end_iter + 1;

        //draw lines
        {
            std::vector<Line> lines;
            for (auto it = start_iter; it != iter; ++it) {
                lines.push_back(*it);
            }

            auto index = std::distance(all_lines.begin(), start_iter);
            const T& object = objects[index];

           vMapDrawable.drawVectorNode(lines, object);
        }
    }
}

template<class T>
void OSGWidget::drawTraceItems(const std::vector<T> &objects) {
    if (objects.empty()) return;

    VMapDrawable vMapDrawable(root_node_);

    auto iter = objects.begin();
    while (iter != objects.end()) {
        auto start_iter = std::find_if(iter, objects.end(), [](const T& object) {
            return object.start_end_tag == 1;
        });

        auto end_iter = std::find_if(start_iter, objects.end(), [](const T& object) {
            return object.start_end_tag == 2;
        });

        //move on
        iter = end_iter + 1;

        //draw lanes
        {
            std::vector<T> lanes;
            for (auto it = start_iter; it != iter; ++it) {
                lanes.push_back(*it);
            }

            auto index = std::distance(objects.begin(), start_iter);
            const T& object = objects[index];

            vMapDrawable.drawTraceNode(lanes, object);
        }
    }
}


