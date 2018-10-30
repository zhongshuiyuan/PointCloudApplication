//
// Created by WuKun on 8/24/18.
// Contact me:wk707060335@gmail.com
//
#include <iostream>
#include <iomanip>
#include <fstream>

#include "VectorMapSingleton.h"

//static member initialize: <data type> <class name>::<member name>=<value>
VectorMapSingleton* VectorMapSingleton::instance = new VectorMapSingleton();

VectorMapSingleton* VectorMapSingleton::getInstance() {
    return instance;
}

size_t VectorMapSingleton::getMaxPointIndex() const {
    return points_.findMaxIndex();
}

size_t VectorMapSingleton::getMaxLineIndex() const {
    return lines_.findMaxIndex();
}

size_t VectorMapSingleton::getMaxAreaIndex() const {
    return areas_.findMaxIndex();
}

size_t VectorMapSingleton::getMaxNodeIndex() const {
    return nodes_.findMaxIndex();
}

size_t VectorMapSingleton::getMaxLaneIndex() const {
    return lanes_.findMaxIndex();
}

size_t VectorMapSingleton::getMaxDtLaneIndex() const {
    return dtlanes_.findMaxIndex();
}

size_t VectorMapSingleton::getMaxRoadEdgeIndex() const {
    return roadedges_.findMaxIndex();
}

size_t VectorMapSingleton::getMaxCrossWalkIndex() const {
    return crosswalks_.findMaxIndex();
}

size_t VectorMapSingleton::getMaxStopLineIndex() const {
    return stoplines_.findMaxIndex();
}

void VectorMapSingleton::update(const std::vector<Point> &points) {
    for (const auto& point : points)
    {
        if (point.pid == 0)
            continue;
        points_.update(Key<Point>(point.pid), point);
    }
}

void VectorMapSingleton::update(const std::vector<Line> &lines) {
    for (const auto& line : lines)
    {
        if (line.lid == 0)
            continue;
        lines_.update(Key<Line>(line.lid), line);
    }
}

void VectorMapSingleton::update(const std::vector<Area> &areas) {
    for (const auto& area : areas)
    {
        if (area.aid == 0)
            continue;
        areas_.update(Key<Area>(area.aid), area);
    }
}

void VectorMapSingleton::update(const std::vector<Node> &nodes) {
    for (const auto& node : nodes)
    {
        if (node.nid == 0)
            continue;
        nodes_.update(Key<Node>(node.nid), node);
    }
}

void VectorMapSingleton::update(const std::vector<Lane> &lanes) {
    for (const auto& lane : lanes)
    {
        if (lane.lnid == 0)
            continue;
        lanes_.update(Key<Lane>(lane.lnid), lane);
    }
}

void VectorMapSingleton::update(const std::vector<dtLane> &dtlanes) {
    for (const auto& dtlane : dtlanes)
    {
        if (dtlane.did == 0)
            continue;
        dtlanes_.update(Key<dtLane>(dtlane.did), dtlane);
    }
}

void VectorMapSingleton::update(const std::vector<RoadEdge> &roadedges) {
    for (const auto& roadedge : roadedges)
    {
        if (roadedge.id == 0)
            continue;
        roadedges_.update(Key<RoadEdge>(roadedge.id), roadedge);
    }
}
void VectorMapSingleton::update(const std::vector<CrossWalk> &crosswalks) {
    for (const auto& crosswalk : crosswalks)
    {
        if (crosswalk.id == 0)
            continue;
        crosswalks_.update(Key<CrossWalk>(crosswalk.id), crosswalk);
    }
}
void VectorMapSingleton::update(const std::vector<StopLine> &stoplines) {
    for (const auto& stopline : stoplines)
    {
        if (stopline.id == 0)
            continue;
        stoplines_.update(Key<StopLine>(stopline.id), stopline);
    }
}

Point VectorMapSingleton::findByID(const Key<Point> &key) const {
    return points_.findByKey(key);
}

Line VectorMapSingleton::findByID(const Key<Line> &key) const {
    return lines_.findByKey(key);
}

Area VectorMapSingleton::findByID(const Key<Area> &key) const {
    return areas_.findByKey(key);
}

Node VectorMapSingleton::findByID(const Key<Node> &key) const {
    return nodes_.findByKey(key);
}

Lane VectorMapSingleton::findByID(const Key<Lane> &key) const {
    return lanes_.findByKey(key);
}

dtLane VectorMapSingleton::findByID(const Key<dtLane> &key) const {
    return dtlanes_.findByKey(key);
}

RoadEdge VectorMapSingleton::findByID(const Key<RoadEdge> &key) const {
    return roadedges_.findByKey(key);
}

CrossWalk VectorMapSingleton::findByID(const Key<CrossWalk> &key) const {
    return crosswalks_.findByKey(key);
}

StopLine VectorMapSingleton::findByID(const Key<StopLine> &key) const {
    return stoplines_.findByKey(key);
}

std::vector<Point> VectorMapSingleton::findByFilter(const Filter<Point>& filter) const
{
    return points_.findByFilter(filter);
}

std::vector<Line> VectorMapSingleton::findByFilter(const Filter<Line>& filter) const
{
    return lines_.findByFilter(filter);
}

std::vector<Area> VectorMapSingleton::findByFilter(const Filter<Area>& filter) const
{
    return areas_.findByFilter(filter);
}

std::vector<Node> VectorMapSingleton::findByFilter(const Filter<Node>& filter) const
{
    return nodes_.findByFilter(filter);
}

std::vector<Lane> VectorMapSingleton::findByFilter(const Filter<Lane>& filter) const
{
    return lanes_.findByFilter(filter);
}

std::vector<dtLane> VectorMapSingleton::findByFilter(const Filter<dtLane>& filter) const
{
    return dtlanes_.findByFilter(filter);
}

std::vector<RoadEdge> VectorMapSingleton::findByFilter(const Filter<RoadEdge>& filter) const
{
    return roadedges_.findByFilter(filter);
}

std::vector<CrossWalk> VectorMapSingleton::findByFilter(const Filter<CrossWalk>& filter) const
{
    return crosswalks_.findByFilter(filter);
}

std::vector<StopLine> VectorMapSingleton::findByFilter(const Filter<StopLine>& filter) const
{
    return stoplines_.findByFilter(filter);
}

void VectorMapSingleton::saveToDir(const std::string& dir_path) const {

    //point
    {
        std::string header = "PID,B,L,H,Bx,Ly,ReF,MCODE1,MCODE2,MCODE3";
        std::string file_path = dir_path + "/point.csv";

        points_.output(file_path, header);
    }

    //line
    {
        std::string header = "LID,BPID,FPID,BLID,FLID";
        std::string file_path = dir_path + "/line.csv";

        lines_.output(file_path, header);
    }

    //area
    {
        std::string header = "AID,SLID,ELID";
        std::string file_path = dir_path + "/area.csv";

        areas_.output(file_path, header);
    }

    //node
    {
        std::string header = "NID,PID";
        std::string file_path = dir_path + "/node.csv";

        nodes_.output(file_path, header);
    }

    //lane
    {
        std::string header = "LnID,DID,BLID,FLID,BNID,FNID,JCT,BLID2,BLID3,BLID4,FLID2,FLID3,FLID4,"
                             "ClossID,Span,LCnt,Lno,LaneType,LimitVel,RefVel,RoadSecID,LaneChgFG,Start_End_tag";
        std::string file_path = dir_path + "/lane.csv";

        lanes_.output(file_path, header);
    }

    //dtlane
    {
        std::string header = "DID,Dist,PID,Dir,Apara,r,slope,cant,LW,RW";
        std::string file_path = dir_path + "/dtlane.csv";

        dtlanes_.output(file_path, header);
    }

    //roadedge
    {
        std::string header = "ID,LID,LinkID";
        std::string file_path = dir_path + "/roadedge.csv";

        roadedges_.output(file_path, header);
    }

    //crosswalk
    {
        std::string header = "ID,AID,Type,BdID,LinkID";
        std::string file_path = dir_path + "/crosswalk.csv";

        crosswalks_.output(file_path, header);
    }

    //stopline
    {
        std::string header = "ID,LID,TLID,SignID,LinkID";
        std::string file_path = dir_path + "/stopline.csv";

        stoplines_.output(file_path, header);
    }
}

void VectorMapSingleton::remove(const Key<Line> &key) {
    lines_.remove(key);
}

void VectorMapSingleton::remove(const Key<RoadEdge> &key) {
    roadedges_.remove(key);
}

void VectorMapSingleton::remove(const Key<StopLine>& key) {
    stoplines_.remove(key);
}

void VectorMapSingleton::remove(const Key<CrossWalk>& key) {
    crosswalks_.remove(key);
}

void VectorMapSingleton::remove(const Key<Lane> &key) {
    lanes_.remove(key);
}

void VectorMapSingleton::remove(const Key<dtLane> &key) {
    dtlanes_.remove(key);
}

std::ostream& operator<<(std::ostream& os, const Point& obj)
{
    os << obj.pid << ",";

    os.setf(std::ios::fixed);
    os << std::setprecision(9)
       << obj.b << ","
       << obj.l << ",";
    os.unsetf(std::ios::fixed);

    os << obj.h << ","
       << obj.bx << ","
       << obj.ly << ","
       << obj.ref << ","
       << obj.mcode1 << ","
       << obj.mcode2 << ","
       << obj.mcode3;
    return os;
}

std::ostream& operator<<(std::ostream& os, const Line& obj)
{
    os << obj.lid << ","
       << obj.bpid << ","
       << obj.fpid << ","
       << obj.blid << ","
       << obj.flid;
    return os;
}

std::ostream& operator<<(std::ostream& os, const Area& obj)
{
    os << obj.aid << ","
       << obj.slid << ","
       << obj.elid;
    return os;
}

std::ostream& operator<<(std::ostream& os, const Node& obj)
{
    os << obj.nid << ","
       << obj.pid;
    return os;
}

std::ostream& operator<<(std::ostream& os, const Lane& obj)
{
    os << obj.lnid << ","
       << obj.did << ","
       << obj.blid << ","
       << obj.flid << ","
       << obj.bnid << ","
       << obj.fnid << ","
       << obj.jct << ","
       << obj.blid2 << ","
       << obj.blid3 << ","
       << obj.blid4 << ","
       << obj.flid2 << ","
       << obj.flid3 << ","
       << obj.flid4 << ","
       << obj.clossid << ","
       << obj.span << ","
       << obj.lcnt << ","
       << obj.lno << ","
       << obj.lanetype << ","
       << obj.limitvel << ","
       << obj.refvel << ","
       << obj.roadsecid << ","
       << obj.lanecfgfg << ","
       << obj.start_end_tag;
    return os;
}

std::ostream& operator<<(std::ostream& os, const dtLane& obj)
{
    os << obj.did << ","
       << obj.dist << ","
       << obj.pid << ","
       << obj.dir << ","
       << obj.apara << ","
       << obj.r << ","
       << obj.slope << ","
       << obj.cant << ","
       << obj.lw << ","
       << obj.rw;
    return os;
}

std::ostream& operator<<(std::ostream& os, const RoadEdge& obj)
{
    os << obj.id << ","
       << obj.lid << ","
       << obj.linkid;
    return os;
}

std::ostream& operator<<(std::ostream& os, const StopLine& obj)
{
    os << obj.id << ","
       << obj.lid << ","
       << obj.tlid << ","
       << obj.signid << ","
       << obj.linkid;
    return os;
}

std::ostream& operator<<(std::ostream& os, const CrossWalk& obj)
{
    os << obj.id << ","
       << obj.aid << ","
       << obj.type << ","
       << obj.bdid << ","
       << obj.linkid;
    return os;
}

std::istream& operator>>(std::istream& is, Point& obj)
{
    std::vector<std::string> columns;
    std::string column;
    while (std::getline(is, column, ','))
    {
        columns.push_back(column);
    }
    obj.pid = std::stoi(columns[0]);
    obj.b = std::stod(columns[1]);
    obj.l = std::stod(columns[2]);
    obj.h = std::stod(columns[3]);
    obj.bx = std::stod(columns[4]);
    obj.ly = std::stod(columns[5]);
    obj.ref = std::stoi(columns[6]);
    obj.mcode1 = std::stoi(columns[7]);
    obj.mcode2 = std::stoi(columns[8]);
    obj.mcode3 = std::stoi(columns[9]);
    return is;
}

std::istream& operator>>(std::istream& is, Line& obj)
{
    std::vector<std::string> columns;
    std::string column;
    while (std::getline(is, column, ','))
    {
        columns.push_back(column);
    }
    obj.lid = std::stoi(columns[0]);
    obj.bpid = std::stoi(columns[1]);
    obj.fpid = std::stoi(columns[2]);
    obj.blid = std::stoi(columns[3]);
    obj.flid = std::stoi(columns[4]);
    return is;
}

std::istream& operator>>(std::istream& is, Area& obj)
{
    std::vector<std::string> columns;
    std::string column;
    while (std::getline(is, column, ','))
    {
        columns.push_back(column);
    }
    obj.aid = std::stoi(columns[0]);
    obj.slid = std::stoi(columns[1]);
    obj.elid = std::stoi(columns[2]);
    return is;
}

std::istream& operator>>(std::istream& is, Node& obj)
{
    std::vector<std::string> columns;
    std::string column;
    while (std::getline(is, column, ','))
    {
        columns.push_back(column);
    }
    obj.nid = std::stoi(columns[0]);
    obj.pid = std::stoi(columns[1]);
    return is;
}

std::istream& operator>>(std::istream& is, Lane& obj)
{
    std::vector<std::string> columns;
    std::string column;
    int n = 0;
    while (std::getline(is, column, ','))
    {
        columns.push_back(column);
        ++n;
    }
    obj.lnid = std::stoi(columns[0]);
    obj.did = std::stoi(columns[1]);
    obj.blid = std::stoi(columns[2]);
    obj.flid = std::stoi(columns[3]);
    obj.bnid = std::stoi(columns[4]);
    obj.fnid = std::stoi(columns[5]);
    obj.jct = std::stoi(columns[6]);
    obj.blid2 = std::stoi(columns[7]);
    obj.blid3 = std::stoi(columns[8]);
    obj.blid4 = std::stoi(columns[9]);
    obj.flid2 = std::stoi(columns[10]);
    obj.flid3 = std::stoi(columns[11]);
    obj.flid4 = std::stoi(columns[12]);
    obj.clossid = std::stoi(columns[13]);
    obj.span = std::stod(columns[14]);
    obj.lcnt = std::stoi(columns[15]);
    obj.lno = std::stoi(columns[16]);
    obj.lanetype = std::stoi(columns[17]);
    obj.limitvel = std::stoi(columns[18]);
    obj.refvel = std::stoi(columns[19]);
    obj.roadsecid = std::stoi(columns[20]);
    obj.lanecfgfg = std::stoi(columns[21]);
    obj.start_end_tag = std::stoi(columns[22]);
    return is;
}

std::istream& operator>>(std::istream& is, dtLane& obj)
{
    std::vector<std::string> columns;
    std::string column;
    while (std::getline(is, column, ','))
    {
        columns.push_back(column);
    }
    obj.did = std::stoi(columns[0]);
    obj.dist = std::stod(columns[1]);
    obj.pid = std::stoi(columns[2]);
    obj.dir = std::stod(columns[3]);
    obj.apara = std::stod(columns[4]);
    obj.r = std::stod(columns[5]);
    obj.slope = std::stod(columns[6]);
    obj.cant = std::stod(columns[7]);
    obj.lw = std::stod(columns[8]);
    obj.rw = std::stod(columns[9]);
    return is;
}

std::istream& operator>>(std::istream& is, RoadEdge& obj)
{
    std::vector<std::string> columns;
    std::string column;
    while (std::getline(is, column, ','))
    {
        columns.push_back(column);
    }
    obj.id = std::stoi(columns[0]);
    obj.lid = std::stoi(columns[1]);
    obj.linkid = std::stoi(columns[2]);
    return is;
}

std::istream& operator>>(std::istream& is, StopLine& obj)
{
    std::vector<std::string> columns;
    std::string column;
    while (std::getline(is, column, ','))
    {
        columns.push_back(column);
    }
    obj.id = std::stoi(columns[0]);
    obj.lid = std::stoi(columns[1]);
    obj.tlid = std::stoi(columns[2]);
    obj.signid = std::stoi(columns[3]);
    obj.linkid = std::stoi(columns[4]);
    return is;
}

std::istream& operator>>(std::istream& is, CrossWalk& obj)
{
    std::vector<std::string> columns;
    std::string column;
    while (std::getline(is, column, ','))
    {
        columns.push_back(column);
    }
    obj.id = std::stoi(columns[0]);
    obj.aid = std::stoi(columns[1]);
    obj.type = std::stoi(columns[2]);
    obj.bdid = std::stoi(columns[3]);
    obj.linkid = std::stoi(columns[4]);
    return is;
}