//
// Created by WuKun on 8/24/18.
// Contact me:wk707060335@gmail.com
//
#include <iostream>
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

std::ostream& operator<<(std::ostream& os, const Point& obj)
{
    os << obj.pid << ","
       << obj.b << ","
       << obj.l << ","
       << obj.h << ","
       << obj.bx << ","
       << obj.ly << ",";
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
       << obj.blid2 << ","
       << obj.blid3 << ","
       << obj.flid2 << ","
       << obj.flid3;
    return os;
}

std::ostream& operator<<(std::ostream& os, const dtLane& obj)
{
    os << obj.did << ","
       << obj.pid << ","
       << obj.apara << ","
       << obj.r;
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
    obj.blid2 = std::stoi(columns[7]);
    obj.blid3 = std::stoi(columns[8]);
    obj.flid2 = std::stoi(columns[10]);
    obj.flid3 = std::stoi(columns[11]);
    return is;
}
