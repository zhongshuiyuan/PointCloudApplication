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

Point VectorMapSingleton::findByID(const Key<Point> &key) const {
    return points_.findByKey(key);
}

Line VectorMapSingleton::findByID(const Key<Line> &key) const {
    return lines_.findByKey(key);
}

Area VectorMapSingleton::findByID(const Key<Area> &key) const {
    return areas_.findByKey(key);
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