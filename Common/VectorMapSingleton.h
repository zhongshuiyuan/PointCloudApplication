//
// Created by WuKun on 8/24/18.
// Contact me:wk707060335@gmail.com
//

#ifndef POINTCLOUDAPPLICATION_VECTORMAPSINGLETON_H
#define POINTCLOUDAPPLICATION_VECTORMAPSINGLETON_H

#include <map>
#include <algorithm>
#include <functional>

#include "DataStructure.h"

using m_map::Point;
using m_map::Line;
using m_map::Area;
using m_map::Key;
using m_map::Handle;
using m_map::Filter;

using category_t = unsigned long;
enum Category : category_t
{
    NONE = 0LLU,

    // Graphical Primitive Class
    POINT = 1LLU << 0,
    LINE = 1LLU << 1,
    AREA = 1LLU << 2,

    // Road Data
    DTLANE = 1LLU << 3,
    NODE = 1LLU << 4,
    LANE = 1LLU << 5,

    // Object Data
    ROAD_EDGE = 1LLU << 6,
    STOP_LINE = 1LLU << 7,
    CROSS_WALK = 1LLU << 9,

    ALL = (1LLU << 16) - 1
};

class VectorMapSingleton {

public:
    static VectorMapSingleton* getInstance();

    size_t getMaxPointIndex() const;
    size_t getMaxLineIndex() const;
    size_t getMaxAreaIndex() const;

    void update(const std::vector<Point>& points);
    void update(const std::vector<Line>& lines);
    void update(const std::vector<Area>& areas);

    Point findByID(const Key<Point>& key) const;
    Line  findByID(const Key<Line>& key) const;
    Area  findByID(const Key<Area>& key) const;

    std::vector<Point> findByFilter(const Filter<Point>& filter) const;
    std::vector<Line>  findByFilter(const Filter<Line>& filter) const;
    std::vector<Area>  findByFilter(const Filter<Area>& filter) const;

private:
    static VectorMapSingleton* instance;

    Handle<Point> points_;
    Handle<Line>  lines_;
    Handle<Area>  areas_;
};

std::ostream& operator<<(std::ostream& os, const Point& obj);
std::ostream& operator<<(std::ostream& os, const Line& obj);
std::ostream& operator<<(std::ostream& os, const Area& obj);

std::istream& operator>>(std::istream& is, Point& obj);
std::istream& operator>>(std::istream& is, Line& obj);
std::istream& operator>>(std::istream& is, Area& obj);

#endif //POINTCLOUDAPPLICATION_VECTORMAPSINGLETON_H
