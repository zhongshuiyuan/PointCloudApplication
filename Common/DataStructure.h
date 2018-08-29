//
// Created by WuKun on 8/28/18.
// Contact me:wk707060335@gmail.com
//

#ifndef POINTCLOUDAPPLICATION_DATASTRUCTURE_H
#define POINTCLOUDAPPLICATION_DATASTRUCTURE_H

#include <map>
#include <algorithm>
#include <functional>
#include <fstream>
#include <sstream>

typedef unsigned int uint;

namespace m_map{

struct Point{
    uint pid;
    double b, l, h;
    double bx, ly;

    Point(uint id, double x, double y){
        pid = id;
        bx = x; ly = y;
        b = l = h = 0;
    }

    Point(){
        pid = 0;
        b = l = h = bx = ly = 0;
    }
};

struct Line{
    uint lid;
    uint bpid, fpid;
    uint blid, flid;

    Line(uint id, uint bp, uint fp, uint bl, uint fl){
        lid = id;
        bpid = bp; fpid = fp;
        blid = bl; flid = fl;
    }

    Line(){
        lid = bpid = fpid = blid = flid = 0;
    }
};

struct Area{
    uint aid;
    uint slid;
    uint elid;

    Area(uint id, uint sl, uint el){
        aid = id;
        slid = sl; elid = el;
    }

    Area(){
        aid = slid = elid = 0;
    }
};

template <class T>
class Key
{
private:
    uint id_;

public:
    Key()
    {
    }

    explicit Key(uint id)
            : id_(id)
    {
    }

    void setId(uint id)
    {
        id_ = id;
    }

    uint getId() const
    {
        return id_;
    }

    bool operator<(const Key<T>& right) const
    {
        return id_ < right.getId();
    }
};

template <class T>
using Filter = std::function<bool(const T&)>;

template <class T>
class Handle
{
private:
    std::map<Key<T>, T> map_;

public:
    Handle() = default;

    T findByKey(const Key<T>& key) const
    {
        auto it = map_.find(key);
        if (it == map_.end())
            return T();
        return it->second;
    }

    std::vector<T> findByFilter(const Filter<T>& filter) const
    {
        std::vector<T> vector;
        for (const auto& pair : map_)
        {
            if (filter(pair.second))
                vector.push_back(pair.second);
        }
        return vector;
    }

    bool empty() const
    {
        return map_.empty();
    }
};

template <class T>
std::vector<T> parse(const std::string& csv_file)
{
    std::ifstream ifs(csv_file.c_str());
    std::string line;
    std::getline(ifs, line); // remove first line
    std::vector<T> objs;
    while (std::getline(ifs, line))
    {
        T obj;
        std::istringstream iss(line);
        iss >> obj;
        objs.push_back(obj);
    }
    return objs;
}

} //m_map


#endif //POINTCLOUDAPPLICATION_DATASTRUCTURE_H
