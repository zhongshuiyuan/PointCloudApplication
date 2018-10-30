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
#include <iostream>

constexpr double RADIUS_MAX = 90000000000;
namespace m_map{

struct Point{
    size_t pid;
    double b, l, h;
    double bx, ly;

    size_t ref, mcode1, mcode2, mcode3;

    Point(size_t id, double x, double y, double _h){
        pid = id;
        bx = x; ly = y; h = _h;
        b = l = 0;
        ref = mcode1 = mcode2 = mcode3 = 0;
    }

    Point(){
        pid = 0;
        b = l = h = bx = ly = 0;;
        ref = mcode1 = mcode2 = mcode3 = 0;
    }
};

struct Line{
    size_t lid;
    size_t bpid, fpid;
    size_t blid, flid;

    Line(size_t id, size_t bp, size_t fp, size_t bl, size_t fl){
        lid = id;
        bpid = bp; fpid = fp;
        blid = bl; flid = fl;
    }

    Line(){
        lid = bpid = fpid = blid = flid = 0;
    }
};

struct Area{
    size_t aid;
    size_t slid;
    size_t elid;

    Area(size_t id, size_t sl, size_t el){
        aid = id;
        slid = sl; elid = el;
    }

    Area(){
        aid = slid = elid = 0;
    }
};

struct Node {
    size_t nid;
    size_t pid;

    Node() {
        nid = pid = 0;
    }
    Node(size_t n, size_t p) {
        nid = n;
        pid = p;
    }
};

struct Lane {
    size_t lnid, did;
    size_t blid, flid;
    size_t bnid, fnid;
    size_t jct;
    size_t blid2, flid2, blid3, flid3, blid4, flid4;
    size_t clossid, span, lcnt, lno, lanetype, limitvel, refvel, roadsecid, lanecfgfg;
    size_t start_end_tag;

    Lane() {
        lnid = did = blid = flid = bnid = fnid = 0;
        jct = blid2 = flid2 = blid3 = flid3 = blid4 = flid4 = 0;

        clossid = span = lcnt = lno = lanetype = roadsecid = lanecfgfg = 0;
        limitvel = refvel = 60;
        start_end_tag = 0;
    }

    Lane(size_t _lnid, size_t _did, size_t _blid, size_t _flid, size_t _bnid, size_t _fnid) {
        lnid = _lnid; did = _did;
        blid = _blid; flid = _flid;
        bnid = _bnid; fnid = _fnid;
        jct = blid2 = flid2 = blid3 = flid3 = blid4 = flid4 = 0;

        clossid = span = lcnt = lno = lanetype = roadsecid = lanecfgfg = 0;
        limitvel = refvel = 60;
        start_end_tag = 0;
    }
};

struct dtLane {
    size_t did;
    size_t dist;
    size_t pid;
    double dir;
    double apara;
    double r;

    double slope, cant, lw, rw;

    dtLane() {
        did = dist = pid = 0;
        dir = apara = r = 0;
        slope = cant = lw = rw = 0;
    }

    dtLane(size_t _did, size_t _dist, size_t _pid, double _a, double _r) {
        did = _did; dist = _dist; pid = _pid;
        apara = _a; r = _r;
        dir = 0;
        slope = cant = lw = rw = 0;
    }
};

struct RoadEdge {
    size_t id;
    size_t lid, linkid;

    RoadEdge() {
        id = lid = linkid = 0;
    }

    RoadEdge(size_t _id, size_t _lid) {
        id = _id;
        lid = _lid;
        linkid = 0;
    }
};

struct CrossWalk {
    size_t id, aid;
    size_t type, bdid;
    size_t linkid;

    CrossWalk() {
        id = aid = type = bdid = linkid = 0;
    }

    CrossWalk(size_t _id, size_t _aid) {
        id = _id;
        aid = _aid;
        type = 0;
        linkid = bdid = 0;
    }
};

struct StopLine {
    size_t id;
    size_t lid, tlid;
    size_t signid, linkid;

    StopLine() {
        id = lid = tlid = signid = linkid = 0;
    }

    StopLine(size_t _id, size_t _lid) {
        id = _id;
        lid = _lid;
        linkid = tlid = signid = 0;
    }
};

template <class T>
class Key
{
private:
    size_t id_;

public:
    Key() = default;

    explicit Key(size_t id)
            : id_(id) {
    }

    void setId(size_t id) {
        id_ = id;
    }

    size_t getId() const {
        return id_;
    }

    bool operator<(const Key<T>& right) const {
        return id_ < right.getId();
    }

    bool operator==(const Key<T>& right) const {
        return id_ == right.getId();
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

    void update(const Key<T>& key, const T& t){
        //map_.insert(std::make_pair(key, t));
        //std::map::insert() function doesn't change pre-existing key-value
        //unless you do it in explicit way
        map_[key] = t;

        //C++17
        //auto [it, inserted] = map_.insert_or_assign(key, t);
    }

    void remove(const Key<T>& key) {
        map_.erase(key);
    }

    T findByKey(const Key<T>& key) const {
        auto it = map_.find(key);
        if (it == map_.end())
            return T();
        return it->second;
    }

    std::vector<T> findByFilter(const Filter<T>& filter) const {
        std::vector<T> vector;
        for (const auto& pair : map_)
        {
            if (filter(pair.second))
                vector.push_back(pair.second);
        }
        return vector;
    }

    bool empty() const {
        return map_.empty();
    }

    size_t findMaxIndex() const {
        size_t index = 0;
        for (const auto& pair : map_)
        {
            size_t cur_index = std::get<0>(pair).getId();
            if (cur_index > index) index = cur_index;
        }
        return index;
    }

    void print() const {
        std::cout << "---------" << std::endl;
        for (const auto& pair : map_) {
            const Key<T>& key = std::get<0>(pair);
            const T& obj = std::get<1>(pair);
            std::cout << key.getId() << "--"<< obj << std::endl;
        }

        //C++17
        //for (const auto& [key, value] : map_) {
        //   std::cout << key.getId() << "--"<< value << std::endl;
        //}
    }

    void output(const std::string& csv_file, const std::string& header) const {
        std::ofstream ofs(csv_file);
        ofs << header << std::endl;

        for (const auto& pair : map_) {
            const T& obj = std::get<1>(pair);
            ofs << obj << std::endl;
        }
        ofs.close();
    }

    size_t size() const {
        return map_.size();
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
