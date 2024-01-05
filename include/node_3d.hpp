#pragma once

#include <boost/heap/binomial_heap.hpp>
#include <cmath>

class Node3D;

struct CompareNode3D {
  bool operator()(const Node3D* lhs, const Node3D* rhs) const;
};

typedef boost::heap::binomial_heap<Node3D*,
        boost::heap::compare<CompareNode3D>> PriorityQueue;

class Node3D
{
public:
    Node3D();
    Node3D(double x, double y, double t, double g, double h, Node3D* parent);

    Node3D* create_child(size_t i);
    double cost() const;
    size_t update_index(size_t width, size_t height, size_t depth);
    void update_g();
    bool is_open();
    bool is_closed();
    void open();
    void close();

    double x;
    double y;
    double t;
    double g;
    double h;
    Node3D* parent;
    PriorityQueue::handle_type handle;
    size_t index;

    const static size_t N_DIRS = 3;
private:
    // R = 6, 6.75 DEG:
    // constexpr static double DY[3] = { 0,        0.0415893,  -0.0415893};
    // constexpr static double DX[3] = { 0.7068582,   0.705224,   0.705224};
    // constexpr static double DT[3] = { 0,         0.1178097,   -0.1178097};
    constexpr static double DY[3] = { 0,        0.48361648,  -0.48361648};
    constexpr static double DX[3] = { 1.414,   1.32872537,   1.32872537};
    constexpr static double DT[3] = { 0,         0.3490658503988659,   -0.3490658503988659};

    bool is_o; // open
    bool is_c; // closed
};
