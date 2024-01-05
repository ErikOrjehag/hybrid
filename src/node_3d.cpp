#include "include/node_3d.hpp"
#include "include/utils.hpp"

Node3D::Node3D() : Node3D(0.0, 0.0, 0.0, 0.0, 0.0, nullptr) {};

Node3D::Node3D(double x, double y, double t, double g, double h, Node3D* parent)
: is_o{false}, is_c{false}, index{0}, parent{nullptr}
{
    this->x = x;
    this->y = y;
    this->t = t;
    this->g = g; // TODO: dont need initialization?
    this->h = h; // TODO: dont need initialization?
    this->parent = parent; // TODO: dont need initialization?
}

bool Node3D::is_open()
{
    return is_o;
}

bool Node3D::is_closed()
{
    return is_c;
}

void Node3D::open()
{
    is_o = true;
    is_c = false;
}

void Node3D::close()
{
    is_o = false;
    is_c = true;
}

double Node3D::cost() const
{
    return g + h;
}

Node3D* Node3D::create_child(size_t i)
{
    double nx;
    double ny;
    double nt;

    // forward
    if (i < 3) {
        nx = x + DX[i] * std::cos(t) - DY[i] * std::sin(t);
        ny = y + DX[i] * std::sin(t) + DY[i] * std::cos(t);
        nt = dyno::normalize_angle(t + DT[i]);
    }
    // backwards
    else
    {
        nx = x - DX[i - 3] * std::cos(t) - DY[i - 3] * std::sin(t);
        ny = y - DX[i - 3] * std::sin(t) + DY[i - 3] * std::cos(t);
        nt = dyno::normalize_angle(t - DT[i - 3]);
    }

    return new Node3D(nx, ny, nt, g, 0, this);
}

size_t Node3D::update_index(size_t width, size_t height, size_t depth)
{
    index = (size_t)(t / (2.0 * M_PI) * depth) * width * height + (size_t)(y) * width + (size_t)(x);
    return index;
}

void Node3D::update_g()
{
    // TODO: Move complexity from turning and reversing
    g = parent->g + DX[0];
}

bool CompareNode3D::operator()(const Node3D* lhs, const Node3D* rhs) const {
    return lhs->cost() > rhs->cost();
}