#pragma once

#include <string>
#include <vector>

namespace dyno
{
class GridMap
{
public:
    GridMap() : data_({}), width_(0), height_(0), x_(0), y_(0), resolution_(1) {}
    void loadPGM(const std::string& filename);
    inline double& at(size_t row, size_t col) { return data_.at(row * width_ + col); }
    inline size_t width() const { return width_; }
    inline size_t height() const { return height_; }
    inline double x() const { return x_; }
    inline double y() const { return y_; }
    inline double resolution() const { return resolution_; }
private:
    std::vector<double> data_;
    size_t width_;
    size_t height_;
    double x_;
    double y_;
    double resolution_;
};
}
