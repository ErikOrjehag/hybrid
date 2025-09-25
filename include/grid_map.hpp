#pragma once

#include <string>
#include <vector>

namespace dyno
{
class GridMap
{
public:
    GridMap() : data_({}), width_(0), height_(0), x_(0), y_(0), resolution_(1) {}
    void loadPGM(const std::string& filename, bool invert);
    void makeESDF(const GridMap& occupancy_map);
    void makeRidge(const GridMap& esdf_map, double min_clearance, double max_clearance);
    inline size_t size() const { return data_.size(); }
    inline size_t idx(size_t row, size_t col) const { return row * width_ + col; }
    inline double at(size_t row, size_t col) const { return data_.at(idx(row, col)); }
    inline double& at(size_t row, size_t col) { return data_.at(idx(row, col)); }
    void interpolate(double x, double y, double& v, double& dvx, double& dvy, double outside) const;
    inline size_t width() const { return width_; }
    inline size_t height() const { return height_; }
    inline double x() const { return x_; }
    inline double y() const { return y_; }
    inline double resolution() const { return resolution_; }
    inline void bounds(double& xmin, double& ymin, double& xmax, double& ymax) const
    {
        xmin = x_;
        ymin = y_;
        xmax = x_ + width_ * resolution_;
        ymax = y_ + height_ * resolution_;
    }
    inline bool isInside(double x, double y) const
    {
        return (x >= x_) && (x < x_ + width_ * resolution_) && (y >= y_) && (y < y_ + height_ * resolution_);
    }
    inline void worldToGrid(double x, double y, size_t& row, size_t& col) const
    {
        col = static_cast<size_t>((x - x_) / resolution_);
        row = static_cast<size_t>((y - y_) / resolution_);
    }
    inline double min()
    {
        auto min = std::min_element(data_.begin(), data_.end());
        if (min != data_.end())
            return *min;
        else
            return 0.0;
    }
    inline double max()
    {
        auto max = std::max_element(data_.begin(), data_.end());
        if (max != data_.end())
            return *max;
        else
            return 0.0;
    }
private:
    std::vector<double> data_;
    size_t width_;
    size_t height_;
    double x_;
    double y_;
    double resolution_;
};
}
