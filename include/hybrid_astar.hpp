
#pragma once

#include <vector>
#include <map>
#include <cmath>
#include "include/grid_map.hpp"

namespace dyno
{
namespace hybrid_a_star
{

const double MAX_TURNING_RATE = 0.9; // rad/s
// const double MAX_VELOCITY = 1.3; // m/s
// const double MIN_VELOCITY = 0.5; // m/s
const double MAX_VELOCITY = 1.0; // m/s
const double MIN_VELOCITY = 1.0; // m/s
const double DT = 0.1; // s

const int NUM_TURNING_RATES = 5;
// const int NUM_VELOCITIES = 3;
const int NUM_VELOCITIES = 1;
const int NUM_STEPS = 10;

struct PrimState
{
    double x;
    double y;
    double yaw;
    double turning_rate;
    double velocity;
};

typedef std::vector<PrimState> Trajectory;
typedef std::pair<size_t, size_t> InitialCondition; // (turning_rate_index, velocity_index)
typedef std::map<InitialCondition, Trajectory> MotionPrimitives;

MotionPrimitives precompute_motion_primatives();

// SoA
struct NodePool
{
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> yaw;
    std::vector<double> g_cost; // cost from start to current node
    std::vector<double> f_cost; // g_cost + heuristic cost to goal
    std::vector<int> parent;
    std::vector<int> prim_turning_rate_index;
    std::vector<int> prim_velocity_index;

    size_t size() const { return x.size(); }
    size_t push_back(double _x, double _y, double _yaw, double _g_cost, double _f_cost, int _parent, int _prim_turning_rate_index, int _prim_velocity_index)
    {
        const size_t idx = x.size();
        x.push_back(_x);
        y.push_back(_y);
        yaw.push_back(_yaw);
        g_cost.push_back(_g_cost);
        f_cost.push_back(_f_cost);
        parent.push_back(_parent);
        prim_turning_rate_index.push_back(_prim_turning_rate_index);
        prim_velocity_index.push_back(_prim_velocity_index);
        return idx;
    }

    void reserve(size_t n)
    {
        x.reserve(n);
        y.reserve(n);
        yaw.reserve(n);
        g_cost.reserve(n);
        f_cost.reserve(n);
        parent.reserve(n);
        prim_turning_rate_index.reserve(n);
        prim_velocity_index.reserve(n);
    }

    void clear()
    {
        x.clear();
        y.clear();
        yaw.clear(),
        g_cost.clear();
        f_cost.clear();
        parent.clear();
        prim_turning_rate_index.clear();
        prim_velocity_index.clear();
    }
};

struct HeapItem
{
    size_t node_index;
    double f_cost;
};

static inline bool min_heap_cmp(const HeapItem& a, const HeapItem& b)
{
    return a.f_cost > b.f_cost; // min-heap
}

class MinHeap
{
public:
    void reserve(size_t n)
    {
        data_.reserve(n);
    }

    void clear()
    {
        data_.clear();
    }

    const std::vector<HeapItem>& data() const
    {
        return data_;
    }

    void push(size_t node_index, double f_cost)
    {
        data_.push_back({node_index, f_cost});
        std::push_heap(data_.begin(), data_.end(), min_heap_cmp);
    }

    int pop()
    {
        std::pop_heap(data_.begin(), data_.end(), min_heap_cmp);
        int node_index = data_.back().node_index;
        data_.pop_back();
        return node_index;
    }

    bool empty() const
    {
        return data_.empty();
    }

    int peek() const
    {
        return data_.back().node_index;
    }

private:
    std::vector<HeapItem> data_;
};

struct PathNode
{
    double x;
    double y;
    double yaw;
};

class DiscreteGrid
{
public:
    DiscreteGrid(const GridMap& map, int num_yaw_bins)
        : width_(map.width()), height_(map.height()), num_yaw_bins_(num_yaw_bins),
          resolution_(map.resolution()), x_(map.x()), y_(map.y())
    {
    }

    inline size_t size() const { return width_ * height_ * num_yaw_bins_; }

    inline size_t idx(size_t row, size_t col, size_t yaw_bin) const
    {
        return (row * width_ + col) * num_yaw_bins_ + yaw_bin;
    }

    inline bool isInside(double x, double y) const
    {
        return (x >= x_) && (x < x_ + width_ * resolution_) && (y >= y_) && (y < y_ + height_ * resolution_);
    }

    inline void worldToGrid(double x, double y, double yaw, size_t& row, size_t& col, size_t& yaw_bin) const
    {
        worldToGridRowCol(x, y, row, col);
        yaw_bin = static_cast<size_t>(std::round(yaw / (2 * M_PI) * num_yaw_bins_)) % num_yaw_bins_;
    }

    inline void worldToGridRowCol(double x, double y, size_t& row, size_t& col) const
    {
        col = static_cast<size_t>((x - x_) / resolution_);
        row = static_cast<size_t>((y - y_) / resolution_);
    }

    inline size_t worldToIdx(double x, double y, double yaw) const
    {
        size_t row, col, yaw_bin;
        worldToGrid(x, y, yaw, row, col, yaw_bin);
        return idx(row, col, yaw_bin);
    }

private:
    int width_;
    int height_;
    int num_yaw_bins_;
    double resolution_;
    double x_;
    double y_;
};

double signed_diff_angle(double theta1, double theta2);

double abs_diff_angle(double theta1, double theta2);

class HybridAStarSearch
{
public:
    HybridAStarSearch(
        const GridMap& esdf,
        const MotionPrimitives& motion_primatives,
        int max_iterations,
        int yield_every_n_iterations);

    void reset();

    void setStartPose(double start_x, double start_y, double start_yaw);

    void getStartPose(double& start_x, double& start_y, double& start_yaw) const;

    void setGoalPose(double goal_x, double goal_y, double goal_yaw);

    void getGoalPose(double& goal_x, double& goal_y, double& goal_yaw) const;



    void start();

    void step();

    bool isSearching() const;
    bool foundGoal() const;

    double heuristic(double x, double y, double yaw) const;
    
    void getGoalPath(std::vector<PathNode>& path) const;

    void getCurrentPath(std::vector<PathNode>& path) const;

    void getFrontier(std::vector<PathNode>& frontier, size_t limit) const
    {
        // return up to 'limit' nodes from the open set with the lowest f_cost
        frontier.clear();
        std::vector<HeapItem> open_set_data = open_set_.data();
        std:sort(open_set_data.begin(), open_set_data.end(), min_heap_cmp);
        std::reverse(open_set_data.begin(), open_set_data.end());
        for (size_t i = 0; i < std::min(limit, open_set_data.size()); ++i)
        {
            const auto& item = open_set_data[i];
            int pool_index = item.node_index;
            double x = pool_.x[pool_index];
            double y = pool_.y[pool_index];
            double yaw = pool_.yaw[pool_index];
            size_t grid_index = grid_.worldToIdx(x, y, yaw);

            if (pool_.g_cost[pool_index] < best_g_cost_[grid_index])
            {
                continue;
            }

            PathNode node;
            node.x = pool_.x[pool_index];
            node.y = pool_.y[pool_index];
            node.yaw = pool_.yaw[pool_index];
            frontier.push_back(node);
        }
    }

private:
    void backTracePath(std::vector<PathNode>& path, int pool_index, bool include_primitives) const;

    const GridMap& esdf_;
    const MotionPrimitives& motion_primatives_;
    int max_iterations_;
    int yield_every_n_iterations_;
    
    NodePool pool_;
    MinHeap open_set_;
    DiscreteGrid grid_;
    std::vector<double> best_g_cost_;

    double start_x_;
    double start_y_;
    double start_yaw_;
    double goal_x_;
    double goal_y_;
    double goal_yaw_;

    int goal_reached_idx_;
    int iterations_;
    int current_pool_index_;
};

} // namespace hybrid_a_star
} // namespace dyno