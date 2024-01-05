#pragma once

#include <vector>
#include <string>
#include <cmath>
#include <tuple>
#include "include/utils.hpp"

namespace dyno
{

namespace rs
{

struct Path {
    std::tuple<double, double, double> lengths;  // length of each part of the path
    std::tuple<char, char, char> types;          // type of each part of the path
    double L;                                    // total length
    std::vector<double> x;                       // x positions [m] of each part of the path
    std::vector<double> y;                       // y positions [m] of each part of the path
    std::vector<double> yaw;                     // yaw angles [rad] of each part of the path
    std::vector<int> directions;                 // 1: forward, -1: backward
};

void add_path(
    std::vector<Path>& paths,
    const std::tuple<double, double, double>& lengths,
    const std::tuple<char, char, char>& types
    );

void generate_paths(
    double start_x,
    double start_y,
    double start_yaw,
    double end_x,
    double end_y,
    double end_yaw,
    double max_curvature,
    double step_size,
    std::vector<Path>& paths
    );

void populate_local_path_course(
    Path& path,
    double start_x,
    double start_y,
    double start_yaw,
    double max_curvature,
    double step_size
    );

void interpolate(
    size_t ind,
    double l,
    double type,
    double max_curvature,
    double ox,
    double oy,
    double oyaw,
    std::vector<double>& x,
    std::vector<double>& y,
    std::vector<double>& yaw,
    std::vector<int>& directions
    );

void SCS(double x, double y, double yaw, std::vector<Path>& paths);

std::tuple<bool, std::tuple<double, double, double>> SLS(double x, double y, double yaw);

} // namespace rs

} // namespace dyno