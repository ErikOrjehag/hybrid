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
    std::vector<double> lengths;  // length of each part of the path
    std::vector<char> types;      // type of each part of the path
    double L;                     // total length
    std::vector<double> x;        // x positions [m] of each part of the path
    std::vector<double> y;        // y positions [m] of each part of the path
    std::vector<double> yaw;      // yaw angles [rad] of each part of the path
    std::vector<int> directions;  // 1: forward, -1: backward
};

void add_path(
    std::vector<Path>& paths,
    const std::vector<double>& lengths,
    const std::vector<char>& types
    );

void shortest_path(
    double start_x,
    double start_y,
    double start_yaw,
    double end_x,
    double end_y,
    double end_yaw,
    double max_curvature,
    double step_size,
    Path& path
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
void CSC(double x, double y, double yaw, std::vector<Path>& paths);
void CCC(double x, double y, double yaw, std::vector<Path>& paths);
void CCCC(double x, double y, double yaw, std::vector<Path>& paths);
void CCSC(double x, double y, double yaw, std::vector<Path>& paths);
void CCSCC(double x, double y, double yaw, std::vector<Path>& paths);

bool SLS(double x, double y, double yaw, double& t, double& u, double& v);
bool LSL(double x, double y, double yaw, double& t, double& u, double& v);
bool LSR(double x, double y, double yaw, double& t, double& u, double& v);
bool LRL(double x, double y, double yaw, double& t, double& u, double& v);
bool LRLRn(double x, double y, double yaw, double& t, double& u, double& v);
bool LRLRp(double x, double y, double yaw, double& t, double& u, double& v);
bool LRSL(double x, double y, double yaw, double& t, double& u, double& v);
bool LRSR(double x, double y, double yaw, double& t, double& u, double& v);
bool LRSLR(double x, double y, double yaw, double& t, double& u, double& v);

std::tuple<double, double> calc_tau_omega(double u, double v, double xi, double eta, double phi);

} // namespace rs

} // namespace dyno