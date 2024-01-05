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
    std::vector<double> lengths;  // TODO
    std::vector<std::string> ctypes; // Type of each part of the path
    double L; // Total length
    std::vector<double> x; // final x positions [m]
    std::vector<double> y; // final y positions [m]
    std::vector<double> yaw; // final yaw angles [rad]
    std::vector<int> directions; // 1: forward, -1: backward
};

void generate_paths(double sx, double sy, double syaw, double ex, double ey, double eyaw, double maxc, const std::vector<Path>& paths)
{
    double dx = ex - sx;
    double dy = ey - sy;
    double dyaw = eyaw - syaw; // TODO: is this OK?
    double c = std::cos(syaw);
    double s = std::sin(syaw);
    double x = (c * dx + s * dy) * maxc;
    double y = (-s * dx + c * dy) * maxc;

    SCS(x, y, dyaw, paths);

    // paths = CSC(x, y, dth, paths)
    // paths = CCC(x, y, dth, paths)
    // paths = CCCC(x, y, dth, paths)
    // paths = CCSC(x, y, dth, paths)
    // paths = CCSCC(x, y, dth, paths)
}

std::tuple<bool, double, double, double> SLS(double x, double y, double yaw)
{
    yaw = dyno::normalize_angle(yaw);
    
    if (y != 0.0 && yaw > 0.0 && yaw < 0.99*M_PI)
    {
        double xd = -y / std::tan(yaw) + x;
        double t = xd - std::tan(yaw / 2.0);
        double u = yaw;
        double v = std::sqrt(std::pow(x - xd, 2) + std::pow(y, 2)) - std::tan(yaw / 2.0);
        
        if (v < 0.0)
        {
            v *= -1;
        }

        return {true, t, u, v};
    }

    return {false, 0.0, 0.0, 0.0};
}

void SCS(double x, double y, double yaw, const std::vector<Path>& paths)
{
    const auto [ok, t, u, v] = SLS(x, y, yaw);
}

} // namespace rs

} // namespace dyno