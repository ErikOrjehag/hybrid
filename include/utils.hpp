#pragma once

#include <string>
#include <vector>
#include <tuple>

namespace dyno
{
    double mod2pi(double angle);
    double pi2pi(double angle);
    std::tuple<double, double> polar(double x, double y);


    void load_pgm(const std::string& filename, std::vector<std::vector<double>>& costmap);

} // namespace dyno