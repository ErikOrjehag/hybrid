#pragma once

#include <string>
#include <vector>

namespace dyno
{
    double normalize_angle(double angle);

    void load_pgm(const std::string& filename, std::vector<std::vector<double>>& costmap);

} // namespace dyno