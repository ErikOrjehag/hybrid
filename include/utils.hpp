#pragma once

#include <string>
#include <vector>

namespace dyno
{
    double mod2pi(double angle);
    double pi2pi(double angle);

    void load_pgm(const std::string& filename, std::vector<std::vector<double>>& costmap);

} // namespace dyno