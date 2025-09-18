
#include "include/utils.hpp"
#include <cmath>
#include <fstream>
#include <sstream>

namespace dyno
{
 
double mod2pi(double angle)
{
    while (angle < 0.0)
    {
        angle += 2 * M_PI;
    }
    while (angle >= 2.0 * M_PI)
    {
        angle -= 2.0 * M_PI;
    }
    return angle;
}

double pi2pi(double angle)
{
    while (angle < -M_PI)
    {
        angle += 2 * M_PI;
    }

    while (angle > M_PI)
    {
        angle -= 2 * M_PI;
    }

    return angle;
}

std::tuple<double, double> polar(double x, double y)
{
    double r = std::sqrt(x * x + y * y);
    double theta = std::atan2(y, x);
    return {r, theta};
}

} // namespace dyno