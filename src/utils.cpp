
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

void load_pgm(const std::string& filename, std::vector<std::vector<double>>& costmap)
{
    std::ifstream file(filename);
    std::string line;

    std::getline(file, line);

    if (line.compare("P5") != 0)
    {
        throw std::runtime_error("Invalid PGM file version");
    }

    do {
        std::getline(file, line);
    } while (line[0] == '#');

    std::stringstream ss;
    ss << line;
    size_t width;
    size_t height;
    ss >> width >> height;

    std::getline(file, line);

    ss.clear();
    ss << line;
    size_t max_val;
    ss >> max_val;

    costmap.resize(height);
    for (size_t row = height; row --> 0 ;)
    {
        costmap[row].resize(width);
        for (size_t col = 0; col < width; ++col)
        {
            costmap[row][col] = 1.0 - (double)file.get() / (double)max_val;
        }
    }
}

} // namespace dyno