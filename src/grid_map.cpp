
#include "include/grid_map.hpp"
#include <fstream>
#include <sstream>

namespace dyno
{
void GridMap::loadPGM(const std::string& yaml_filename)
{
    // @arg path to yaml file

    std::ifstream yaml_file(yaml_filename);

    if (!yaml_file.is_open())
    {
        throw std::runtime_error("Failed to open YAML file");
    }

    std::string pgm_filename_relative;

    std::stringstream ss;
    std::string line;

    while (std::getline(yaml_file, line))
    {
        ss.clear();
        ss.str(line);
        std::string key;
        ss >> key;
        if (key == "image:")
        {
            ss >> pgm_filename_relative;
        }
        else if (key == "resolution:")
        {
            ss >> resolution_;
        }
        else if (key == "origin:")
        {
            // origin is in the form origin: [-79.5, -216, 0]
            char ch;
            ss >> ch; // [
            ss >> x_;
            ss >> ch; // ,
            ss >> y_;
            // we ignore the last value
        }
    }

    yaml_file.close();

    std::string pgm_filename = yaml_filename.substr(0, yaml_filename.find_last_of("/\\") + 1) + pgm_filename_relative;

    std::ifstream pgm_file(pgm_filename);
    if (!pgm_file.is_open())
    {
        throw std::runtime_error("Failed to open PGM file");
    }

    std::getline(pgm_file, line);

    if (line.compare("P5") != 0)
    {
        throw std::runtime_error("Invalid PGM file version");
    }

    do {
        std::getline(pgm_file, line);
    } while (line[0] == '#');

    ss.clear();
    ss.str(line);
    ss >> width_ >> height_;

    std::getline(pgm_file, line);

    ss.clear();
    ss.str(line);
    size_t max_val;
    ss >> max_val;

    data_.resize(width_ * height_);

    for (size_t row = height_; row --> 0 ;)
    {
        for (size_t col = 0; col < width_; ++col)
        {
            at(row, col) = (double)pgm_file.get() / (double)max_val;
        }
    }

    pgm_file.close();


}
} // namespace dyno