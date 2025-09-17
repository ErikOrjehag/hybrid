
#include "include/grid_map.hpp"
#include <fstream>
#include <sstream>
#include <queue>
#include <cmath>

namespace dyno
{
void GridMap::loadPGM(const std::string& yaml_filename, bool invert)
{
    // Load PGM, occupancy if invert = true
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
            double val = (double)pgm_file.get() / (double)max_val;
            if (invert)
            {
                val = 1.0 - val;
            }
            at(row, col) = val;
        }
    }

    pgm_file.close();
}

void GridMap::makeESDF(const GridMap& occupancy_map)
{
    // Euclidean Signed Distance Field using 8-connectivity Dijkstra

    struct Node
    {
        double distance;
        size_t row;
        size_t col;
        bool operator<(const Node& other) const { return distance > other.distance; }
    };

    auto esdf_dijkstra8 = [&](std::vector<double>& field, bool invert)
    {
        field.resize(occupancy_map.size(), std::numeric_limits<double>::infinity());
        std::priority_queue<Node> queue;
        std::vector<bool> visited(occupancy_map.width() * occupancy_map.height(), false);

        for (size_t row = 0; row < occupancy_map.height(); ++row)
        {
            for (size_t col = 0; col < occupancy_map.width(); ++col)
            {
                bool is_obstacle = occupancy_map.at(row, col) > 0.5;
                if (is_obstacle != invert)
                {
                    field[occupancy_map.idx(row, col)] = 0.0;
                    queue.push({0.0, row, col});
                }
            }
        }

        static const double SQRT2 = std::sqrt(2.0);
        static const int drow[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
        static const int dcol[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
        static const double dcost[8] = {SQRT2, 1.0, SQRT2, 1.0, 1.0, SQRT2, 1.0, SQRT2};

        while (!queue.empty())
        {
            Node current = queue.top();
            queue.pop();

            if (visited[occupancy_map.idx(current.row, current.col)])
            {
                continue;
            }
            visited[occupancy_map.idx(current.row, current.col)] = true;

            for (size_t k = 0; k < 8; ++k)
            {
                int nrow = static_cast<int>(current.row) + drow[k];
                int ncol = static_cast<int>(current.col) + dcol[k];
                if (nrow >= 0 && nrow < static_cast<int>(occupancy_map.height()) && ncol >= 0 && ncol < static_cast<int>(occupancy_map.width()))
                {
                    double new_distance = current.distance + dcost[k] * occupancy_map.resolution();
                    if (new_distance < field[occupancy_map.idx(nrow, ncol)])
                    {
                        field[occupancy_map.idx(nrow, ncol)] = new_distance;
                        queue.push({new_distance, static_cast<size_t>(nrow), static_cast<size_t>(ncol)});
                    }
                }
            }
        }
    };

    // obstacles as sources
    std::vector<double> d_out;
    esdf_dijkstra8(d_out, false);
    // free as sources
    std::vector<double> d_in;
    esdf_dijkstra8(d_in, true);

    data_.resize(occupancy_map.size());
    width_ = occupancy_map.width();
    height_ = occupancy_map.height();
    x_ = occupancy_map.x();
    y_ = occupancy_map.y();
    resolution_ = occupancy_map.resolution();
    for (size_t row = 0; row < height_; ++row)
    {
        for (size_t col = 0; col < width_; ++col)
        {
            double dist_out = d_out[occupancy_map.idx(row, col)];
            double dist_in = d_in[occupancy_map.idx(row, col)];
            if (occupancy_map.at(row, col) > 0.5)
            {
                at(row, col) = -dist_in;
            }
            else
            {
                at(row, col) = dist_out;
            }
        }
    }
    
}

void GridMap::interpolate(double x, double y, double& v, double& dvx, double& dvy, double outside) const
{
    // Bilinear interpolation with gradient

    double gy = (y - y_) / resolution_ - 0.5;
    double gx = (x - x_) / resolution_ - 0.5;
    int ix = static_cast<int>(std::floor(gx));
    int iy = static_cast<int>(std::floor(gy));

    if (ix < 0 || ix + 1 >= static_cast<int>(width_) || iy < 0 || iy + 1 >= static_cast<int>(height_))
    {
        v = outside;
        dvx = 0.0;
        dvy = 0.0;
        return;
    }

    double tx = gx - ix;
    double ty = gy - iy;

    // v_xy
    double v00 = at(iy, ix);
    double v10 = at(iy, ix + 1);
    double v01 = at(iy + 1, ix);
    double v11 = at(iy + 1, ix + 1);

    double v0 = (1.0 - tx) * v00 + tx * v10;
    double v1 = (1.0 - tx) * v01 + tx * v11;
    v = (1.0 - ty) * v0 + ty * v1;
    dvx = ((1.0 - ty) * (v10 - v00) + ty * (v11 - v01)) / resolution_;
    dvy = ((1.0 - tx) * (v01 - v00) + tx * (v11 - v10)) / resolution_;
}

void GridMap::makeRidge(const GridMap& esdf_map, double min_clearance, double max_clearance)
{
    // Ridge detection

    const double delta = 1.0; // sampling step as multiples of cell size (δ = delta * resolution) (1.0 - 2.0 is a good range)
    const double tol = 1e-3; // tolerance for local max test (meters) (1e-4 to 5e-3 is a good range)
    const double curv_thresh = -1e-3; // 2nd-derivative threshold (negative = concave ridge)
    const double cos_tresh = -0.3; // cosine of angle threshold between gradients (negative = >90 degrees, concave ridge)

    data_.resize(esdf_map.size(), 0.0);
    width_ = esdf_map.width();
    height_ = esdf_map.height();
    x_ = esdf_map.x();
    y_ = esdf_map.y();
    resolution_ = esdf_map.resolution();

    auto normalize = [](double& x, double& y, double eps=1e-12) {
        double norm = std::sqrt(x * x + y * y);
        if (norm < eps) {
            x = 0.0;
            y = 0.0;
        } else {
            x /= norm;
            y /= norm;
        }
    };

    for (size_t row = 0; row < height_; ++row)
    {
        for (size_t col = 0; col < width_; ++col)
        {
            double x = x_ + (col + 0.5) * resolution_;
            double y = y_ + (row + 0.5) * resolution_;
            double v0, dvx0, dvy0;
            esdf_map.interpolate(x, y, v0, dvx0, dvy0, 0.0);

            if (v0 < min_clearance)
            {
                continue;
            }

            if (v0 > max_clearance)
            {
                continue;
            }
            
            normalize(dvx0, dvy0);

            if (std::abs(dvx0) < 1e-6 && std::abs(dvy0) < 1e-6)
            {
                continue;
            }

            double gx = dvx0;
            double gy = dvy0;

            // sample points along normal direction
            double x1 = x + delta * resolution_ * gx;
            double y1 = y + delta * resolution_ * gy;
            double x2 = x - delta * resolution_ * gx;
            double y2 = y - delta * resolution_ * gy;
            double v1, dvx1, dvy1;
            double v2, dvx2, dvy2;
            esdf_map.interpolate(x1, y1, v1, dvx1, dvy1, 0.0);
            esdf_map.interpolate(x2, y2, v2, dvx2, dvy2, 0.0);
            
            bool nms = (v0 > v1 - tol) && (v0 > v2 - tol); // non-maximum suppression
            if (!nms)
            {
                continue;
            }

            double d2g = v1 - 2.0*v0 + v2; // 2nd derivative along gradient direction
            bool concave = d2g < curv_thresh;

            if (!concave)
            {
                continue;
            }

            normalize(dvx1, dvy1);
            normalize(dvx2, dvy2);
            if ((std::abs(dvx1) < 1e-6 && std::abs(dvy1) < 1e-6) || (std::abs(dvx2) < 1e-6 && std::abs(dvy2) < 1e-6))
            {
                continue;
            }
            double cos_angle = dvx1*dvx2 + dvy1*dvy2; // cosine of angle between gradients
            bool opposing = cos_angle < cos_tresh;

            if (!opposing)
            {
                continue;
            }

            at(row, col) = 1.0; // mark as ridge
        }
    }
}

} // namespace dyno