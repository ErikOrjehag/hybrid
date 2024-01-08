
#include <include/voronoi.hpp>
#include <set>
#include <stdexcept>
#include <limits>
#include <cmath>

namespace dyno
{

void voronoi_field(const std::vector<std::vector<double>>& costmap, std::vector<std::vector<double>>& field)
{
    const double OCCUPIED = 0.65; // TODO

    std::set<std::pair<size_t, size_t>> queue;

    field.resize(costmap.size());
    for (size_t row = 0; row < costmap.size(); ++row)
    {
        field[row].resize(costmap[0].size());
        for (size_t col = 0; col < costmap[0].size(); ++col)
        {
            if (costmap[row][col] >= OCCUPIED)
            {
                field[row][col] = 0.0;
                if (row > 0 && costmap[row - 1][col] < OCCUPIED)
                {
                    queue.emplace(std::make_pair(row - 1, col));
                }
                if (row < costmap.size() - 1 && costmap[row + 1][col] < OCCUPIED)
                {
                    queue.emplace(std::make_pair(row + 1, col));
                }
                if (col > 0 && costmap[row][col - 1] < OCCUPIED)
                {
                    queue.emplace(std::make_pair(row, col - 1));
                }
                if (col < costmap[0].size() - 1 && costmap[row][col + 1] < OCCUPIED)
                {
                    queue.emplace(std::make_pair(row, col + 1));
                }
            }
            else
            {
                field[row][col] = std::numeric_limits<double>::infinity();
            }
        }
    }

    static const double SQRT2 = std::sqrt(2.0);

    while (!queue.empty())
    {
        std::pair<size_t, size_t> cell = *queue.begin();
        queue.erase(queue.begin());

        size_t row = cell.first;
        size_t col = cell.second;

        double min = std::numeric_limits<double>::infinity();
        if (row > 0)
        {
            min = std::min(min, field[row - 1][col] + 1.0);
        }
        if (row < costmap.size() - 1)
        {
            min = std::min(min, field[row + 1][col] + 1.0);
        }
        if (col > 0)
        {
            min = std::min(min, field[row][col - 1] + 1.0);
        }
        if (col < costmap[0].size() - 1)
        {
            min = std::min(min, field[row][col + 1] + 1.0);
        }
        if (row > 0 && col > 0)
        {
            min = std::min(min, field[row - 1][col - 1] + SQRT2);
        }
        if (row > 0 && col < costmap[0].size() - 1)
        {
            min = std::min(min, field[row - 1][col + 1] + SQRT2);
        }
        if (row < costmap.size() - 1 && col > 0)
        {
            min = std::min(min, field[row + 1][col - 1] + SQRT2);
        }
        if (row < costmap.size() - 1 && col < costmap[0].size() - 1)
        {
            min = std::min(min, field[row + 1][col + 1] + SQRT2);
        }

        if (min == std::numeric_limits<double>::infinity())
        {
            throw std::runtime_error("Voronoi field failed");
        }

        if (min < field[row][col])
        {
            field[row][col] = min;
            if (row > 0 && field[row - 1][col] != 0.0)
            {
                queue.emplace(std::make_pair(row - 1, col));
            }
            if (row < costmap.size() - 1 && field[row + 1][col] != 0.0)
            {
                queue.emplace(std::make_pair(row + 1, col));
            }
            if (col > 0 && field[row][col - 1] != 0.0)
            {
                queue.emplace(std::make_pair(row, col - 1));
            }
            if (col < costmap[0].size() - 1 && field[row][col + 1] != 0.0)
            {
                queue.emplace(std::make_pair(row, col + 1));
            }
        }
    }

    int x = 0;
}

}