
#include "include/astar.hpp"
#include <cmath>
#include <iostream>

namespace dyno
{

bool CompareCell::operator()(const Cell* lhs, const Cell* rhs) const
{
    return lhs->f > rhs->f;
}

void astart_search(
    std::vector<std::vector<double>>& costmap,
    std::vector<std::vector<double>>& field,
    size_t start_row,
    size_t start_col,
    size_t goal_row,
    size_t goal_col,
    std::vector<Eigen::Vector2i>& path,
    std::vector<Cell>& out_cells
)
{
    const size_t cols = costmap.size();
    const size_t rows = costmap[0].size();
    const size_t N = cols * rows;

    State goal_state{goal_row, goal_col};

    auto heuristic_cost_fn = [](const State& s1, const State& s2) -> double
    {
        // return std::sqrt(
        //     std::pow(static_cast<double>(s1.row) - static_cast<double>(s2.row), 2) +
        //     std::pow(static_cast<double>(s1.col) - static_cast<double>(s2.col), 2)
        // );
        return std::abs(static_cast<double>(s1.row) - static_cast<double>(s2.row)) + std::abs(static_cast<double>(s1.col) - static_cast<double>(s2.col));
    };

    auto cost_fn = [&costmap](const State& s1, const State& s2) -> double
    {
        return std::sqrt(
            std::pow(s1.row - s2.row, 2) +
            std::pow(s1.col - s2.col, 2)
        ); // + costmap[s2.row][s2.col];
    };

    auto find_neighbors = [rows, cols, &costmap](const State& s, std::unordered_map<State, Cell, StateHashFn>& cells, std::vector<Cell*>& neighbors)
    {
        neighbors.clear();

        for (int i = -1; i <= 1; ++i)
        {
            for (int j = -1; j <= 1; ++j)
            {
                if (i == 0 && j == 0)
                {
                    continue;
                }

                Cell n({s.row + i, s.col + j});

                if (n.state.row < 0 || n.state.row >= rows || n.state.col < 0 || n.state.col >= cols)
                {
                    continue;
                }

                if (costmap[n.state.col][n.state.row] > 0.2)
                {
                    continue;
                }

                cells.try_emplace(n.state, n);
                neighbors.push_back(&cells[n.state]);
            }
        }
    };

    PriorityQueue frontier;
    auto add_to_frontier = [&frontier](Cell* cell)
    {
        cell->open();
        frontier.push(cell);
    };

    std::unordered_map<State, Cell, StateHashFn> cells(N, StateHashFn(rows, cols));
    
    {
        Cell s({start_row, start_col});
        s.g = 0.0;
        s.f = s.g + heuristic_cost_fn(s.state, {goal_row, goal_col});
        cells[s.state] = s;
        add_to_frontier(&cells[s.state]);
    }

    Cell* goal = nullptr;

    int itr = 1'000'000;

    std::vector<Cell*> neighbors;
    neighbors.reserve(8);

    while (!frontier.empty() && itr --> 0)
    {
        Cell* current = nullptr;
        while (!frontier.empty())
        {
            current = frontier.top();
            frontier.pop();
            if (current->is_open())
            {
                break;
            }
            else
            {
                current = nullptr;
            }
        }

        if (current == nullptr)
        {
            break;
        }

        current->close();

        if (current->state == goal_state)
        {
            goal = current;
            break;
        }

        find_neighbors(current->state, cells, neighbors);

        for (Cell* neighbor : neighbors)
        {
            if (neighbor->is_closed())
            {
                continue;
            }

            double tentative_g = current->g + cost_fn(current->state, neighbor->state);

            if (tentative_g < neighbor->g)
            {
                neighbor->came_from = current;
                neighbor->g = tentative_g;
                neighbor->f = neighbor->g + heuristic_cost_fn(neighbor->state, goal_state);

                add_to_frontier(neighbor);
            }
        }
    }

    path.clear();

    if (goal != nullptr)
    {
        std::cout << "Found goal" << std::endl;
        Cell* current = goal;
        while (current != nullptr)
        {
            path.push_back({current->state.row, current->state.col});
            current = current->came_from;
        }
    }

    out_cells.clear();
    for (auto& [_, cell] : cells)
    {
        out_cells.push_back(cell);
    }
}

} // namespace dyno
