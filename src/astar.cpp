
#include "include/astar.hpp"

namespace dyno
{

bool CompareNode::operator()(const Node* lhs, const Node* rhs) const
{
    return lhs->cost() > rhs->cost();
}

void astart_search(
    std::vector<std::vector<double>>& costmap,
    std::vector<std::vector<double>>& field,
    size_t start_row,
    size_t start_col,
    size_t goal_row,
    size_t goal_col
)
{
    PriorityQueue pq;

    
}

} // namespace dyno
