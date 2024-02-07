
#include <boost/heap/binomial_heap.hpp>

namespace dyno
{

struct Node;

struct CompareNode {
    bool operator()(const Node* lhs, const Node* rhs) const;
};

typedef boost::heap::binomial_heap<
    Node*,
    boost::heap::compare<CompareNode>
> PriorityQueue;

void astart_search(
    std::vector<std::vector<double>>& costmap,
    std::vector<std::vector<double>>& field,
    size_t start_row,
    size_t start_col,
    size_t goal_row,
    size_t goal_col
);

struct Node
{
    size_t row;
    size_t col;
    double g;
    double h;
    double f;
    Node* parent;
    PriorityQueue::handle_type handle;
    bool is_o; // open
    bool is_c; // closed

    double cost() const
    {
        return g + h;
    }

    bool is_open()
    {
        return is_o;
    }

    bool is_closed()
    {
        return is_c;
    }

    void open()
    {
        is_o = true;
        is_c = false;
    }

    void close()
    {
        is_o = false;
        is_c = true;
    }
};

}