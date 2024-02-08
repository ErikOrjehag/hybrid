
#include <boost/heap/binomial_heap.hpp>
#include <Eigen/Dense>

namespace dyno
{

struct Cell;

struct CompareCell {
    bool operator()(const Cell* lhs, const Cell* rhs) const;
};

typedef boost::heap::binomial_heap<
    Cell*,
    boost::heap::compare<CompareCell>
> PriorityQueue;

void astart_search(
    std::vector<std::vector<double>>& costmap,
    std::vector<std::vector<double>>& field,
    size_t start_row,
    size_t start_col,
    size_t goal_row,
    size_t goal_col,
    std::vector<Eigen::Vector2i>& path,
    std::vector<Cell>& out_cells
);

struct State
{
    size_t row;
    size_t col;

    bool operator==(const State& other) const
    {
        return row == other.row && col == other.col;
    }
};

struct StateHashFn
{
    StateHashFn(size_t rows, size_t cols)
        : rows(rows), cols(cols)
    {
    }

    size_t operator()(const State& s) const
    {
        return s.row * cols + s.col;
    }

private:
    size_t rows;
    size_t cols;
};

struct Cell
{
    Cell() : Cell({0, 0})
    {
        
    }

    Cell(State state)
        : state(state),
          g(std::numeric_limits<double>::infinity()),
          f(std::numeric_limits<double>::infinity()),
          came_from(nullptr),
          pq_handle(),
          is_o(false),
          is_c(false)
    {
    }

    State state;

    double g;
    double f;

    Cell* came_from;

    PriorityQueue::handle_type pq_handle;

    bool is_o; // open
    bool is_c; // closed

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