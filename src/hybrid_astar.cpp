#include "include/hybrid_astar.hpp"
#include <cmath>

namespace dyno
{
namespace hybrid_a_star
{
MotionPrimitives precompute_motion_primatives()
{
    // Create a table with all combinations of initial turning rates and velocities that are evenly spaced within the defined limits.
    // The table should include both positive and negative turning rates, as well as zero.
    // The table should also include both positive and negative velocities, as well as zero.
    // For each combination, simulate the motion of the robot for a fixed number of time steps, using a simple kinematic model.
    // Store the resulting trajectories in a data structure that can be used to look up the trajectory for a given initial turning rate and velocity.
    // The trajectory is local to the robot frame (i.e., starting at (x=0,y=0,theta=0)).
    // Kinematics model is:
    // dx/dt = v * cos(yaw)
    // dy/dt = v * sin(yaw)
    // dyaw/dt = turning_rate
    std::vector<double> turning_rates;
    std::vector<double> velocities;
    for (int i = 0; i < NUM_TURNING_RATES; ++i)
    {
        double tr = -MAX_TURNING_RATE + i * (2 * MAX_TURNING_RATE) / (NUM_TURNING_RATES - 1);
        turning_rates.push_back(tr);
    }
    for (int i = 0; i < NUM_VELOCITIES; ++i)
    {
        // double v = -MAX_VELOCITY + i * (2 * MAX_VELOCITY) / (NUM_VELOCITIES - 1);
        // double v = i * (MAX_VELOCITY) / (NUM_VELOCITIES - 1);
        double v = MIN_VELOCITY + i * (MAX_VELOCITY - MIN_VELOCITY) / (NUM_VELOCITIES - 1);
        velocities.push_back(v);
    }
    MotionPrimitives motion_primatives;
    for (size_t i = 0; i < turning_rates.size(); ++i)
    {
        for (size_t j = 0; j < velocities.size(); ++j)
        {
            double turning_rate = turning_rates[i];
            double velocity = velocities[j];
            Trajectory trajectory;
            PrimState state = {0.0, 0.0, 0.0, turning_rate, velocity};
            trajectory.push_back(state);
            for (int step = 1; step < NUM_STEPS; ++step)
            {
                PrimState new_state;
                new_state.turning_rate = turning_rate;
                new_state.velocity = velocity;
                new_state.yaw = state.yaw + state.turning_rate * DT;
                new_state.x = state.x + state.velocity * std::cos(state.yaw) * DT;
                new_state.y = state.y + state.velocity * std::sin(state.yaw) * DT;
                trajectory.push_back(new_state);
                state = new_state;
            }
            motion_primatives[{i, j}] = trajectory;
        }
    }

    return motion_primatives;
}

class HybridAStarSearch
{
public:
    HybridAStarSearch(
        const GridMap& esdf,
        const MotionPrimitives& motion_primatives,
        int max_iterations,
        int yield_every_n_iterations
    ) : esdf_(esdf),
        motion_primatives_(motion_primatives),
        max_iterations_(max_iterations),
        yield_every_n_iterations_(yield_every_n_iterations),
        pool_(),
        open_set_(),
        grid_(esdf, 72)
    {
    }

    void start(
        double start_x,
        double start_y,
        double start_yaw,
        double goal_x,
        double goal_y,
        double goal_yaw
    )
    {
        start_x_ = start_x;
        start_y_ = start_y;
        start_yaw_ = start_yaw;
        goal_x_ = goal_x;
        goal_y_ = goal_y;
        goal_yaw_ = goal_yaw;

        goal_reached_idx_ = -1;
        iterations_ = 0;

        pool_.clear();
        pool_.reserve(200'000);

        open_set_.clear();
        open_set_.reserve(10'000);

        best_g_cost_.clear();
        best_g_cost_.resize(grid_.size(), std::numeric_limits<double>::infinity());

        size_t start_idx = grid_.worldToIdx(start_x, start_y, start_yaw);
        size_t goal_idx = grid_.worldToIdx(goal_x, goal_y, goal_yaw);

        double g0 = 0.0;
        double f0 = g0 + heuristic(start_x, start_y, start_yaw);
        size_t n0 = pool_.push_back(start_x, start_y, start_yaw, g0, f0, -1, -1, -1);
        best_g_cost_[start_idx] = g0;
        open_set_.push(n0, f0);

        step();
    }

    void step()
    {
        while (!open_set.empty())
        {
            if (iterations++ > max_iterations)
            {
                break;
            }

            size_t current_pool_index = open_set.pop();
            double current_x = pool.x[current_pool_index];
            double current_y = pool.y[current_pool_index];
            double current_yaw = pool.yaw[current_pool_index];
            size_t current_grid_index = grid.worldToIdx(current_x, current_y, current_yaw);

            if (!grid.isInside(current_x, current_y))
            {
                continue;
            }

            if (pool.g_cost[current_pool_index] > best_g_cost[current_grid_index])
            {
                continue;
            }
            
            double dx = goal_x - current_x;
            double dy = goal_y - current_y;
            double d_pos = std::sqrt(dx * dx + dy * dy);
            double d_yaw = std::abs(goal_yaw - current_yaw);
            d_yaw = std::fmod(d_yaw + M_PI, 2 * M_PI) - M_PI; // wrap to [-pi, pi]
            if (d_pos < 0.5/* && std::abs(d_yaw) < (15.0 * M_PI / 180.0)*/)
            {
                goal_reached_idx = current_pool_index;
                break; // goal reached
            }

            double cos_current_yaw = std::cos(current_yaw);
            double sin_current_yaw = std::sin(current_yaw);

            for (const auto& [prim_idx, trajectory] : motion_primatives)
            {
                int turning_rate_index = prim_idx.first;
                int velocity_index = prim_idx.second;

                const PrimState& last_state = trajectory.back();
                double new_x = current_x + last_state.x * cos_current_yaw - last_state.y * sin_current_yaw;
                double new_y = current_y + last_state.x * sin_current_yaw + last_state.y * cos_current_yaw;
                double new_yaw = current_yaw + last_state.yaw;
                new_yaw = std::fmod(new_yaw + M_PI, 2 * M_PI) - M_PI; // wrap to [-pi, pi]

                if (!grid.isInside(new_x, new_y))
                {
                    continue;
                }

                bool collision = false;
                for (const auto& state : trajectory)
                {
                    double check_x = current_x + state.x * cos_current_yaw - state.y * sin_current_yaw;
                    double check_y = current_y + state.x * sin_current_yaw + state.y * cos_current_yaw;
                    size_t row, col;
                    grid.worldToGridRowCol(check_x, check_y, row, col);
                    if (esdf.at(row, col) < ROBOT_RADIUS)
                    {
                        collision = true;
                        break;
                    }
                }
                if (collision)
                {
                    continue;
                }

                double step_cost = 0.0;
                for (const auto& state : trajectory)
                {
                    double check_x = current_x + state.x * cos_current_yaw - state.y * sin_current_yaw;
                    double check_y = current_y + state.x * sin_current_yaw + state.y * cos_current_yaw;
                    size_t row, col;
                    grid.worldToGridRowCol(check_x, check_y, row, col);
                    double clearance = esdf.at(row, col);
                    if (clearance < ROBOT_RADIUS)
                    {
                        step_cost += 1e6; // high cost for being too close to obstacles
                    }
                    else
                    {
                        step_cost += 1.0 / clearance; // cost inversely proportional to clearance
                    }
                }
                step_cost += trajectory.size() * DT; // time cost
                double new_g_cost = pool.g_cost[current_pool_index] + step_cost;

                if (new_g_cost >= best_g_cost[grid.worldToIdx(new_x, new_y, new_yaw)])
                {
                    continue; // not a better path
                }

                size_t new_grid_index = grid.worldToIdx(new_x, new_y, new_yaw);
                best_g_cost[new_grid_index] = new_g_cost;
                double new_f_cost = new_g_cost + heuristic(new_x, new_y, new_yaw);
                size_t new_pool_index = pool.push_back(new_x, new_y, new_yaw, new_g_cost, new_f_cost, current_pool_index, turning_rate_index, velocity_index);
                open_set.push(new_pool_index, new_f_cost);
            }
        }
    }

    double heuristic(double x, double y, double yaw) const
    {
        double dx = goal_x_ - x;
        double dy = goal_y_ - y;
        double d_pos = std::sqrt(dx * dx + dy * dy);
        double d_yaw = std::abs(goal_yaw_ - yaw);
        d_yaw = std::fmod(d_yaw + M_PI, 2 * M_PI) - M_PI; // wrap to [-pi, pi]
        return d_pos/* + std::abs(d_yaw)*/; // weight position and yaw equally
    }

    void getPath(std::vector<PathNode>& path)
    {
        path.clear();
        if (goal_reached_idx_ < 0)
        {
            return;
        }
        int idx = goal_reached_idx_;
        while (idx >= 0)
        {
            PathNode node;
            node.x = pool.x[idx];
            node.y = pool.y[idx];
            node.yaw = pool.yaw[idx];
            path.push_back(node);
            idx = pool.parent[idx];
        }
        std::reverse(path.begin(), path.end());
    }

private:
    const GridMap esdf_;
    const MotionPrimitives motion_primatives_;
    int max_iterations_;
    int yield_every_n_iterations_;
    
    NodePool pool_;
    MinHeap open_set_;
    DiscreteGrid grid_;
    std::vector<double> best_g_cost_;

    double start_x_;
    double start_y_;
    double start_yaw_;
    double goal_x_;
    double goal_y_;
    double goal_yaw_;
    int goal_reached_idx_;
    int iterations_;
};

} // namespace hybrid_a_star
} // namespace dyno
