#pragma once

#include <ompl/geometric/SimpleSetup.h>
#include "include/grid_map.hpp"
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/StateSampler.h>
#include <random>
#include "include/grid_map.hpp"


namespace dyno
{
namespace ompl_planner
{

class MyStateValidityChecker : public ompl::base::StateValidityChecker
{
public:
    MyStateValidityChecker(const ompl::base::SpaceInformationPtr& si, const GridMap& esdf_map)
        : ompl::base::StateValidityChecker(si), esdf_map_(esdf_map)
    {}
    virtual bool isValid(const ompl::base::State* state) const override
    {
        const auto* dubins_state = state->as<ompl::base::SE2StateSpace::StateType>();
        double x = dubins_state->getX();
        double y = dubins_state->getY();
        if (!esdf_map_.isInside(x, y))
            return false;
        size_t row, col;
        esdf_map_.worldToGrid(x, y, row, col);
        double v = esdf_map_.at(row, col);
        return v > 0.5; // robot radius is 0.5m
    }
private:
    const GridMap& esdf_map_;
};

class MySampler : public ompl::base::StateSampler
{
public:
    MySampler(const ompl::base::StateSpace* space, const GridMap& esdf_map, const std::vector<std::array<double, 3>>& guides)
        : ompl::base::StateSampler(space), rng_(std::random_device{}()), esdf_map_(esdf_map), guides_(guides)
    {
    }

    virtual void sampleUniform(ompl::base::State* state) override
    {
        // printf("SAMPLING\n");
        // 10% probability to sample from guides
        if (!guides_.empty() && rng_.uniformReal(0.0, 1.0) < 0.1)
        {
            // printf("GUIDE SAMPLING\n");
            int index = rng_.uniformInt(0, guides_.size() - 1);
            auto guide = guides_[index];
            auto* se2_state = state->as<ompl::base::SE2StateSpace::StateType>();

            // Sample x and y around the guide with Gaussian noise
            double sampled_x = rng_.gaussian(guide[0], 0.3);
            double sampled_y = rng_.gaussian(guide[1], 0.3);
            double sampled_yaw = rng_.gaussian(guide[2], 0.4);

            se2_state->setX(sampled_x);
            se2_state->setY(sampled_y);
            se2_state->setYaw(sampled_yaw);
            return;
        }
        double x_min, y_min, x_max, y_max;
        esdf_map_.bounds(x_min, y_min, x_max, y_max);
        auto* se2_state = state->as<ompl::base::SE2StateSpace::StateType>();
        se2_state->setX(rng_.uniformReal(x_min, x_max));
        se2_state->setY(rng_.uniformReal(y_min, y_max));
        se2_state->setYaw(rng_.uniformReal(-M_PI, M_PI));
        return;
    }

    virtual void sampleUniformNear(ompl::base::State* state, const ompl::base::State* near, double distance) override
    {
        const auto* near_state = near->as<ompl::base::SE2StateSpace::StateType>();
        auto* se2_state = state->as<ompl::base::SE2StateSpace::StateType>();
        double angle = rng_.uniformReal(-M_PI, M_PI);
        double r = rng_.uniformReal(0.0, distance);
        double new_x = near_state->getX() + r * cos(angle);
        double new_y = near_state->getY() + r * sin(angle);
        se2_state->setX(new_x);
        se2_state->setY(new_y);
        se2_state->setYaw(rng_.uniformReal(-M_PI, M_PI));
        return;
    }

    virtual void sampleGaussian(ompl::base::State* state, const ompl::base::State* mean, double stdDev) override
    {
        const auto* mean_state = mean->as<ompl::base::SE2StateSpace::StateType>();
        auto* se2_state = state->as<ompl::base::SE2StateSpace::StateType>();
        double new_x = rng_.gaussian(mean_state->getX(), stdDev);
        double new_y = rng_.gaussian(mean_state->getY(), stdDev);
        if (!esdf_map_.isInside(new_x, new_y))
            return;
        se2_state->setX(new_x);
        se2_state->setY(new_y);
        se2_state->setYaw(rng_.uniformReal(-M_PI, M_PI));
        return;
    }

private:
    ompl::RNG rng_;
    const GridMap& esdf_map_;
    const std::vector<std::array<double, 3>>& guides_;
};

class OMPLPlanner
{
public:
    OMPLPlanner();
    ~OMPLPlanner() = default;

    void setup(const GridMap& esdf_map, const std::vector<std::array<double, 3>>& guides);

    void plan(double start_x, double start_y, double start_yaw,
              double goal_x, double goal_y, double goal_yaw,
              std::vector<std::array<double, 3>>& path);

private:
    ompl::base::StateSamplerPtr allocOBMySampler(const ompl::base::StateSpace *space, const GridMap& esdf_map, const std::vector<std::array<double, 3>>& guides_)
    {
        return std::make_shared<MySampler>(space, esdf_map, guides_);
    }

    std::shared_ptr<ompl::geometric::SimpleSetup> simple_setup_;

    std::shared_ptr<ompl::base::OptimizationObjective> optimization_objective_;

    std::shared_ptr<ompl::base::Planner> planner_;

    std::shared_ptr<ompl::base::StateValidityChecker> state_validity_checker_;
};

}
}