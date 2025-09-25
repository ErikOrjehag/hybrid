
#include "include/ompl_planner.hpp"
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

namespace dyno {
namespace ompl_planner {

OMPLPlanner::OMPLPlanner()
{
      // Create a car state space ptr
    ompl::base::StateSpacePtr car_state_space = std::make_shared<ompl::base::DubinsStateSpace>(1.0);

    simple_setup_ = std::make_shared<ompl::geometric::SimpleSetup>(car_state_space);

    optimization_objective_ =
        std::make_shared<ompl::base::PathLengthOptimizationObjective>(
            simple_setup_->getSpaceInformation());

    simple_setup_->setOptimizationObjective(optimization_objective_);

    planner_ = std::make_shared<ompl::geometric::RRTstar>(
        simple_setup_->getSpaceInformation());
    
    planner_->as<ompl::geometric::RRTstar>()->setAdmissibleCostToCome(true);
    planner_->as<ompl::geometric::RRTstar>()->setInformedSampling(false);
    // planner_->as<ompl::geometric::RRTstar>()->setTreePruning(true);
    // planner_->as<ompl::geometric::RRTstar>()->setPrunedMeasure(true);

    simple_setup_->setPlanner(planner_);
}

void OMPLPlanner::setup(const GridMap& esdf_map, const std::vector<std::array<double, 3>>& guides)
{
    double xmin, ymin, xmax, ymax;
    esdf_map.bounds(xmin, ymin, xmax, ymax);

    ompl::base::RealVectorBounds bounds(2);
    bounds.low[0] = xmin;
    bounds.high[0] = xmax;
    bounds.low[1] = ymin;
    bounds.high[1] = ymax;
    simple_setup_->getStateSpace()->as<ompl::base::SE2StateSpace>()->setBounds(bounds);

    simple_setup_->getStateSpace()->setLongestValidSegmentFraction(
        esdf_map.resolution() / simple_setup_->getStateSpace()->getMaximumExtent());

    state_validity_checker_ = std::make_shared<MyStateValidityChecker>(
        simple_setup_->getSpaceInformation(), esdf_map);

    simple_setup_->setStateValidityChecker(state_validity_checker_);

    simple_setup_->getSpaceInformation()->setValidStateSamplerAllocator(
        std::bind(&OMPLPlanner::allocOBMySampler, this, std::placeholders::_1, esdf_map, guides));
}


void OMPLPlanner::plan(double start_x, double start_y, double start_yaw,
            double goal_x, double goal_y, double goal_yaw,
            std::vector<std::array<double, 3>>& path)
{   
    planner_->clear();
    ompl::base::ScopedState<ompl::base::SE2StateSpace> start(simple_setup_->getStateSpace());
    start->setXY(start_x, start_y);
    start->setYaw(start_yaw);
    ompl::base::ScopedState<ompl::base::SE2StateSpace> goal(simple_setup_->getStateSpace());
    goal->setXY(goal_x, goal_y);
    goal->setYaw(goal_yaw);
    simple_setup_->setStartAndGoalStates(start, goal);

    ompl::base::PlannerStatus status =
      simple_setup_->solve(10.0);
    
    if (status == ompl::base::PlannerStatus::EXACT_SOLUTION ||
        status == ompl::base::PlannerStatus::APPROXIMATE_SOLUTION)
    {
        ompl::geometric::PathGeometric path_geometric = simple_setup_->getSolutionPath();
        path_geometric.interpolate(path_geometric.length() / 0.1); // interpolate to 0.1m
        path.clear();
        for (size_t i = 0; i < path_geometric.getStateCount(); ++i)
        {
            const auto* state = path_geometric.getState(i)->as<ompl::base::SE2StateSpace::StateType>();
            double x = state->getX();
            double y = state->getY();
            double yaw = state->getYaw();
            path.push_back({x, y, yaw});
        }
    }
    else
    {
        path.clear();
    }
}
} // namespace ompl_planner
} // namespace dyno