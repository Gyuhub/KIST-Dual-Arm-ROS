#pragma once
#ifndef __MOTIONPLAN_H
#define __MOTIONPLAN_H

// Optimization
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/base/Cost.h>
// Problem definition
#include <ompl/base/ProblemDefinition.h>
// State space
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/State.h>
// Setting goal
#include <ompl/base/Goal.h>
// Choose planner
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "custommath.h"

#define _jdof 15

namespace ob = ompl::base;
namespace og = ompl::geometric;
using namespace ompl;
using namespace Eigen;
namespace MjOmpl
{
    class CMotionPlan
    {
    public:
        CMotionPlan();
        ~CMotionPlan();

        void solve(); // solve problem while smoothing path using bspline
        void solveWithoutSmoothing(); // solve problem without smoothing path using bspline
        std::size_t getStateCount();
        Eigen::MatrixXd getResultPosMatrix();
        Eigen::MatrixXd getResultOriMatrix();

        void constructStateSpace();
        void setBoundsOfState(Eigen::VectorXd low_bounds, Eigen::VectorXd high_bounds);
        void setStartState(Eigen::Vector3d start_pos, Eigen::Quaterniond start_ori);
        void setGoalState(Eigen::Vector3d goal_pos, Eigen::Quaterniond goal_ori);

        void constructSpaceInformation();
        void constructProblemDefinition();
        void constructProblemDefinition(Eigen::VectorXd start_state, Eigen::VectorXd goal_state);

        void getCurrentState(Eigen::Vector3d state_pos, Eigen::Quaterniond state_ori);
        bool isGoalSatisfied(double threshold);

        ob::StateSpacePtr _space;
        ob::SpaceInformationPtr _si;
        ob::ProblemDefinitionPtr _pdef;
        ob::PlannerPtr _planner;

    private:
        void initialize();
        // Returns a structure representing the optimization objective to use
        // for optimal motion planning. This method returns an objective which
        // attempts to minimize the length in configuration space of computed
        // paths.
        inline ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr &si)
        {
            return ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si));
        }
        // Returns a structure representing the optimization objective to use
        // for optimal motion planning. This method returns an objective which
        // attempts to minimize the length in configuration space of computed
        // paths. The objective is satified by the paths of length less than
        // threshold.
        inline ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr &si)
        {
            ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
            obj->setCostToGoHeuristic(&ob::goalRegionCostToGo);
            obj->setCostThreshold(ob::Cost(0.5)); // Threshold : 1.0 [m]
            return obj;
        }
        void setOptimalPlanner();
        ob::PlannerStatus _solved;
        og::PathGeometric* _path_smooth = NULL;
        Eigen::MatrixXd _result_mat_pos, _result_mat_ori;
        std::size_t _state_count;
        ob::State *_start;
        ob::State *_goal;
        ob::State *_state;
    };

    class CStateValidityChecker : public base::StateValidityChecker
    {
    public:
        CStateValidityChecker(const base::SpaceInformationPtr &si) : base::StateValidityChecker(si)
        {
        }
        bool isValid(const ob::State *state) const
        {
            return true; // use when do not consider the obstacle
            //return this->clearance(state) > 0.0;
        }

        double clearance(const ob::State *state) const
        {
            const ob::SE3StateSpace::StateType* se3state =
                state->as<ob::SE3StateSpace::StateType>();
            // const ob::RealVectorStateSpace::StateType* state6D =
            //     state->as<ob::RealVectorStateSpace::StateType>();
            
            // Extract the robot's (x,y,z,r,p,y) position from its state
			const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);
			const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);
            double x = pos->values[0];
            double y = pos->values[1];
            double z = pos->values[2];
            double rot_x = rot->x;
            double rot_y = rot->y;
            double rot_z = rot->z;
            double rot_w = rot->w;
            // double x = state6D->values[0];
            // double y = state6D->values[1];
            // double z = state6D->values[2];
            // double roll = state6D->values[3];
            // double pitch = state6D->values[4];
            // double yaw = state6D->values[5];
            
            // Distance formula between two points, offset by the circle's
            // radius
            return sqrt((x-0.66)*(x-0.66) + (y-0.2)*(y-0.2) + (z-0.66)*(z-0.66)) - 0.1;
        }
    };

    class CClearanceObjective : public ob::StateCostIntegralObjective
    {
    public:
        CClearanceObjective(const ob::SpaceInformationPtr& si) : ob::StateCostIntegralObjective(si, true) {}

        ob::Cost stateCost(const ob::State* s) const
        {
            return ob::Cost(1 / si_->getStateValidityChecker()->clearance(s));
        }
    };

    class CMotionGoal : public ob::Goal
    {
    public:
        CMotionGoal(const ob::SpaceInformationPtr &si) : ob::Goal(si) {}
        virtual bool isSatisfied(const ob::ScopedState<ob::SE3StateSpace> *st) const
        {
        }
    private:
        ob::GoalPtr _goal;
    };
}
#endif