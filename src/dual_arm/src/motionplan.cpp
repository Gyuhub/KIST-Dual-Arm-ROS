#include "motionplan.h"

using namespace MjOmpl;

CMotionPlan::CMotionPlan()
{
	initialize();
}

CMotionPlan::~CMotionPlan()
{

}

void CMotionPlan::solve()
{
	setOptimalPlanner();
	_solved = _planner->solve(1.0);
	if (_solved)
	{
		// get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path

		//Path smoothing using bspline
		og::PathSimplifier* pathBSpline = new og::PathSimplifier(_si);
		_path_smooth = new og::PathGeometric(dynamic_cast<const og::PathGeometric&>(*_pdef->getSolutionPath()));
		pathBSpline->smoothBSpline(*_path_smooth,3);

		Eigen::Vector3d result_vec;
		Eigen::Quaterniond result_quat;
		
		_state_count = _path_smooth->getStateCount();
		_result_mat_pos.resize(3, _state_count);
		_result_mat_ori.resize(4, _state_count);
		_result_mat_pos.setZero(3, _state_count);
		_result_mat_ori.setZero(4, _state_count);
		
		for (std::size_t path_idx = 0; path_idx < _state_count; path_idx++)
		{
			const ob::SE3StateSpace::StateType *se3state = _path_smooth->getState(path_idx)->as<ob::SE3StateSpace::StateType>();

            // extract the first component of the state and cast it to what we expect
			const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

            // extract the second component of the state and cast it to what we expect
			const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);
			
			result_vec(0) = pos->values[0];
			result_vec(1) = pos->values[1];
			result_vec(2) = pos->values[2];
			
			result_quat.x() = rot->x;
			result_quat.y() = rot->y;
			result_quat.z() = rot->z;
			result_quat.w() = rot->w;

			_result_mat_pos.block<3,1>(0, path_idx) = result_vec;
			_result_mat_ori.block<4,1>(0, path_idx) = result_quat.coeffs();
		}
		_path_smooth->printAsMatrix(std::cout);// Print geometric Solution as matrix
		_planner->clear();
		_pdef->clearSolutionPaths();
	}
	else
		std::cout << "OPML cannot solve!" << '\n';
}

void CMotionPlan::solveWithoutSmoothing()
{
	setOptimalPlanner();
	_solved = _planner->solve(1.0);
	if (_solved)
	{
		// get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
		ob::PathPtr path = _pdef->getSolutionPath();
		og::PathGeometric* pth = _pdef->getSolutionPath()->as<og::PathGeometric>();

		Eigen::Vector3d result_vec;
		Eigen::Quaterniond result_quat;
		
		_state_count = pth->getStateCount();
		_result_mat_pos.resize(3, _state_count);
		_result_mat_ori.resize(4, _state_count);
		_result_mat_pos.setZero(3, _state_count);
		_result_mat_ori.setZero(4, _state_count);
		
		for (std::size_t path_idx = 0; path_idx < _state_count; path_idx++)
		{
			const ob::SE3StateSpace::StateType *se3state = pth->getState(path_idx)->as<ob::SE3StateSpace::StateType>();

            // extract the first component of the state and cast it to what we expect
			const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

            // extract the second component of the state and cast it to what we expect
			const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);
			
			result_vec(0) = pos->values[0];
			result_vec(1) = pos->values[1];
			result_vec(2) = pos->values[2];
			
			result_quat.x() = rot->x;
			result_quat.y() = rot->y;
			result_quat.z() = rot->z;
			result_quat.w() = rot->w;

			_result_mat_pos.block<3,1>(0, path_idx) = result_vec;
			_result_mat_ori.block<4,1>(0, path_idx) = result_quat.coeffs();
		}
		pth->printAsMatrix(std::cout);// Print geometric Solution as matrix
		_planner->clear();
		_pdef->clearSolutionPaths();
	}
	else
		std::cout << "OPML cannot solve!" << '\n';
}

void CMotionPlan::setOptimalPlanner()
{
	//_planner = std::make_shared<og::RRTstar>(_si);
	//_planner = std::make_shared<og::PRM>(_si);
	_planner = std::make_shared<og::PRMstar>(_si);
	_planner->setProblemDefinition(_pdef);
	_planner->setup();
}

void CMotionPlan::constructStateSpace()
{
	// ompl::base::StateSpacePtr r3(new ompl::base::RealVectorStateSpace(3));
	// ompl::base::StateSpacePtr so3(new ompl::base::SO3StateSpace());
	// _space = r3 + so3;
	ompl::base::StateSpacePtr se3(new ompl::base::SE3StateSpace());
	_space = se3;
}

void CMotionPlan::setBoundsOfState(Eigen::VectorXd low_bounds, Eigen::VectorXd high_bounds)
{
	Eigen::Vector3d low_bounds_position = low_bounds.head(3);
	Eigen::Vector3d high_bounds_position = high_bounds.head(3);
	
	ob::RealVectorBounds bounds(3);

	for (int i = 0; i < 3; i++)
	{
		bounds.setLow(i, low_bounds_position(i));
		bounds.setHigh(i, high_bounds_position(i));
	}

	_space->as<ob::SE3StateSpace>()->setBounds(bounds);
}

void CMotionPlan::setStartState(Eigen::Vector3d start_pos, Eigen::Quaterniond start_ori)
{
	_pdef = std::make_shared<ob::ProblemDefinition>(_si); //*******************************************************************************
	// ob::ScopedState<ob::SE3StateSpace> start__(_space);
	ob::State *start = _space->allocState();
	start->as<ob::SE3StateSpace::StateType>()->setX(start_pos(0));
	start->as<ob::SE3StateSpace::StateType>()->setY(start_pos(1));
	start->as<ob::SE3StateSpace::StateType>()->setZ(start_pos(2));
	// start__->setX(start_state(0));
	// start__->setY(start_state(1));
	// start__->setZ(start_state(2));
	// start__->as<ob::SO3StateSpace::StateType>(1)->x = quat.x();
	// start__->as<ob::SO3StateSpace::StateType>(1)->y = quat.y();
	// start__->as<ob::SO3StateSpace::StateType>(1)->z = quat.z();
	// start__->as<ob::SO3StateSpace::StateType>(1)->w = quat.w();
	start->as<ob::SE3StateSpace::StateType>()->rotation().x = start_ori.x();
	start->as<ob::SE3StateSpace::StateType>()->rotation().y = start_ori.y();
	start->as<ob::SE3StateSpace::StateType>()->rotation().z = start_ori.z();
	start->as<ob::SE3StateSpace::StateType>()->rotation().w = start_ori.w();
	//start->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
	_pdef->addStartState(start); //*******************************************************************************
	_start = _space->allocState();
	_space->copyState(_start, start);
	_space->freeState(start);
	_space->freeState(_start);
}

void CMotionPlan::setGoalState(Eigen::Vector3d goal_pos, Eigen::Quaterniond goal_ori)
{
	// ob::ScopedState<ob::SE3StateSpace> goal(_space);
	ob::State *goal = _space->allocState();
	goal->as<ob::SE3StateSpace::StateType>()->setX(goal_pos(0));
	goal->as<ob::SE3StateSpace::StateType>()->setY(goal_pos(1));
	goal->as<ob::SE3StateSpace::StateType>()->setZ(goal_pos(2));
	// goal->setX(goal_state(0));
	// goal->setY(goal_state(1));
	// goal->setZ(goal_state(2));
	// goal->as<ob::SO3StateSpace::StateType>(1)->x = quat.x();
	// goal->as<ob::SO3StateSpace::StateType>(1)->y = quat.y();
	// goal->as<ob::SO3StateSpace::StateType>(1)->z = quat.z();
	// goal->as<ob::SO3StateSpace::StateType>(1)->w = quat.w();
	goal->as<ob::SE3StateSpace::StateType>()->rotation().x = goal_ori.x();
	goal->as<ob::SE3StateSpace::StateType>()->rotation().y = goal_ori.y();
	goal->as<ob::SE3StateSpace::StateType>()->rotation().z = goal_ori.z();
	goal->as<ob::SE3StateSpace::StateType>()->rotation().w = goal_ori.w();
	//goal->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
	_pdef->setGoalState(goal); // *******************************************************************************
	_goal = _space->allocState();
	_space->copyState(_goal, goal);
	_space->freeState(goal);
	_space->freeState(_goal);
}

void CMotionPlan::constructSpaceInformation()
{
	_si = std::make_shared<ob::SpaceInformation>(_space);
	_si->setStateValidityChecker(ob::StateValidityCheckerPtr(new CStateValidityChecker(_si)));
	_si->setup();
}

void CMotionPlan::constructProblemDefinition()
{
	// _pdef = std::make_shared<ob::ProblemDefinition>(_si);
	//_pdef->setOptimizationObjective(getPathLengthObjective(_si));
	_pdef->setOptimizationObjective(getThresholdPathLengthObj(_si));
}

std::size_t CMotionPlan::getStateCount()
{
	return _state_count;
}

Eigen::MatrixXd CMotionPlan::getResultPosMatrix()
{
	return _result_mat_pos;
}

Eigen::MatrixXd CMotionPlan::getResultOriMatrix()
{
	return _result_mat_ori;
}

void CMotionPlan::getCurrentState(Eigen::Vector3d state_pos, Eigen::Quaterniond state_ori)
{
	ob::ScopedState<ob::SE3StateSpace> st(_space);
	st->setX(state_pos(0));
	st->setY(state_pos(1));
	st->setZ(state_pos(2));
	st->as<ob::SO3StateSpace::StateType>(1)->x = state_ori.x();
	st->as<ob::SO3StateSpace::StateType>(1)->y = state_ori.y();
	st->as<ob::SO3StateSpace::StateType>(1)->z = state_ori.z();
	st->as<ob::SO3StateSpace::StateType>(1)->w = state_ori.w();

	//_scoped_state = st->as<ob::ScopedState>();
//	_scoped_state = st.state_->as<ob::ScopedState>();
}

bool CMotionPlan::isGoalSatisfied(double threshold)
{
	ob::Goal *goal = _pdef->getGoal().get();
}

void CMotionPlan::initialize()
{
	_result_mat_pos.setZero(3, 100);
	_result_mat_ori.setZero(4, 100);
	//ompl::msg::noOutputHandler();
}

// bool CStateValidityChecker::isValid(const ob::State *state) const
// {
//     if(si_->getStateSpace())
//     {
        
//     }
//     else
//     {

//     }
//     return true;
// }