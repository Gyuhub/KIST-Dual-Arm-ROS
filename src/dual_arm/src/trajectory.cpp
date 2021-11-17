#include "trajectory.h"

CTrajectory::CTrajectory()
{	
	Initialize();
}

CTrajectory::~CTrajectory()
{
}

void CTrajectory::set_size(int dof)
{
	_vector_size = dof;
	_init_pos.setZero(_vector_size);
	_init_vel.setZero(_vector_size);
	_goal_pos.setZero(_vector_size);
	_goal_vel.setZero(_vector_size);
}

void CTrajectory::Initialize()
{
	_time_start = 0.0;
	_time = 0.0;
	_time_end = 0.0;
	_vector_size = 1; //default = 1
	_init_pos.setZero(_vector_size);
	_init_vel.setZero(_vector_size);
	_goal_pos.setZero(_vector_size);
	_goal_vel.setZero(_vector_size);

	_bool_trajectory_complete = false;

	/**
	 * Quaternion methods & variables
	 */
	_init_rot_mat.setZero();
	_goal_rot_mat.setZero();
	_start_pos.setZero();
	_start_pos_vel.setZero();
	_start_ori_vel.setZero();
	_end_pos.setZero();
	_end_pos_vel.setZero();
	_end_ori_vel.setZero();

	_bool_pre_processing = false;
	_bool_initialization = false;

	_init_w.setZero();
	_init_alpha.setZero();

	for (int i = 0; i < MAX_SIZE; i++) {
		_R[i].setZero();
		_q[i].setZero();
		_a[i].setZero();
		_b[i].setZero();
		_c[i].setZero();
	}

	_x.setZero();
	_z.setZero();
	_z(3) = 1.0;

	_phi= 0.0;
	_dt = 0.002; // simulate time

	_psi.setZero();
	_wdotd.setZero();

	_i = 1; // i is start from 1
}

// For original cubic spline
// void CTrajectory::reset_initial(double time0, VectorXd init_pos, VectorXd init_vel)
// {
// 	check_vector_size(init_pos);
// 	check_vector_size(init_vel);
//
// 	_time_start = time0;
// 	_init_pos = init_pos;
// 	_init_vel = init_vel;
//
// 	_bool_trajectory_complete = false;
// }

/**
 * Initialize the variables for position and orientation trajectory generation
 */ 
void CTrajectory::reset_initial(double time0, VectorXd init_pos, VectorXd init_vel)
{
	_time_start = time0;
	_init_pos = init_pos;
	_init_vel = init_vel;

	_start_pos = _init_pos.head(3);
	_start_pos_vel = _init_vel.head(3);
	// remove it
	// _start_ori.coeffs() = _init_pos.tail(4).normalized();
	_start_ori_vel = _init_vel.tail(3);

	_bool_trajectory_complete = false;
}

void CTrajectory::update_time(double time)
{
	_time = time;
}

// For original cubic spline
// void CTrajectory::update_goal(VectorXd goal_pos, VectorXd goal_vel, double goal_time)
// {
// 	check_vector_size(goal_pos);
// 	check_vector_size(goal_vel);
//
// 	_goal_pos = goal_pos;
// 	_goal_vel = goal_vel;
//
// 	_time_end = goal_time;
// }

/**
 * Update the variables for position and orientation trajectory generation
 */ 
void CTrajectory::update_goal(VectorXd goal_pos, VectorXd goal_vel, double goal_time)
{
	_goal_pos = goal_pos;
	_goal_vel = goal_vel;

	_end_pos = _goal_pos.head(3);
	_end_pos_vel = _goal_vel.head(3);
	// remove it
	// _end_ori.coeffs() = _goal_pos.tail(4).normalized();
	_end_ori_vel = _goal_vel.tail(3);
	_time_end = goal_time;
}

VectorXd CTrajectory::position_cubicSpline()
{
	VectorXd xd(_vector_size);

	if (_time <= _time_start)
	{
		xd = _init_pos;
	}
	else if (_time >= _time_end)
	{
		xd = _goal_pos;
	}
	else {
		xd = _init_pos + _init_vel * (_time - _time_start)
			+ (3.0 * (_goal_pos - _init_pos) / ((_time_end - _time_start) * (_time_end - _time_start)) - 2.0 * _init_vel / (_time_end - _time_start) - _goal_vel / (_time_end - _time_start)) * (_time - _time_start) * (_time - _time_start)
			+ (-2.0 * (_goal_pos - _init_pos) / ((_time_end - _time_start) * (_time_end - _time_start) * (_time_end - _time_start)) + (_init_vel + _goal_vel) / ((_time_end - _time_start) * (_time_end - _time_start))) * (_time - _time_start) * (_time - _time_start) * (_time - _time_start);
	}
	return xd;
}

/**
 * Only create cubic spline of position, not orientation.
 */
Vector3d CTrajectory::positionCubicSpline()
{
	Vector3d xd;

	if (_time <= _time_start)
	{
		xd = _start_pos;
	}
	else if (_time >= _time_end)
	{
		xd = _end_pos;
	}
	else {
		xd = _start_pos + _start_pos_vel * (_time - _time_start)
			+ (3.0 * (_end_pos - _start_pos) / ((_time_end - _time_start) * (_time_end - _time_start)) - 2.0 * _start_pos_vel / (_time_end - _time_start) - _end_pos_vel / (_time_end - _time_start)) * (_time - _time_start) * (_time - _time_start)
			+ (-2.0 * (_end_pos - _start_pos) / ((_time_end - _time_start) * (_time_end - _time_start) * (_time_end - _time_start)) + (_start_pos_vel + _end_pos_vel) / ((_time_end - _time_start) * (_time_end - _time_start))) * (_time - _time_start) * (_time - _time_start) * (_time - _time_start);
	}
	return xd;
}

Quaterniond CTrajectory::orientationCubicSpline()
{
	if (_time <= _time_start)
	{
		_qd = _R[_i - 1];
	}
	else
	{
		double tau_ = (_time - _time_start) / (_time_end - _time_start);
		_x = _a[_i] * (std::pow(tau_, 3.0)) + _b[_i] * (std::pow(tau_, 2.0)) + _c[_i] * tau_ + _z;
		double x_square = _x.transpose() * _x;
		Matrix3d R_ = _R[_i - 1] * Theta(_x) / x_square;
		_qd = R_;
	}
	return _qd;
}

VectorXd CTrajectory::velocity_cubicSpline()
{
	VectorXd xdotd(_vector_size);

	if (_time <= _time_start)
	{
		xdotd = _init_vel;
	}
	else if (_time >= _time_end)
	{
		xdotd = _goal_vel;
	}
	else {
		xdotd = _init_vel + 2.0 * (3.0 * (_goal_pos - _init_pos) / ((_time_end - _time_start) * (_time_end - _time_start)) - 2.0 * _init_vel / (_time_end - _time_start) - _goal_vel / (_time_end - _time_start)) * (_time - _time_start)
			+ 3.0 * (-2.0 * (_goal_pos - _init_pos) / ((_time_end - _time_start) * (_time_end - _time_start) * (_time_end - _time_start)) + (_init_vel + _goal_vel) / ((_time_end - _time_start) * (_time_end - _time_start))) * (_time - _time_start) * (_time - _time_start);
	}
	return xdotd;
}

/**
 * Only create cubic spline of velocity of position, not velocity of orientation.
 */
Vector3d CTrajectory::velocityCubicSpline()
{
	Vector3d xdotd;

	if (_time <= _time_start)
	{
		xdotd = _start_pos_vel;
	}
	else if (_time >= _time_end)
	{
		xdotd = _end_pos_vel;
	}
	else {
		xdotd = _start_pos_vel + 2.0 * (3.0 * (_end_pos - _start_pos) / ((_time_end - _time_start) * (_time_end - _time_start)) - 2.0 * _start_pos_vel / (_time_end - _time_start) - _end_pos_vel / (_time_end - _time_start)) * (_time - _time_start)
			+ 3.0 * (-2.0 * (_end_pos - _start_pos) / ((_time_end - _time_start) * (_time_end - _time_start) * (_time_end - _time_start)) + (_start_pos_vel + _end_pos_vel) / ((_time_end - _time_start) * (_time_end - _time_start))) * (_time - _time_start) * (_time - _time_start);
	}
	return xdotd;
}

Vector3d CTrajectory::orientationVelocityCubicSpline(Vector3d omega)
{
	if (_time <= _time_start)
	{
		_wdotd = _start_ori_vel;
	}
	else if (_time >= _time_end)
	{
		_wdotd = _end_ori_vel;
	}
	else {
		AngleAxisd aa_d_, aa_;
		aa_d_ = _qd;
		aa_ = q.normalized();
		double phi_d_ = aa_d_.angle();
		double phi_ = aa_.angle();
		Vector3d axis_ = aa_d_.axis();
		Vector4d q_;
		q_(0) = std::cos(phi_d_ / 2.0);
		q_(1) = axis_(0) * std::sin(phi_d_ / 2.0);
		q_(2) = axis_(1) * std::sin(phi_d_ / 2.0);
		q_(3) = axis_(2) * std::sin(phi_d_ / 2.0);
		// std::complex<double> i_(1 ,1);
		// Matrix2cd M_;
		// M_ << q_(0) + q_(1)*i_ , q_(2) + q_(3)*i_
		// 	,-q_(2) + q_(3)*i_ , q_(0) - q_(1)*i_;
		MatrixXd M_inv;
		M_inv.setZero(3, 4);
		M_inv << -q_(1) , q_(0) , q_(3) , -q_(2)
				,-q_(2) ,-q_(3) , q_(0) ,  q_(1)
				,-q_(3) , q_(2) ,-q_(1) ,  q_(0);
		Vector4d qdot_;
		double phi_dot_ = (phi_d_ - phi_) / _dt; // if dt is too small, it's value effects to phi_dot too much when we calculate phi_dot. so we do multiply dt to phi_dot.
		qdot_(0) = -std::sin(phi_d_ / 2.0) * phi_dot_;
		qdot_(1) = axis_(0) * std::cos(phi_d_) * phi_dot_;
		qdot_(2) = axis_(1) * std::cos(phi_d_) * phi_dot_;
		qdot_(3) = axis_(2) * std::cos(phi_d_) * phi_dot_;
		_wdotd = 2 * M_inv * qdot_;
		


		// AngleAxisd aa_des_, aa_;
		// aa_des_ = _qd;
		// aa_ = q;
		// Matrix3d R_des_ = aa_des_.toRotationMatrix();
		// Matrix3d R_ = aa_.toRotationMatrix();
		// _wdotd = -CustomMath::getPhi(R_, R_des_);
		return _wdotd;
	}
}

void CTrajectory::check_vector_size(VectorXd X)
{
	if (X.size() == _vector_size)
	{

	}
	else
	{
		cout << "Warning!!! -- Vector size in CTrajectory mismatch occured! --" << endl << endl;
	}
}

void CTrajectory::getInitialRotationMatrix(Matrix3d R)
{
	_R[0] = R;
}

void CTrajectory::getRotationMatrices(MatrixXd R, int state_size)
{
	for (int i = 1; i <= state_size; i++)
	{
		Vector4d quat_vec_ = R.block<4, 1>(0, i-1);
		Quaterniond quat_;
		quat_.coeffs() = quat_vec_;
		// _R[i] = _R[i - 1] * quat_.normalized().toRotationMatrix();
		_R[i] = quat_.normalized().toRotationMatrix();
	}
}

void CTrajectory::preProcessing(int state_size)
{
	if (_bool_pre_processing == false)
	{
		for (int i = 1; i <= state_size; i++)
		{
			Matrix3d skew_rot_mat_;
			skew_rot_mat_.setZero();
			skew_rot_mat_ = (_R[i - 1].transpose() * _R[i]);
			if (((skew_rot_mat_.transpose() + skew_rot_mat_) / 2.0).isIdentity())
			{
				_phi = 0.0;
				_q[i].head(3) = Eigen::Vector3d::Zero(3);
				_q[i](3) = std::sqrt(1.0 / 2.0);
			}
			else
			{
				_phi = (((_R[i - 1].transpose() * _R[i]).trace()) - 1.0) / 2.0;
				skew_rot_mat_ = ((_R[i - 1].transpose() * _R[i]) - (_R[i].transpose() * _R[i - 1]));
				_psi(0) = skew_rot_mat_(2, 1);
				_psi(1) = skew_rot_mat_(0, 2);
				_psi(2) = skew_rot_mat_(1, 0);
				_q[i].head(3) = _psi * (1.0 / _psi.norm()) * std::sqrt(((1.0 - _phi) / 2.0));
				_q[i](3) = std::sqrt(((1.0 + _phi) / 2.0));
			}
			// _q[i].normalize();
		}
		_bool_pre_processing = true;
	}
	else return;
}

void CTrajectory::initialization()
{
	if (_bool_initialization == false)
	{
		for (int i = 0; i < 3; i++) _c[1](i) = _init_w(i) / 2.0;
		_c[1](3) = 0.0;
		Eigen::Vector4d omega_cc_ = Omega(_c[1]) * _c[1];
		for (int i = 0; i < 3; i++) _b[1](i) = (_init_alpha(i) - 2.0 * omega_cc_(i)) / 4.0;
		_b[1](3) = (omega_cc_(3)) * -2.0 / 4.0;
		_a[1] = _q[1] - _b[1] - _c[1] - _z;
		_bool_initialization = true;
	}
	else return;
}

void CTrajectory::iterate(int state_size)
{
	for (int i = 2; i <= state_size; i++)
	{
		Vector4d s_, t_, u_; s_.setZero(); t_.setZero(); u_.setZero();
		s_ = _q[i];
		t_ = 3 * _a[i - 1] + 2 * _b[i - 1] + _c[i - 1];
		u_ = 6 * _a[i - 1] + 2 * _b[i - 1];
		_c[i] = Omega(s_) * t_;
        _b[i] = (Omega(t_) * t_ + Omega(s_) * u_ - Omega(_c[i]) * _c[i]) / 2.0;
        _a[i] = s_ - _b[i] - _c[i] - _z;
	}
}

int CTrajectory::check_trajectory_complete() //1 = time when trajectory complete
{
	int diff = 0;
	bool previous_bool = _bool_trajectory_complete;
	if (_time >= _time_end && _bool_trajectory_complete == false)
	{
		_bool_trajectory_complete = true;
		diff = 1;
	}

	return diff;
}

Matrix4d CTrajectory::Omega(Vector4d x)
{
	Matrix4d omega_;
	omega_ << x(3)  ,  x(2), -x(1), -x(0)
        	, -x(2) ,  x(3),  x(0), -x(1)
        	,  x(1) , -x(0),  x(3), -x(2)
        	,  x(0) ,  x(1),  x(2),  x(3);
	return omega_;
}

Matrix3d CTrajectory::Theta(Vector4d x)
{
	Matrix3d theta_;
	theta_ << (x(3)*x(3) + x(0)*x(0) - x(1)*x(1) - x(2)*x(2)) , 2.0 * (x(0)*x(1) - x(2)*x(3))                    , 2.0 * (x(0)*x(2) + x(1)*x(3))
         	, 2.0 * (x(0)*x(1) + x(2)*x(3))                   , (x(3)*x(3) - x(0)*x(0) + x(1)*x(1) - x(2)*x(2) ) , 2.0 * (x(1)*x(2) - x(0)*x(3))
         	, 2.0 * (x(0)*x(2) - x(1)*x(3))                   , 2.0 * (x(1)*x(2) + x(0)*x(3))                    , (x(3)*x(3) - x(0)*x(0) - x(1)*x(1) + x(2)*x(2) );
	return theta_;
}

void CTrajectory::increaseStep()
{
	_i++;
	return;
}