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

	// Quaternion initialize
	_init_quat.vec().setZero();
	_init_quat.w() = 0.0;
	_goal_quat.vec().setZero();
	_goal_quat.w() = 0.0;
	_bool_trajectory_complete = false;
	_bool_trajectory_reset_init = false;
	_bool_trajectory_update_init = false;
	_bool_trajectory_iteration = false;

	// Quaternion cubic spline variables
	_phi = 0.0;
	_psi_skew.setZero();
	_psi_vec.setZero();
	_q_i.vec().setZero();
	_a_i.vec().setZero();
	_b_i.vec().setZero();
	_c_i.vec().setZero();
	_s.vec().setZero();
	_t.vec().setZero();
	_u.vec().setZero();
	_pre_a_i.vec().setZero();
	_pre_b_i.vec().setZero();
	_pre_c_i.vec().setZero();
	_x.vec().setZero();
	_res_quat.vec().setZero();
	_pre_res_quat.vec().setZero();
	_res_quatdot.vec().setZero();
	_pre_res_quatdot.vec().setZero();

	_q_i.w() = 0.0;
	_a_i.w() = 0.0;
	_b_i.w() = 0.0;
	_c_i.w() = 0.0;
	_s.w() = 0.0;
	_t.w() = 0.0;
	_u.w() = 0.0;
	_pre_a_i.w() = 0.0;
	_pre_b_i.w() = 0.0;
	_pre_c_i.w() = 0.0;
	_x.w() = 0.0;
	_res_quat.w() = 0.0;
	_pre_res_quat.w() = 0.0;
	_res_quatdot.w() = 0.0;
	_pre_res_quatdot.w() = 0.0;

	_omega_s.setZero();
	_omega_t.setZero();
	_omega_c_i.setZero();
	_vec_q_i.setZero();
	_vec_a_i.setZero();
	_vec_b_i.setZero();
	_vec_c_i.setZero();
	_vec_s.setZero();
	_vec_t.setZero();
	_vec_u.setZero();
	_vec_pre_a_i.setZero();
	_vec_pre_b_i.setZero();
	_vec_pre_c_i.setZero();
	_vec_x.setZero();
	_vec_res_quatdot.setZero();

	// Quaternion SQUAD variables
	_qd.vec().setZero();
	_qd.w() = 0.0;
	_quat_s1.vec().setZero();
	_quat_s1.w() = 0.0;
	_quat_s2.vec().setZero();
	_quat_s2.w() = 0.0;
	_pre_init_quat.vec().setZero();
	_pre_init_quat.w() = 0.0;
	_pre_goal_quat.vec().setZero();
	_pre_goal_quat.w() = 0.0;

	_init_rotmat.setZero();
	_goal_rotmat.setZero();
	_des_rotmat.setZero();
	_des_quat.vec().setZero();
	_des_quat.w() = 0.0;
}

void CTrajectory::reset_initial(double time0, VectorXd init_pos, VectorXd init_vel)
{
	check_vector_size(init_pos);
	check_vector_size(init_vel);

	_time_start = time0;
	_init_pos = init_pos;
	_init_vel = init_vel;

	_bool_trajectory_complete = false;
}

void CTrajectory::reset_initial(double time0, Quaterniond init_quat, Matrix3d init_rotmat, Vector3d init_angvel, Vector3d init_angacc)
{
	_time_start = time0;
	if (_bool_trajectory_reset_init == false)
	{
		_init_quat = init_quat;
		_pre_init_quat = init_quat;
		_bool_trajectory_reset_init = true;
	}
	else
	{
		_pre_init_quat = _init_quat;
		_init_quat = init_quat;
	}
	_init_rotmat = init_rotmat;
	_init_angvel = init_angvel;
	_init_angacc = init_angacc;

	_bool_trajectory_complete = false;
}

void CTrajectory::update_time(double time)
{
	_time = time;
}

void CTrajectory::update_goal(VectorXd goal_pos, VectorXd goal_vel, double goal_time)
{
	check_vector_size(goal_pos);
	check_vector_size(goal_vel);
	_goal_pos = goal_pos;
	_goal_vel = goal_vel;
	_time_end = goal_time;
}

void CTrajectory::update_goal(Quaterniond goal_quat, Matrix3d goal_rotmat, double goal_time)
{
	_goal_rotmat = goal_rotmat;
	if (_bool_trajectory_update_init == false)
	{
		_goal_quat = goal_quat;
		_pre_goal_quat = goal_quat;
		_phi = 1.0;
		_psi_skew.setZero();
		_q_i.w() = 1.0;
		_psi_vec(0) = _psi_skew(2, 1);
		_psi_vec(1) = _psi_skew(0, 2);
		_psi_vec(2) = _psi_skew(1, 0);
		_q_i.vec().setZero();

		// // case 1. When initial angular velocity and acceleration are equal to zero
		// _c_i.w() = 0.0;
		// _c_i.vec().setZero();
		//
		// Quaterniond temp_quat;
		// temp_quat.w() = 0.0;
		// temp_quat.vec().setZero();
		// Vector4d temp_vec_c_i, temp_vec_b_i, temp_vec;
		// QuatToVec4D(temp_vec_c_i, _c_i);
		// QuatToVec4D(temp_vec_b_i, _b_i);
		// QuatToVec4D(temp_vec, temp_quat);
		// Matrix4d temp_omega_c_i;
		// temp_omega_c_i = CustomMath::MapFromQuaternionTo4DMatrix(_c_i);
		// temp_vec_b_i = (temp_vec - 2.0 * temp_omega_c_i * temp_vec_c_i) / 4.0;
		// Vec4DToQuat(_b_i, temp_vec_b_i);
		//
		// temp_quat.w() = 1.0;
		// temp_quat.vec().setZero();
		// _a_i.w() = _q_i.w() - _b_i.w() - _c_i.w() - temp_quat.w();
		// _a_i.vec() = _q_i.vec() - _b_i.vec() - _c_i.vec() - temp_quat.vec();
		//
		// case 2. When initial angular velocity and acceleration are not equal to zero
		_c_i.w() = 0.0;
		_c_i.vec() = _init_angvel;

		Quaterniond temp_quat;
		temp_quat.w() = 0.0;
		temp_quat.vec() = _init_angacc;
		Vector4d temp_vec_c_i, temp_vec_b_i, temp_vec;
		QuatToVec4D(temp_vec_c_i, _c_i);
		QuatToVec4D(temp_vec_b_i, _b_i);
		QuatToVec4D(temp_vec, temp_quat);
		Matrix4d temp_omega_c_i;
		temp_omega_c_i = CustomMath::MapFromQuaternionTo4DMatrix(_c_i);
		temp_vec_b_i = (temp_vec - 2.0 * temp_omega_c_i * temp_vec_c_i) / 4.0;
		Vec4DToQuat(_b_i, temp_vec_b_i);

		temp_quat.w() = 1.0;
		temp_quat.vec().setZero();
		_a_i.w() = _q_i.w() - _b_i.w() - _c_i.w() - temp_quat.w();
		_a_i.vec() = _q_i.vec() - _b_i.vec() - _c_i.vec() - temp_quat.vec();
		_bool_trajectory_update_init = true;
	}
	else
	{
		_bool_trajectory_iteration = true;
		_pre_goal_quat = _goal_quat;
		_goal_quat = goal_quat;

		_pre_c_i = _c_i;
		_pre_b_i = _b_i;
		_pre_a_i = _a_i;

		_phi = ((_init_rotmat.transpose() * _goal_rotmat).trace() - 1.0) / 2.0;
		_psi_skew = _init_rotmat.transpose() * _goal_rotmat - _goal_rotmat.transpose() * _init_rotmat;
		_q_i.w() = sqrt((1.0 + _phi) / 2.0);
		_psi_vec(0) = _psi_skew(2, 1);
		_psi_vec(1) = _psi_skew(0, 2);
		_psi_vec(2) = _psi_skew(1, 0);
		_q_i.vec() = _psi_vec / _psi_vec.norm() * sqrt((1.0 - _phi) / 2.0);
	}
	_s = _q_i;
	QuatToVec4D(_vec_s, _s);
	QuatToVec4D(_vec_pre_a_i, _pre_a_i);
	QuatToVec4D(_vec_pre_b_i, _pre_b_i);
	QuatToVec4D(_vec_pre_c_i, _pre_c_i);

	_vec_t = 3 * _vec_pre_a_i + 2 * _vec_pre_b_i + _vec_pre_c_i;
	_vec_u = 6 * _vec_pre_a_i + 2 * _vec_pre_b_i;

	Vec4DToQuat(_t, _vec_t);

	_omega_s = CustomMath::MapFromQuaternionTo4DMatrix(_s);
	_omega_t = CustomMath::MapFromQuaternionTo4DMatrix(_t);
	_c_i = _omega_s * _vec_t;
	_omega_c_i = CustomMath::MapFromQuaternionTo4DMatrix(_c_i);
	QuatToVec4D(_vec_c_i, _c_i);

	_vec_b_i = (_omega_t * _vec_t + _omega_s * _vec_u - _omega_c_i * _vec_c_i) / 2.0;

	_vec_a_i(0) = _vec_s(0) - _vec_b_i(0) - _vec_c_i(0) - 1.0;
	_vec_a_i.tail(3) = _vec_s.tail(3) - _vec_b_i.tail(3) - _vec_c_i.tail(3);

	Vec4DToQuat(_c_i, _vec_c_i);
	Vec4DToQuat(_b_i, _vec_b_i);
	Vec4DToQuat(_a_i, _vec_a_i);
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

Quaterniond CTrajectory::quaternion_angular_orientation_cubicSpline()
{
    double tau = (_time - _time_start) / (_time_end - _time_start);
	if (_bool_trajectory_iteration == false)
	{
		_x.w() = _a_i.w() * (tau * tau * tau) + _b_i.w() * (tau * tau) + _c_i.w() * tau + 1.0;
		_x.vec() = _a_i.vec() * (tau * tau * tau) + _b_i.vec() * (tau * tau) + _c_i.vec() * tau;
		QuatToVec4D(_vec_x, _x);
	}
	else
	{
		_vec_x(0) = _vec_a_i(0) * (tau * tau * tau) + _vec_b_i(0) * (tau * tau) + _vec_c_i(0) * tau + 1.0;
		_vec_x.tail(3) = _vec_a_i.tail(3) * (tau * tau * tau) + _vec_b_i.tail(3) * (tau * tau) + _vec_c_i.tail(3) * tau;
	}
	
	Vec4DToQuat(_res_quat, _vec_x);
	Matrix3d res_mat;
	res_mat = _init_rotmat * CustomMath::ConverseFromQuaternionToOGMatrix(_res_quat) / (_vec_x.transpose() * _vec_x);
	return CustomMath::CalcMatrixToQuaternion(res_mat);
}

Vector3d CTrajectory::quaternion_angular_velocity_cubicSpline(double dt)
{
	/*_res_quatdot.w() = CustomMath::VelLowpassFilter(dt, 2.0 * PI * 20.0, _pre_res_quat.w(), _res_quat.w(), _pre_res_quatdot.w());
	
	_pre_res_quat.w() = _res_quat.w();
	_pre_res_quatdot.w() = _res_quatdot.w();

	for (int i = 0; i < 3; i++)
	{
		_res_quatdot.vec().coeffRef(i) = CustomMath::VelLowpassFilter(dt, 2.0 * PI * 20.0, _pre_res_quat.vec().coeffRef(i), _res_quat.vec().coeffRef(i), _pre_res_quatdot.vec().coeffRef(i));
	
		_pre_res_quat.vec().coeffRef(i) = _res_quat.vec().coeffRef(i);
		_pre_res_quatdot.vec().coeffRef(i) = _res_quatdot.vec().coeffRef(i);
	}	*/
	if (_bool_trajectory_iteration == false)
	{
		_pre_res_quat = _res_quat;
	}
	_res_quatdot.w() = (_res_quat.w() - _pre_res_quat.w()) / dt;
	_res_quatdot.vec() = (_res_quat.vec() - _pre_res_quat.vec()) / dt;
	_pre_res_quat = _res_quat;
	_vec_res_quatdot(0) = _res_quatdot.w();
	_vec_res_quatdot.tail(3) = _res_quatdot.vec();

	return (2.0 * CustomMath::CalcAngularVelFromBodyFixedQuaternion(_res_quat) * _vec_res_quatdot);
}

Quaterniond CTrajectory::quaternion_SQUAD()
{
	if (_time <= _time_start)
	{
		_qd = _init_quat;
	}
	else if (_time >= _time_end)
	{
		_qd = _goal_quat;
	}
	else
	{
		// For quaternion spherical quadrangle interpoltation
		// _quat_temp_1 = CustomMath::LogQuaternion(CustomMath::MultiplicationTwoQuaternions(_init_quat.inverse(), _pre_goal_quat));
		// _quat_temp_2 = CustomMath::LogQuaternion(CustomMath::MultiplicationTwoQuaternions(_init_quat.inverse(), _pre_init_quat));
		// _quat_sum.vec() = -(_quat_temp_1.vec() + _quat_temp_2.vec()) / 4.0;
		// _quat_sum.w() = -(_quat_temp_1.w() + _quat_temp_2.w()) / 4.0;
		// _quat_s1 = _init_quat * CustomMath::ExpQuaternion(_quat_sum);

		// _quat_temp_1 = CustomMath::LogQuaternion(CustomMath::MultiplicationTwoQuaternions(_pre_goal_quat.inverse(), _goal_quat));
		// _quat_temp_2 = CustomMath::LogQuaternion(CustomMath::MultiplicationTwoQuaternions(_pre_goal_quat.inverse(), _init_quat));
		// _quat_sum.vec() = -(_quat_temp_1.vec() + _quat_temp_2.vec()) / 4.0;
		// _quat_sum.w() = -(_quat_temp_1.w() + _quat_temp_2.w()) / 4.0;
		// _quat_s2 = _pre_goal_quat * CustomMath::ExpQuaternion(_quat_sum);
		//_qd = (_init_quat.slerp((_time - _time_start) * (_time_end - _time), _goal_quat)).slerp(2 * (_time - _time_start) * (_time_end - _time) * (1 - (_time - _time_start) * (_time_end - _time)), _quat_s1.slerp((_time - _time_start) * (_time_end - _time), _quat_s2));

		_qd = _init_quat.slerp((_time - _time_start)/(_time_end - _time_start), _goal_quat);
	}
	return _qd;
}

template<class T1, class T2>
void CTrajectory::QuatToVec4D(T1& Vec, T2& Quat)
{
	Vec(0) = Quat.w();
	Vec.tail(3) = Quat.vec();
}

template<class T1, class T2>
void CTrajectory::Vec4DToQuat(T1& Quat, T2& Vec)
{
	Quat.w() = Vec(0);
	Quat.vec() = Vec.tail(3);
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