#pragma once
#ifndef __TRAJECTORY_H
#define __TRAJECTORY_H

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "custommath.h"
using namespace std;
using namespace Eigen;

class CTrajectory
{

public:
	CTrajectory();
	virtual ~CTrajectory();
		
	int _vector_size;
	double _time_start, _time, _time_end;	
	VectorXd _init_pos, _init_vel, _goal_pos, _goal_vel;
	Matrix3d _init_rotmat, _goal_rotmat, _des_rotmat;
	Vector3d _init_angvel, _init_angacc;
	Quaterniond _init_quat, _goal_quat, _quat_s1, _quat_s2, _pre_init_quat, _pre_goal_quat, _quat_temp_1, _quat_temp_2, _quat_sum, _qd, _des_quat;

	// Quaternion cubic spline
	double _phi;
	Matrix3d _psi_skew;
	Vector3d _psi_vec;
	Quaterniond _q_i, _a_i, _b_i, _c_i, _s, _t, _u, _pre_a_i, _pre_b_i, _pre_c_i, _x, _res_quat, _pre_res_quat, _res_quatdot, _pre_res_quatdot;
	Matrix4d _omega_s, _omega_t, _omega_c_i;
	Vector4d _vec_q_i, _vec_a_i, _vec_b_i, _vec_c_i, _vec_s, _vec_t, _vec_u, _vec_pre_a_i, _vec_pre_b_i, _vec_pre_c_i, _vec_x, _vec_res_quatdot;

	void set_size(int dof);
	void reset_initial(double time0, VectorXd init_pos, VectorXd init_vel);
	void reset_initial(double time0, Quaterniond init_quat, Matrix3d init_rotmat, Vector3d init_angvel, Vector3d init_angacc);
	void update_time(double time);
	void update_goal(VectorXd goal_pos, VectorXd goal_vel, double goal_time);
	void update_goal(Quaterniond goal_quat, Matrix3d goal_rotmat, double goal_time);
	void calculate_quaternion();
	int check_trajectory_complete();
	VectorXd position_cubicSpline();
	VectorXd velocity_cubicSpline();
	Quaterniond quaternion_angular_orientation_cubicSpline();
	Vector3d quaternion_angular_velocity_cubicSpline(double dt);
	Quaterniond quaternion_SQUAD();
	//CTrajectory operator * (CTrajectory& CT);
	template<class T1, class T2> void QuatToVec4D(T1& Vec, T2& Quat);
	template<class T1, class T2> void Vec4DToQuat(T1& Quat, T2& Vec);
private:
	void Initialize();
	void check_vector_size(VectorXd X);
	bool _bool_trajectory_complete;
	bool _bool_trajectory_reset_init;
	bool _bool_trajectory_update_init;
	bool _bool_trajectory_iteration;
};

#endif