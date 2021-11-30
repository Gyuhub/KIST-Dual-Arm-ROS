#pragma once
#ifndef __TRAJECTORY_H
#define __TRAJECTORY_H

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <complex>
#include "custommath.h"
using namespace std;
using namespace Eigen;

#define MAX_SIZE 100 // maximum size of rotation matrices

class CTrajectory
{

public:
	CTrajectory();
	virtual ~CTrajectory();
		
	int _vector_size;
	double _time_start, _time, _time_end;	
	VectorXd _init_pos, _init_vel, _goal_pos, _goal_vel;

	Matrix3d _init_rot_mat, _goal_rot_mat;
	Vector3d _start_pos, _start_pos_vel, _end_pos, _end_pos_vel, _start_ori_vel, _end_ori_vel;
	Quaterniond _start_ori, _end_ori;

	void set_size(int dof);
	void reset_initial(double time0, VectorXd init_pos, VectorXd init_vel);
	void update_time(double time);
	void update_goal(VectorXd goal_pos, VectorXd goal_vel, double goal_time);
	int check_trajectory_complete();
	void getInitialRotationMatrix(Matrix3d R);
	void getRotationMatrices(MatrixXd R, int state_size); // get rotation matrices from motion planning. state size is the number of the states that obtained from motion planning
	void preProcessing(int state_size); // Preprocessing for rotate matrix
	void initialization(); // Initialization for quaternion interpolation using cubic spline
	void iterate(int state_size); // Calculate the coefficients of cubic polynomial problem
	void increaseStep(); // Increase the step of the orientation cubic spline
	VectorXd position_cubicSpline();
	VectorXd velocity_cubicSpline();
	Vector3d positionCubicSpline();
	Vector3d velocityCubicSpline();
	Quaterniond orientationCubicSpline();
	Vector3d orientationVelocityCubicSpline(Matrix3d R_des); //(Vector3d omegaMatrix3d R) R is rotation matrix from base to body
private:
	void Initialize();
	void check_vector_size(VectorXd X);
	Matrix4d Omega(Vector4d x);
	Matrix3d Theta(Vector4d x);
	bool _bool_trajectory_complete;
	bool _bool_pre_processing, _bool_initialization;
	bool _bool_rotation_hold; // this flag is set when previous rotation matrix and current rotation matrix are same
	double _phi;
	double _dt;
	Matrix3d _R[MAX_SIZE]; // Rotation matrices that contain the rotation matrix of each step from i = 0 to n
	Vector4d _a[MAX_SIZE], _b[MAX_SIZE], _c[MAX_SIZE]; // constant variables for calculating the interpolating trajectory
	Vector4d _x, _z; // x is the result of cubic polynomial and z is the transformation of quaternion which is obtained from identity matrix
	Vector3d _init_w, _init_alpha, _psi, _wdotd;
	Vector4d _q[MAX_SIZE]; // Vectors that contain the quaternion of each step from i = 1 to n
	Quaterniond _qd, _qd_pre;
	int _i; // step of the orientation cubic spline which is counted from 0 to n. n is the index of final rotation matrix
};

#endif