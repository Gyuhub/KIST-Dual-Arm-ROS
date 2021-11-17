#pragma once
#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include <iostream>
#include <fstream>
#include <string>
#include <Eigen/Dense>
#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

#include "trajectory.h"
#include "robotmodel.h"
#include "custommath.h"
#include "quadraticprogram.h"

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

//#define DEG2RAD (0.01745329251994329576923690768489)
//#define RAD2DEG 1.0/DEG2RAD

using namespace std;
using namespace Eigen;


class CController
{

public:
	CController(int JDOF);
	virtual ~CController();	
	void read(double time, double* q, double* qdot, double* torque);
	//void write(double* qdes, double* qdotdes);
	void write(double* torque);
	_Float64 _joy_command[19];
	void control_mujoco(_Float64 _joy_command[]);

public:
	VectorXd _torque;
	VectorXd _torque_a;//actuated torque
	VectorXd _q; //joint angle vector
	VectorXd _qdot; //joint velocity vector

private:
	double _t;
	bool _bool_init;
	double _init_t;
	double _dt;
	double _pre_t;
	int _dofj; //number of joint
	int _control_mode; //1: joint space, 2: operational space 
	VectorXd _zero_vec;	
	VectorXd _q_home;

	//VectorXd _q_limit; // TEST(!!!!!!!!!!!!!!!!!!!!!!!!!!)

	VectorXd _pre_q;
	VectorXd _pre_qdot;

	void Initialize();

	//joy stick
	void joystickSub(_Float64 joy_command[]);
	enum
	{
		eJoyBtA = 8,
		eJoyBtB,
		eJoyBtX,
		eJoyBtY,
		eJoyBtLB,
		eJoyBtRB,
		eJoyBtBack,
		eJoyBtStart,
		eJoyBtPower,
		eJoyBtLT = 2,
		eJoyBtRT = 5,
	};
	string _joy_button = "";
	string _joy_pre_button = "";
	bool _bool_joy_button_push;
	VectorXd _bool_joy_plan;
	int _cnt_joy_mode;
	double _LR_stick_left;
	double _UD_stick_left;
	double _LR_stick_right;
	double _UD_stick_right;

	//plan
	void reset_target(double motion_time, VectorXd target_joint_position);
	void reset_target(double motion_time, Vector3d target_pos_lh, Vector3d target_ori_lh, Vector3d target_pos_rh, Vector3d target_ori_rh);
	void reset_target(double motion_time, Vector3d target_pos_lh, Vector3d target_ori_lh, Vector3d target_pos_rh, Vector3d target_ori_rh, string joy_button);
	void motionPlan();
	int _cnt_plan;
	VectorXd _time_plan;
	VectorXi _bool_plan;

	//controller
	double _kpj; //joint control P gain
	double _kdj; //joint control D gain	
	double _kp; //Operational space control P gain
	double _kd; //Operational space control D gain	
	void JointControl();
	void OperationalSpaceControl();
	void OperationalSpaceControlWithoutBodyLink(); //!!!!!!!!!!!!!!!!!!
	void HQPTaskSpaceControl();
	void ReducedHQPTaskSpaceControl();

	//robot model
	CModel Model;
	void ModelUpdate();
	MatrixXd _J_hands; // 12x15
	MatrixXd _J_hands_prism; // 1x15 !!!!!!!!!!!!!!!!!!!!!!!1 
	MatrixXd _J_K_hands; // 12x14
	MatrixXd _J_T_hands; // 15x12
	MatrixXd _J_bar_T_hands;//12x15
	MatrixXd _J_T_hands_prism; // 15x1
	MatrixXd _J_K_T_hands; // 14x12
	MatrixXd _J_Winv_hands; // 15x12
	MatrixXd _J_Winv_T_hands; // 12x15 
	MatrixXd _Jdot_hands;
	MatrixXd _pre_J_hands;
	MatrixXd _pre_Jdot_hands;
	VectorXd _Jdot_qdot;

	MatrixXd _J_pos_hands; // 6x15
	MatrixXd _J_pos_T_hands; // 15x6
	MatrixXd _J_ori_hands; // 6x15
	MatrixXd _J_ori_T_hands; // 15x6	

	//operational space variables (two hand)
	VectorXd _x_left_hand; //state
	VectorXd _x_right_hand; //state
	VectorXd _xdot_left_hand; //state
	VectorXd _xdot_right_hand; //state

	MatrixXd _Lambda_hands; //inertia matrix 12x12
	MatrixXd _Lambda_hands_prism; // inertia matrix 1x1
	MatrixXd _Null_hands; //null space projection matrix 15x15
	MatrixXd _Null_hands_prism; //null space projection matrix 15x15 without prismatic joint //!!!!!!!!!!!!!!!!!!
	MatrixXd _Null_hands_K; // null space projection matrix with selection matrix 14x14
	MatrixXd _S_hands; // selection matrix 14x15
	MatrixXd _S_T_hands; // selection matrix transpose 15x14
	MatrixXd _Id_15, _Id_14, _Id_12;
	VectorXd _xddot_star; //12

	MatrixXd _Lambda_pos_hands; //inertia matri 6x6
	MatrixXd _Lambda_ori_hands; //inertia matri 6x6
	MatrixXd _Null_hands_ori; //null space projection matrix 15x15
	MatrixXd _Null_hands_pos; //null space projection matrix 15x15

	MatrixXd _S_T; //selection matrix transpose
	MatrixXd _J_bar_T_hands_S_T; //
	MatrixXd _W_mat_S;
	MatrixXd _J_tilde_T; //J^T with consideration of selection matrix
	VectorXd _xddot_reinforce; //for additonal feedback acceleration
	VectorXd _torque_reinforce;

	Vector3d _x_err_left_hand;
	Vector3d _x_err_right_hand;
	Vector3d _xdot_err_left_hand;
	Vector3d _xdot_err_right_hand;
	Vector3d _R_err_left_hand;
	Vector3d _R_err_right_hand;
	Vector3d _Rdot_err_left_hand;
	Vector3d _Rdot_err_right_hand;

	//HQP
	CQuadraticProgram HQP_P1; //first priority task - dual hand control
	CQuadraticProgram HQP_P2; //second priority task - joint damping
	MatrixXd _H1, _H2, _A1, _A2;
	VectorXd _g1, _g2, _lbA1, _lbA2, _ubA1, _ubA2, _lb1, _lb2, _ub1, _ub2;

	//Reduced HQP
	CQuadraticProgram rHQP_P1; //first priority task - dual hand control
	CQuadraticProgram rHQP_P2; //second priority task - joint damping
	MatrixXd _rH1, _rH2, _rA1, _rA2;
	VectorXd _rg1, _rg2, _rlbA1, _rlbA2, _rubA1, _rubA2, _rlb1, _rlb2, _rub1, _rub2;
	
	//motion trajectory
	double _start_time, _end_time, _motion_time;

	//joint space
	bool _bool_joint_motion;
	CTrajectory JointTrajectory; //size = joint dof
	VectorXd _q_goal;
	VectorXd _qdot_goal;
	VectorXd _q_des;//desired joint angle vector
	VectorXd _qdot_des;//desired joint velocity vector

	//operational space (two hand)
	bool _bool_ee_motion;
	CTrajectory RightHandTrajectory; //size = 6
	CTrajectory LeftHandTrajectory; //size = 6

	VectorXd _x_goal_left_hand;
	VectorXd _xdot_goal_left_hand;
	VectorXd _x_des_left_hand;
	VectorXd _xdot_des_left_hand;
	VectorXd _x_goal_right_hand;
	VectorXd _xdot_goal_right_hand;
	VectorXd _x_des_right_hand;
	VectorXd _xdot_des_right_hand;
	Matrix3d _R_des_left_hand;
	Matrix3d _R_des_right_hand;

	Vector3d _pos_goal_left_hand;
	Vector3d _rpy_goal_left_hand;
	Vector3d _pos_goal_right_hand;
	Vector3d _rpy_goal_right_hand;

	// Quaternion  ////////////////////////////////////////////////////////////////////////////////////	
	double _ka; //Orientation control a gain
	double _ko; //Orientation control o gain
	Quaterniond _q_left_hand, _pre_q_left_hand;
	Quaterniond _q_right_hand, _pre_q_right_hand;
	Quaterniond _qdot_left_hand, _pre_qdot_left_hand;
	Quaterniond _qdot_right_hand, _pre_qdot_right_hand;
	Quaterniond _q_goal_left_hand;
	Quaterniond _q_goal_right_hand;

	Vector4d _vec_qdot_left_hand; //quaternion dot left hand(w, x, y, z) for arithmetic
	Vector4d _vec_qdot_right_hand; //quaternion dot right hand(w, x, y, z) for arithmetic

	Quaterniond _q_des_left_hand;
	Quaterniond _q_des_right_hand;
	Vector3d _q_des_vec_left_hand;
	Vector3d _q_des_vec_right_hand;
	float _q_des_scalar_left_hand;
	float _q_des_scalar_right_hand;
	Matrix3d _q_des_skew_left_hand;
	Matrix3d _q_des_skew_right_hand;
	Vector3d _q_err_left_hand;
	Vector3d _q_err_right_hand;

	Vector3d _w_left_hand; //angular velocity vector 3x1
	Vector3d _w_right_hand; //angular velocity vector 3x1
	Vector3d _pre_w_left_hand;
	Vector3d _pre_w_right_hand;
	Vector3d _pre_wdot_left_hand;
	Vector3d _pre_wdot_right_hand;
	Vector3d _wdot_left_hand; //angular acceleration vector 3x1
	Vector3d _wdot_right_hand; //angular acceleration vector 3x1
	Vector3d _w_des_left_hand; //desired angular acceleration vector 3x1
	Vector3d _w_des_right_hand; //desired angular acceleration vector 3x1

	Matrix3d _global_rotate; //global rotation matrix for Mujoco Simulator

	///////////////////////////////////////////////////////////////////////////////////////////////////

	// addons  ////////////////////////////////////////////////////////////////////////////////////////	
	bool _bool_safemode;
	void safeModeReplaceTorque(bool bool_safemode);
	void safeWorkSpaceLimit();
	double _dist_shoulder_hand_left;
	double _dist_shoulder_hand_right;
	double _workspace_avoid_gain;
	Vector3d _dir_hand_to_shoulder_left;
	Vector3d _dir_hand_to_shoulder_right;
	Vector3d _acc_workspace_avoid_left;
	Vector3d _acc_workspace_avoid_right;


	////////////////////////////////////////////////////////////////////////////////////////////////////

	// ROS
	VectorXd _joy_vec_command;

	// File stream
	ofstream _ofs_OSF, _ofs_HQP, _ofs_RHQP, _log_sim_torque, _log_com_torque, _log_sim_task;
	int _idx;
};

#endif