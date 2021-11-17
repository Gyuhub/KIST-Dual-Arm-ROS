#include "controller.h"
#include <chrono>

CController::CController(int JDOF)
{
	_dofj = JDOF;
	Initialize();
}

CController::~CController()
{
	_log_sim_torque.close();
	_log_com_torque.close();
	_log_sim_task.close();
}

void CController::read(double t, double* q, double* qdot, double* torque)
{	
	_t = t;

	/////////////////check1
	if (_bool_init == true)
	{
		_init_t = _t;
		_bool_init = false;
	}

	_dt = t - _pre_t;
	_pre_t = t;

	for (int i = 0; i < _dofj; i++)
	{
		_q(i) = q[i];
		//_qdot(i) = qdot[i]; //from simulator
		_qdot(i) = CustomMath::VelLowpassFilter(_dt, 2.0*PI* 10.0, _pre_q(i), _q(i), _pre_qdot(i)); //low-pass filter
		
		_pre_q(i) = _q(i);
		_pre_qdot(i) = _qdot(i);
		//NOTE: log the simulation joint torque and desired joint torque datas for plot
		_log_sim_torque << to_string(torque[i]) << "\t";
	}
	_log_sim_torque << endl;
	_log_sim_task << _x_left_hand(0) << "\t" << _x_left_hand(1) << "\t" << _x_left_hand(2) << "\t" << _x_des_left_hand(0) << "\t" << _x_des_left_hand(1) << "\t" << _x_des_left_hand(2) << "\t" << _x_right_hand(0) << "\t" << _x_right_hand(1) << "\t" << _x_right_hand(2) << "\t" << _x_des_right_hand(0) << "\t" << _x_des_right_hand(1) << "\t" << _x_des_right_hand(2) << "\n";
}

void CController::write(double* torque)
{
	for (int i = 0; i < _dofj; i++)
	{
		torque[i] = _torque(i);
		_log_com_torque << to_string(torque[i]) << "\t";
	}
	torque[0] = 0;
	_log_com_torque << endl;
}

void CController::reset_target(double motion_time, VectorXd target_joint_position)
{
	_control_mode = 1;
	_motion_time = motion_time;
	_bool_joint_motion = false;
	_bool_ee_motion = false;

	_q_goal = target_joint_position.head(15);
	_qdot_goal.setZero();
}

void CController::reset_target(double motion_time, Vector3d target_pos_lh, Vector3d target_ori_lh, Vector3d target_pos_rh, Vector3d target_ori_rh)
{
	_control_mode = 4;
	_motion_time = motion_time;
	_bool_joint_motion = false;
	_bool_ee_motion = false;

	_x_goal_left_hand.head(3) = target_pos_lh;
	_x_goal_left_hand.tail(3) = target_ori_lh;
	_xdot_goal_left_hand.setZero();
	_x_goal_right_hand.head(3) = target_pos_rh;
	_x_goal_right_hand.tail(3) = target_ori_rh;
	_xdot_goal_right_hand.setZero();
	// cout << "-----------------------------" << endl;
	// cout << "left hands target orientation " << endl << _x_goal_left_hand.tail(3) * RAD2DEG << endl;
	// cout << "right hands target orientation " << endl << _x_goal_right_hand.tail(3) * RAD2DEG << endl;
	_q_goal_left_hand = CustomMath::CalcVectorToQuaternion(target_ori_lh);
	_q_goal_right_hand = CustomMath::CalcVectorToQuaternion(target_ori_rh);
}

void CController::reset_target(double motion_time, Vector3d target_pos_lh, Vector3d target_ori_lh, Vector3d target_pos_rh, Vector3d target_ori_rh, string joy_button)
{
	_motion_time = motion_time;
	_bool_joint_motion = false;
	_bool_ee_motion = false;

	_x_goal_left_hand.head(3) = target_pos_lh;
	_x_goal_left_hand.tail(3) = target_ori_lh;
	_xdot_goal_left_hand.setZero();
	_x_goal_right_hand.head(3) = target_pos_rh;
	_x_goal_right_hand.tail(3) = target_ori_rh;
	_xdot_goal_right_hand.setZero();
}

void CController::joystickSub(_Float64 joy_command[])
{
	// Table of index number of /joy_command:
	// Index
	// Axis name on the actual controller
	// 0	 Left/Right Axis stick left
	// 1	 Up/Down Axis stick left
	// 2	 LT
	// 3	 Left/Right Axis stick right
	// 4	 Up/Down Axis stick right
	// 5	 RT
	// 6	 cross key left/right
	// 7	 cross key up/down
	//-------------------------------------
	// Button name on the actual controller
	// 8 	 A
	// 9 	 B
	// 10 	 X
	// 11 	 Y
	// 12 	 LB
	// 13	 RB
	// 14	 back
	// 15	 start
	// 16	 power
	// 17	 Button stick left
	// 18	 Button stick right
	
	// Button control
	// int _joy_count = 0;
	// for (int i = 8; i < 16; i++)
	// {
	// 	if (joy_command[i] == 1)
	// 	{
	// 		switch (i)
	// 		{
	// 		case eJoyBtA:
	// 			_joy_button = "A";	break;
	// 		case eJoyBtB:
	// 			_joy_button = "B";	break;
	// 		case eJoyBtX:
	// 			_joy_button = "X";	break;
	// 		case eJoyBtY:
	// 			_joy_button = "Y";	break;
	// 		case eJoyBtStart:
	// 			_joy_button = "Start"; break;
	// 		}
	// 		//cout << "button : " << _joy_button << endl;
	// 		if (_joy_pre_button != _joy_button)
	// 		{
	// 			_bool_joy_button_push = true;
	// 			if (_bool_joy_plan(_cnt_plan) == 1)
	// 			{
	// 				_init_t = _t;
	// 			}
	// 			cout << "button change flag" << endl; 
	// 			// cout << "pre : " << _joy_pre_button << " , now : " << _joy_button << endl;
	// 		}
	// 		_joy_pre_button = _joy_button;
	// 	}
	// 	else
	// 	{
	// 		_joy_count++;
	// 	}
	// }
	// if (_joy_count == 8)
	// {
	// 	_joy_button = "NA";
	// }
	// Axis Stick control
	// _LR_stick_left = joy_command[0] * 0.5;
	// _UD_stick_left = joy_command[1] * 0.5;
	// _LR_stick_right = joy_command[3] * 0.5;
	// _UD_stick_right = joy_command[4] * 0.5;
}

void CController::motionPlan()
{
	_time_plan(1) = 5.0; //move task position
	_time_plan(2) = 5.0; //taks space motion 1
	_time_plan(3) = 5.0; //task space motion 2
	_time_plan(4) = 5.0; //task space motion 3
	_time_plan(5) = 5.0; //move to another task position
	_time_plan(6) = 5.0; //task space motion 4
	_time_plan(7) = 5.0; //task space motion 5
	_time_plan(8) = 1000000.0; // move home position
	

	// Motion plan holding object
	if (_bool_plan(_cnt_plan) == 1)
	{
		_cnt_plan = _cnt_plan + 1;
		cout << "//////////  motion plan " << _cnt_plan  << " start.  //////////\n" << endl;
		if (_cnt_plan == 1)
		{
			_pos_goal_left_hand(0) = _x_left_hand(0) + 0.1;
		 	_pos_goal_left_hand(1) = _x_left_hand(1);
		 	_pos_goal_left_hand(2) = _x_left_hand(2);
		 	_rpy_goal_left_hand(0) = _x_left_hand(3);//180.0 * DEG2RAD;
			_rpy_goal_left_hand(1) = _x_left_hand(4);//
			_rpy_goal_left_hand(2) = _x_left_hand(5);//-90.0 * DEG2RAD;
	
			_pos_goal_right_hand(0) = _x_right_hand(0) + 0.1;
		 	_pos_goal_right_hand(1) = _x_right_hand(1);
		 	_pos_goal_right_hand(2) = _x_right_hand(2);
		 	_rpy_goal_right_hand(0) = _x_right_hand(3);
			_rpy_goal_right_hand(1) = _x_right_hand(4);
			_rpy_goal_right_hand(2) = _x_right_hand(5);//90.0 * DEG2RAD;
			reset_target(_time_plan(_cnt_plan), _pos_goal_left_hand, _rpy_goal_left_hand, _pos_goal_right_hand, _rpy_goal_right_hand);
		}
		if (_cnt_plan == 2)
		{
			_pos_goal_left_hand(0) = _x_left_hand(0);
		 	_pos_goal_left_hand(1) = _x_left_hand(1);
		 	_pos_goal_left_hand(2) = _x_left_hand(2) + 0.1;
		 	_rpy_goal_left_hand(0) = _x_left_hand(3);
			_rpy_goal_left_hand(1) = _x_left_hand(4);
			_rpy_goal_left_hand(2) = _x_left_hand(5);
	
			_pos_goal_right_hand(0) = _x_right_hand(0);
		 	_pos_goal_right_hand(1) = _x_right_hand(1);
		 	_pos_goal_right_hand(2) = _x_right_hand(2) + 0.1;
		 	_rpy_goal_right_hand(0) = _x_right_hand(3);
			_rpy_goal_right_hand(1) = _x_right_hand(4);
			_rpy_goal_right_hand(2) = _x_right_hand(5);
			reset_target(_time_plan(_cnt_plan), _pos_goal_left_hand, _rpy_goal_left_hand, _pos_goal_right_hand, _rpy_goal_right_hand);
		}
		if (_cnt_plan == 3)
		{
			_pos_goal_left_hand(0) = _x_left_hand(0);
		 	_pos_goal_left_hand(1) = _x_left_hand(1) + 0.1;
		 	_pos_goal_left_hand(2) = _x_left_hand(2);
		 	_rpy_goal_left_hand(0) = _x_left_hand(3);//0.0 * DEG2RAD;
			_rpy_goal_left_hand(1) = _x_left_hand(4);//40.0 * DEG2RAD;
			_rpy_goal_left_hand(2) = _x_left_hand(5);//0.0 * DEG2RAD;
	
			_pos_goal_right_hand(0) = _x_right_hand(0);
		 	_pos_goal_right_hand(1) = _x_right_hand(1) - 0.1;
		 	_pos_goal_right_hand(2) = _x_right_hand(2);
		 	_rpy_goal_right_hand(0) = _x_right_hand(3);//180.0 * DEG2RAD;
			_rpy_goal_right_hand(1) = _x_right_hand(4);//40.0 * DEG2RAD;
			_rpy_goal_right_hand(2) = _x_right_hand(5);//0.0 * DEG2RAD;
			reset_target(_time_plan(_cnt_plan), _pos_goal_left_hand, _rpy_goal_left_hand, _pos_goal_right_hand, _rpy_goal_right_hand);
		}
		if (_cnt_plan == 4)
		{
			_pos_goal_left_hand(0) = _x_left_hand(0);
		 	_pos_goal_left_hand(1) = _x_left_hand(1);
		 	_pos_goal_left_hand(2) = _x_left_hand(2) - 0.1;
		 	_rpy_goal_left_hand(0) = _x_left_hand(3);//0.0 * DEG2RAD;
			_rpy_goal_left_hand(1) = _x_left_hand(4);//40.0 * DEG2RAD;
			_rpy_goal_left_hand(2) = _x_left_hand(5);//0.0 * DEG2RAD;
	
			_pos_goal_right_hand(0) = _x_right_hand(0);
		 	_pos_goal_right_hand(1) = _x_right_hand(1);
		 	_pos_goal_right_hand(2) = _x_right_hand(2) - 0.1;
		 	_rpy_goal_right_hand(0) = _x_right_hand(3);//180.0 * DEG2RAD;
			_rpy_goal_right_hand(1) = _x_right_hand(4);//40.0 * DEG2RAD;
			_rpy_goal_right_hand(2) = _x_right_hand(5);//0.0 * DEG2RAD;
			reset_target(_time_plan(_cnt_plan), _pos_goal_left_hand, _rpy_goal_left_hand, _pos_goal_right_hand, _rpy_goal_right_hand);
		}
		if (_cnt_plan == 5)
		{
			_pos_goal_left_hand(0) = _x_left_hand(0);
		 	_pos_goal_left_hand(1) = _x_left_hand(1) - 0.1;
		 	_pos_goal_left_hand(2) = _x_left_hand(2);
		 	_rpy_goal_left_hand(0) = _x_left_hand(3);//0.0 * DEG2RAD;
			_rpy_goal_left_hand(1) = _x_left_hand(4);//40.0 * DEG2RAD;
			_rpy_goal_left_hand(2) = _x_left_hand(5);//0.0 * DEG2RAD;
	
			_pos_goal_right_hand(0) = _x_right_hand(0);
		 	_pos_goal_right_hand(1) = _x_right_hand(1) + 0.1;
		 	_pos_goal_right_hand(2) = _x_right_hand(2);
		 	_rpy_goal_right_hand(0) = _x_right_hand(3);//180.0 * DEG2RAD;
			_rpy_goal_right_hand(1) = _x_right_hand(4);//40.0 * DEG2RAD;
			_rpy_goal_right_hand(2) = _x_right_hand(5);//0.0 * DEG2RAD;
			reset_target(_time_plan(_cnt_plan), _pos_goal_left_hand, _rpy_goal_left_hand, _pos_goal_right_hand, _rpy_goal_right_hand);
		}
		if (_cnt_plan == 6)
		{
			_pos_goal_left_hand(0) = _x_left_hand(0) + 0.05;
		 	_pos_goal_left_hand(1) = _x_left_hand(1) + 0.05;
		 	_pos_goal_left_hand(2) = _x_left_hand(2) + 0.05;
		 	_rpy_goal_left_hand(0) = _x_left_hand(3);//0.0 * DEG2RAD;
			_rpy_goal_left_hand(1) = _x_left_hand(4);//40.0 * DEG2RAD;
			_rpy_goal_left_hand(2) = _x_left_hand(5);//0.0 * DEG2RAD;
	
			_pos_goal_right_hand(0) = _x_right_hand(0) + 0.05;
		 	_pos_goal_right_hand(1) = _x_right_hand(1) - 0.05;
		 	_pos_goal_right_hand(2) = _x_right_hand(2) + 0.05;
		 	_rpy_goal_right_hand(0) = _x_right_hand(3);//180.0 * DEG2RAD;
			_rpy_goal_right_hand(1) = _x_right_hand(4);//40.0 * DEG2RAD;
			_rpy_goal_right_hand(2) = _x_right_hand(5);//0.0 * DEG2RAD;
			reset_target(_time_plan(_cnt_plan), _pos_goal_left_hand, _rpy_goal_left_hand, _pos_goal_right_hand, _rpy_goal_right_hand);
		}
		if (_cnt_plan == 7)
		{
			reset_target(_time_plan(_cnt_plan), _q_home);
		}
		if (_cnt_plan == 8)
		{
			reset_target(_time_plan(_cnt_plan), _q);
			
		}

		// if (_cnt_plan == 1) //move task position
		// {
		// 	_pos_goal_left_hand(0) = 0.7;
		//  	_pos_goal_left_hand(1) = 0.2 + 0.075;
		//  	_pos_goal_left_hand(2) = 0.52;
		//  	_rpy_goal_left_hand(0) = 0.0 * DEG2RAD;
		// 	_rpy_goal_left_hand(1) = -90.0 * DEG2RAD;
		// 	_rpy_goal_left_hand(2) = 0.0 * DEG2RAD;
		//
		// 	_pos_goal_right_hand(0) = 0.7;
		//  	_pos_goal_right_hand(1) = 0.2 - 0.075;
		//  	_pos_goal_right_hand(2) = 0.52;
		//  	_rpy_goal_right_hand(0) = 0.0 * DEG2RAD;
		// 	_rpy_goal_right_hand(1) = -90.0 * DEG2RAD;
		// 	_rpy_goal_right_hand(2) = 180.0 * DEG2RAD;
		// 	reset_target(_time_plan(_cnt_plan), _pos_goal_left_hand, _rpy_goal_left_hand, _pos_goal_right_hand, _rpy_goal_right_hand);
		// 	cout << "motion plan " << _cnt_plan  << " start. move task position" << endl;
		// }
		// else if (_cnt_plan == 2) //task space motion 1
		// {
		// 	_pos_goal_left_hand(0) = _x_left_hand(0);
		//  	_pos_goal_left_hand(1) = _x_left_hand(1);
		//  	_pos_goal_left_hand(2) = _x_left_hand(2) - 0.05;
		//  	_rpy_goal_left_hand(0) = _x_left_hand(3);
		//  	_rpy_goal_left_hand(1) = _x_left_hand(4);
		//  	_rpy_goal_left_hand(2) = _x_left_hand(5);
		//
		// 	_pos_goal_right_hand(0) = _x_right_hand(0);
		//  	_pos_goal_right_hand(1) = _x_right_hand(1);
		//  	_pos_goal_right_hand(2) = _x_right_hand(2) - 0.05;
		//  	_rpy_goal_right_hand(0) = _x_right_hand(3);
		//  	_rpy_goal_right_hand(1) = _x_right_hand(4);
		//  	_rpy_goal_right_hand(2) = _x_right_hand(5);
		// 	reset_target(_time_plan(_cnt_plan), _pos_goal_left_hand, _rpy_goal_left_hand, _pos_goal_right_hand, _rpy_goal_right_hand);
		// 	cout << "motion plan " << _cnt_plan  << " start. task space motion 1" << endl;
		// }
		// else if (_cnt_plan == 3) //task space motion 2
		// {
		// 	_pos_goal_left_hand(0) = 0.7;
		//  	_pos_goal_left_hand(1) = 0.24;
		//  	_pos_goal_left_hand(2) = 0.47;
		//  	_rpy_goal_left_hand(0) = _x_left_hand(3);
		//  	_rpy_goal_left_hand(1) = _x_left_hand(4);
		//  	_rpy_goal_left_hand(2) = _x_left_hand(5);
		//
		// 	_pos_goal_right_hand(0) = 0.7;
		//  	_pos_goal_right_hand(1) = 0.16;
		//  	_pos_goal_right_hand(2) = 0.47;
		//  	_rpy_goal_right_hand(0) = _x_right_hand(3);
		//  	_rpy_goal_right_hand(1) = _x_right_hand(4);
		//  	_rpy_goal_right_hand(2) = _x_right_hand(5);
		// 	reset_target(_time_plan(_cnt_plan), _pos_goal_left_hand, _rpy_goal_left_hand, _pos_goal_right_hand, _rpy_goal_right_hand);
		// 	cout << "motion plan " << _cnt_plan  << " start. task space motion 2" << endl;
		// }
		// else if (_cnt_plan == 4) //task space motion 3
		// {
		// 	_pos_goal_left_hand(0) = 0.7;
		//  	_pos_goal_left_hand(1) = _x_left_hand(1) - 0.01;
		//  	_pos_goal_left_hand(2) = _x_left_hand(2) + 0.05;
		//  	_rpy_goal_left_hand(0) = _x_left_hand(3);
		//  	_rpy_goal_left_hand(1) = _x_left_hand(4);
		//  	_rpy_goal_left_hand(2) = _x_left_hand(5);
		//
		// 	_pos_goal_right_hand(0) = 0.7;
		//  	_pos_goal_right_hand(1) = _x_right_hand(1) + 0.01;
		//  	_pos_goal_right_hand(2) = _x_right_hand(2) + 0.05;
		//  	_rpy_goal_right_hand(0) = _x_right_hand(3);
		//  	_rpy_goal_right_hand(1) = _x_right_hand(4);
		//  	_rpy_goal_right_hand(2) = _x_right_hand(5);
		//
		// 	reset_target(_time_plan(_cnt_plan), _pos_goal_left_hand, _rpy_goal_left_hand, _pos_goal_right_hand, _rpy_goal_right_hand);
		// 	cout << "motion plan " << _cnt_plan  << " start. task space motion 3" << endl;
		// }
		// else if (_cnt_plan == 5) //move to another task position
		// {
		// 	_pos_goal_left_hand(0) = 0.7;
		//  	_pos_goal_left_hand(1) = -0.2 + 0.05 - 0.03;
		//  	_pos_goal_left_hand(2) = 0.52;
		//  	_rpy_goal_left_hand(0) = 0.0 * DEG2RAD;
		// 	_rpy_goal_left_hand(1) = -90.0 * DEG2RAD;
		// 	_rpy_goal_left_hand(2) = 0.0 * DEG2RAD;
		//
		// 	_pos_goal_right_hand(0) = 0.7;
		//  	_pos_goal_right_hand(1) = -0.2 - 0.05 + 0.03;
		//  	_pos_goal_right_hand(2) = 0.52;
		//  	_rpy_goal_right_hand(0) = 0.0 * DEG2RAD;
		// 	_rpy_goal_right_hand(1) = -90.0 * DEG2RAD;
		// 	_rpy_goal_right_hand(2) = 180.0 * DEG2RAD;
		// 	reset_target(_time_plan(_cnt_plan), _pos_goal_left_hand, _rpy_goal_left_hand, _pos_goal_right_hand, _rpy_goal_right_hand);
		// 	cout << "motion plan " << _cnt_plan  << " start. move to another task position" << endl;
		// }
		// else if (_cnt_plan == 6) //task space motion 4
		// {
		// 	_pos_goal_left_hand(0) = _x_left_hand(0);
		//  	_pos_goal_left_hand(1) = _x_left_hand(1) - 0.01;
		//  	_pos_goal_left_hand(2) = 0.47;
		//  	_rpy_goal_left_hand(0) = _x_left_hand(3);
		//  	_rpy_goal_left_hand(1) = _x_left_hand(4);
		//  	_rpy_goal_left_hand(2) = _x_left_hand(5);
		//
		// 	_pos_goal_right_hand(0) = _x_right_hand(0);
		//  	_pos_goal_right_hand(1) = _x_right_hand(1) + 0.01;
		//  	_pos_goal_right_hand(2) = 0.47;
		//  	_rpy_goal_right_hand(0) = _x_right_hand(3);
		//  	_rpy_goal_right_hand(1) = _x_right_hand(4);
		//  	_rpy_goal_right_hand(2) = _x_right_hand(5);
		// 	reset_target(_time_plan(_cnt_plan), _pos_goal_left_hand, _rpy_goal_left_hand, _pos_goal_right_hand, _rpy_goal_right_hand);
		// 	cout << "motion plan " << _cnt_plan  << " start. task space motion 4" << endl;
		// }
		// else if (_cnt_plan == 7) // task space motion 5
		// {
		// 	_pos_goal_left_hand(0) = _x_left_hand(0);
		//  	_pos_goal_left_hand(1) = -0.2 + 0.05 + 0.1;
		//  	_pos_goal_left_hand(2) = _x_left_hand(2);
		//  	_rpy_goal_left_hand(0) = _x_left_hand(3);
		//  	_rpy_goal_left_hand(1) = _x_left_hand(4);
		//  	_rpy_goal_left_hand(2) = _x_left_hand(5);
		//
		// 	_pos_goal_right_hand(0) = _x_right_hand(0);
		//  	_pos_goal_right_hand(1) = -0.2 - 0.05 - 0.1;
		//  	_pos_goal_right_hand(2) = _x_right_hand(2);
		//  	_rpy_goal_right_hand(0) = _x_right_hand(3);
		//  	_rpy_goal_right_hand(1) = _x_right_hand(4);
		//  	_rpy_goal_right_hand(2) = _x_right_hand(5);
		// 	reset_target(_time_plan(_cnt_plan), _pos_goal_left_hand, _rpy_goal_left_hand, _pos_goal_right_hand, _rpy_goal_right_hand);
		// 	cout << "motion plan " << _cnt_plan  << " start. task space motion 5" << endl;
		// }
		// else if (_cnt_plan == 8) // task space motion 6
		// {
		// 	_pos_goal_left_hand(0) = _x_left_hand(0) - 0.1;
		//  	_pos_goal_left_hand(1) = _x_left_hand(1);
		//  	_pos_goal_left_hand(2) = _x_left_hand(2) + 0.05;
		//  	_rpy_goal_left_hand(0) = _x_left_hand(3);
		//  	_rpy_goal_left_hand(1) = _x_left_hand(4);
		//  	_rpy_goal_left_hand(2) = _x_left_hand(5);
		//
		// 	_pos_goal_right_hand(0) = _x_right_hand(0) - 0.1;
		//  	_pos_goal_right_hand(1) = _x_right_hand(1);
		//  	_pos_goal_right_hand(2) = _x_right_hand(2) + 0.05;
		//  	_rpy_goal_right_hand(0) = _x_right_hand(3);
		//  	_rpy_goal_right_hand(1) = _x_right_hand(4);
		//  	_rpy_goal_right_hand(2) = _x_right_hand(5);
		// 	reset_target(_time_plan(_cnt_plan), _pos_goal_left_hand, _rpy_goal_left_hand, _pos_goal_right_hand, _rpy_goal_right_hand);
		// 	cout << "motion plan " << _cnt_plan  << " start. task space motion 6" << endl;
		// }
		// else if (_cnt_plan == 9) // move home position
		// {
		// 	reset_target(_time_plan(_cnt_plan), _q_home);
		// 	cout << "motion plan " << _cnt_plan  << " start. move home position" << endl;
		// }
	}

	// Joystick Axis stick control plan
	// if (_bool_plan(_cnt_plan) == 1)
	// {
	// 	if (_LR_stick_left == 0 & _UD_stick_left == 0 & _LR_stick_right == 0 & _UD_stick_right == 0) //move task position
	// 	{
	// 		reset_target(3.0, _q_home);
	// 	}
	// 	else
	// 	{
	// 		_pos_goal_left_hand(0) = _x_left_hand(0) + _UD_stick_left;
	// 	 	_pos_goal_left_hand(1) = _x_left_hand(1) + _LR_stick_left;
	// 	 	_pos_goal_left_hand(2) = _x_left_hand(2);
	// 	 	_rpy_goal_left_hand(0) = _x_left_hand(3);
	// 		_rpy_goal_left_hand(1) = _x_left_hand(4);
	// 		_rpy_goal_left_hand(2) = _x_left_hand(5);
	//
	// 		_pos_goal_right_hand(0) = _x_right_hand(0) + _UD_stick_right;
	// 	 	_pos_goal_right_hand(1) = _x_right_hand(1) + _LR_stick_right;
	// 	 	_pos_goal_right_hand(2) = _x_right_hand(2);
	// 	 	_rpy_goal_right_hand(0) = _x_right_hand(3);
	// 		_rpy_goal_right_hand(1) = _x_right_hand(4);
	// 		_rpy_goal_right_hand(2) = _x_right_hand(5);
	// 		reset_target(3.0, _pos_goal_left_hand, _rpy_goal_left_hand, _pos_goal_right_hand, _rpy_goal_right_hand);
	// 		cout << "joystick motion start. move task position" << endl;
	// 	}
	// }
	
	// Joystick Button control plan
	// if(_bool_joy_button_push == true)
	// {
	// 	if (_joy_button == "Start")
	// 	{
	// 		_cnt_joy_mode = _cnt_joy_mode + 1;
	// 		switch (_cnt_joy_mode)
	// 		{
	// 		case 1:
	// 			cout << "Control mode [2] --Operational Space Control--" << endl;
	// 			_control_mode = 2;
	// 			break;
	// 		case 2:
	// 			cout << "Control mode [3] --HQP Task Space Control--" << endl;
	// 			_control_mode = 3;
	// 			break;
	// 		case 3:
	// 			cout << "Control mode [4] --Reduced HQP Task Space Control--" << endl;
	// 			_control_mode = 4;
	// 			break;
	// 		case 4:
	// 			cout << "Control mode [5] --Operational Space Control Without Body Link--" << endl;
	// 			_control_mode = 5;
	// 			break;
	// 		case 5:
	// 			cout << "Control mode init ..." << endl;
	// 			_cnt_joy_mode = 0;
	// 			break;
	// 		}
	// 		_pos_goal_left_hand(0) = _x_left_hand(0);
	// 	 	_pos_goal_left_hand(1) = _x_left_hand(1);
	// 	 	_pos_goal_left_hand(2) = _x_left_hand(2);
	// 	 	_rpy_goal_left_hand(0) = _x_left_hand(3);
	// 	 	_rpy_goal_left_hand(1) = _x_left_hand(4);
	// 	 	_rpy_goal_left_hand(2) = _x_left_hand(5);
	//
	// 		_pos_goal_right_hand(0) = _x_right_hand(0);
	// 	 	_pos_goal_right_hand(1) = _x_right_hand(1);
	// 	 	_pos_goal_right_hand(2) = _x_right_hand(2);
	// 	 	_rpy_goal_right_hand(0) = _x_right_hand(3);
	// 	 	_rpy_goal_right_hand(1) = _x_right_hand(4);
	// 	 	_rpy_goal_right_hand(2) = _x_right_hand(5);
	// 		reset_target(1000.0, _pos_goal_left_hand, _rpy_goal_left_hand, _pos_goal_right_hand, _rpy_goal_right_hand, _joy_button);
	// 		_bool_joy_button_push = false;
	// 	}
	// 	else if (_joy_button == "NA")
	// 	{
	// 		/* code */
	// 	}
	// 	else if (_bool_joy_plan(_cnt_plan) == 1)
	// 	{
	// 		if (_control_mode == 1)
	// 		{
	// 			_control_mode = 5;
	// 		}
	//		
	// 		_cnt_plan = _cnt_plan + 1;
	//
	// 		_rpy_goal_left_hand(0) = 0.0 * DEG2RAD;
	// 		_rpy_goal_left_hand(1) = -90.0 * DEG2RAD;
	// 		_rpy_goal_left_hand(2) = 90.0 * DEG2RAD;
	//
	// 		_rpy_goal_right_hand(0) = 0.0 * DEG2RAD;
	// 		_rpy_goal_right_hand(1) = -90.0 * DEG2RAD;
	// 		_rpy_goal_right_hand(2) = 90.0 * DEG2RAD;
	//
	// 		if (_joy_button == "A")
	// 		{
	// 			cout << "Joystick button [" << _joy_button << "] motion plan" << endl;
	// 			_pos_goal_left_hand(0) = 0.7 - 0.2;
	// 			_pos_goal_left_hand(1) = 0.075;
	// 			_pos_goal_left_hand(2) = 0.52;
	//
	// 			_pos_goal_right_hand(0) = 0.7 - 0.2;
	// 			_pos_goal_right_hand(1) = -0.075;
	// 			_pos_goal_right_hand(2) = 0.52;
	// 			reset_target(3.0, _pos_goal_left_hand, _rpy_goal_left_hand, _pos_goal_right_hand, _rpy_goal_right_hand, _joy_button);
	// 		}
	// 		else if (_joy_button == "B")
	// 		{
	// 			cout << "Joystick button [" << _joy_button << "] motion plan" << endl;
	// 			_pos_goal_left_hand(0) = 0.7;
	// 			_pos_goal_left_hand(1) = -0.2 + 0.075;
	// 			_pos_goal_left_hand(2) = 0.52;
	//
	// 			_pos_goal_right_hand(0) = 0.7;
	// 			_pos_goal_right_hand(1) = -0.2 - 0.075;
	// 			_pos_goal_right_hand(2) = 0.52;
	// 			reset_target(3.0, _pos_goal_left_hand, _rpy_goal_left_hand, _pos_goal_right_hand, _rpy_goal_right_hand, _joy_button);
	// 		}
	// 		else if (_joy_button == "X")
	// 		{
	// 			cout << "Joystick button [" << _joy_button << "] motion plan" << endl;
	// 			_pos_goal_left_hand(0) = 0.7;
	// 			_pos_goal_left_hand(1) = 0.2 + 0.075;
	// 			_pos_goal_left_hand(2) = 0.52;
	//
	// 			_pos_goal_right_hand(0) = 0.7;
	// 			_pos_goal_right_hand(1) = 0.2 - 0.075;
	// 			_pos_goal_right_hand(2) = 0.52;
	// 			reset_target(3.0, _pos_goal_left_hand, _rpy_goal_left_hand, _pos_goal_right_hand, _rpy_goal_right_hand, _joy_button);
	// 		}
	// 		else if (_joy_button == "Y")
	// 		{
	// 			cout << "Joystick button [" << _joy_button << "] motion plan" << endl;
	// 			_pos_goal_left_hand(0) = 0.7 + 0.1;
	// 			_pos_goal_left_hand(1) = 0.075;
	// 			_pos_goal_left_hand(2) = 0.52;
	//
	// 			_pos_goal_right_hand(0) = 0.7 + 0.1;
	// 			_pos_goal_right_hand(1) = -0.075;
	// 			_pos_goal_right_hand(2) = 0.52;
	// 			reset_target(3.0, _pos_goal_left_hand, _rpy_goal_left_hand, _pos_goal_right_hand, _rpy_goal_right_hand, _joy_button);
	// 		}
	// 	}
	// }
}

void CController::control_mujoco(_Float64 _joy_command[])
{	
	ModelUpdate();
	// Joystick node that subscribes /joy_command
	// joystickSub(_joy_command);
	motionPlan();
	//Control
	if (_control_mode == 1) //joint space control
	{
		if (_t - _init_t < 0.1 && _bool_joint_motion == false)
		{
			_start_time = _init_t;
			_end_time = _start_time + _motion_time;
			JointTrajectory.reset_initial(_start_time, _q, _qdot);
			JointTrajectory.update_goal(_q_goal, _qdot_goal, _end_time);
			_bool_joint_motion = true;
		}
		JointTrajectory.update_time(_t);
		_q_des = JointTrajectory.position_cubicSpline();
		_qdot_des = JointTrajectory.velocity_cubicSpline();
		JointControl();
		if (JointTrajectory.check_trajectory_complete() == 1)
		{
			_bool_plan(_cnt_plan) = 1;
			_bool_joy_plan(_cnt_plan) = 1;
			_bool_init = true;
			_bool_joy_button_push = false;
		}
	}
	else if (_control_mode == 2 || _control_mode == 3 || _control_mode == 4 || _control_mode == 5) //task space hand control
	{
		if (_t - _init_t < 0.1 && _bool_ee_motion == false)
		{ 
			_start_time = _init_t;
			_end_time = _start_time + _motion_time;
			LeftHandTrajectory.reset_initial(_start_time, _x_left_hand, _xdot_left_hand);
			LeftHandTrajectory.update_goal(_x_goal_left_hand, _xdot_goal_left_hand, _end_time);
			LeftHandTrajectory.reset_initial(_start_time, _q_left_hand, Model._R_left_hand, _w_left_hand , _wdot_left_hand); // Quaternion
			//LeftHandTrajectory.reset_initial(_start_time, _q_left_hand, CustomMath::CalcQuaternionToMatrix(_q_left_hand),_w_left_hand ,_wdot_left_hand); // Quaternion
			LeftHandTrajectory.update_goal(_q_goal_left_hand, CustomMath::GetBodyRotationMatrix(_x_goal_left_hand.tail(3)), _end_time);

			RightHandTrajectory.reset_initial(_start_time, _x_right_hand, _xdot_right_hand);
			RightHandTrajectory.update_goal(_x_goal_right_hand, _xdot_goal_right_hand, _end_time);
			RightHandTrajectory.reset_initial(_start_time, _q_right_hand, Model._R_right_hand, _w_right_hand , _wdot_right_hand);
			//RightHandTrajectory.reset_initial(_start_time, _q_right_hand, CustomMath::CalcQuaternionToMatrix(_q_right_hand),_w_right_hand ,_wdot_right_hand); // Quaternion CustomMath::GetBodyRotationMatrix(_x_right_hand(3), _x_right_hand(4), _x_right_hand(5))
			RightHandTrajectory.update_goal(_q_goal_right_hand, CustomMath::GetBodyRotationMatrix(_x_goal_right_hand.tail(3)), _end_time);

			_bool_ee_motion = true;
		}
		LeftHandTrajectory.update_time(_t);
		_x_des_left_hand = LeftHandTrajectory.position_cubicSpline();		
		_xdot_des_left_hand = LeftHandTrajectory.velocity_cubicSpline();
		_q_des_left_hand = LeftHandTrajectory.quaternion_SQUAD();
		// For quaternion cubic spline interpolation
		// _q_des_left_hand = LeftHandTrajectory.quaternion_angular_orientation_cubicSpline();
		// _w_des_left_hand = LeftHandTrajectory.quaternion_angular_velocity_cubicSpline(_dt);

		RightHandTrajectory.update_time(_t);		
		_x_des_right_hand = RightHandTrajectory.position_cubicSpline();		
		_xdot_des_right_hand = RightHandTrajectory.velocity_cubicSpline();
		_q_des_right_hand = RightHandTrajectory.quaternion_SQUAD();
		// _q_des_right_hand = RightHandTrajectory.quaternion_angular_orientation_cubicSpline();
		// _w_des_right_hand = RightHandTrajectory.quaternion_angular_velocity_cubicSpline(_dt);

		if (_control_mode == 2)
		{
			chrono::system_clock::time_point start = chrono::system_clock::now();

			OperationalSpaceControl();

			chrono::system_clock::time_point end = chrono::system_clock::now();
			chrono::nanoseconds nano = end - start;
			//printf("%f \n", nano / 1000.0);//millisec
		}
		else if (_control_mode == 3)
		{
			chrono::system_clock::time_point start = chrono::system_clock::now();

			HQPTaskSpaceControl();

			chrono::system_clock::time_point end = chrono::system_clock::now();
			chrono::nanoseconds nano = end - start;
			//printf("%f \n", nano / 1000.0);//millisec
			// _ofs_HQP << to_string(nano.count() / 1000.0) << endl;
			// if (++_idx == 10000)
			// {
			// 	cout << "Dataset of " << _idx << " numbers is collected!! Close file..." << endl;
			// 	_ofs_HQP.close();
			// }
		}
		else if (_control_mode == 4)
		{
			chrono::system_clock::time_point start = chrono::system_clock::now();

			ReducedHQPTaskSpaceControl();

			chrono::system_clock::time_point end = chrono::system_clock::now();
			chrono::nanoseconds nano = end - start;
			//printf("%f \n", nano/1000.0);//millisec
			// _ofs_RHQP << to_string(nano.count() / 1000.0) << endl;
			// if (++_idx == 10000)
			// {
			// 	cout << "Dataset of " << _idx << " numbers is collected!! Close file..." << endl;
			// 	_ofs_RHQP.close();
			// }
		}
		else if (_control_mode == 5)
		{
			chrono::system_clock::time_point start = chrono::system_clock::now();

			OperationalSpaceControlWithoutBodyLink();

			chrono::system_clock::time_point end = chrono::system_clock::now();
			chrono::nanoseconds nano = end - start;
			// //printf("%f \n", nano / 1000.0);//millisec
			// _ofs_OSF << to_string(nano.count() / 1000.0) << endl;
			// if (++_idx == 10000)
			// {
			// 	cout << "Dataset of " << _idx << " numbers is collected!! Close file..." << endl;
			// 	_ofs_OSF.close();
			// }
		}

		if (LeftHandTrajectory.check_trajectory_complete() == 1 || RightHandTrajectory.check_trajectory_complete() == 1)
		{
			//cout << "end motion cnt plan" << _cnt_plan << endl; 
			_bool_plan(_cnt_plan) = 1;
			_bool_joy_plan(_cnt_plan) = 1;
			_bool_joy_button_push = false;
			_bool_init = true;
			
			// for repeating task !!!!!!!!!!!!!!!!!!!!!!!!!
			// if(_cnt_plan == 5)
			// {
			// 	_bool_plan.setZero(20);
			// 	_cnt_plan = 0;
			// 	reset_target(_time_plan(_cnt_plan), _q_home);
			// 	cout << "cnt plan initialize!!" << endl;
			// }
		}
	}

	safeModeReplaceTorque(_bool_safemode);
	
}

void CController::ModelUpdate()
{
	Model.update_kinematics(_q, _qdot);
	Model.update_dynamics();
	Model.calculate_EE_Jacobians();
	Model.calculate_EE_positions_orientations();
	Model.calculate_EE_velocity();
	Model.calculate_Euler_To_Quat();

	//set Jacobian
	_J_hands.block<6, 15>(0, 0) = Model._J_left_hand;
	_J_hands.block<6, 15>(6, 0) = Model._J_right_hand;
	_J_T_hands = _J_hands.transpose();

	_J_ori_hands.block<3, 15>(0, 0) = Model._J_left_hand_ori;
	_J_ori_hands.block<3, 15>(3, 0) = Model._J_right_hand_ori;
	_J_ori_T_hands = _J_ori_hands.transpose();
	_J_pos_hands.block<3, 15>(0, 0) = Model._J_left_hand_pos;
	_J_pos_hands.block<3, 15>(3, 0) = Model._J_right_hand_pos;
	_J_pos_T_hands = _J_pos_hands.transpose();

	//calc Jacobian dot (with lowpass filter)	
	for (int i = 0; i < 12; i++)
	{
		for (int j = 0; j < 15; j++)
		{
			_Jdot_hands(i,j) = CustomMath::VelLowpassFilter(_dt, 2.0 * PI * 20.0, _pre_J_hands(i,j), _J_hands(i,j), _pre_Jdot_hands(i,j)); //low-pass filter

			_pre_J_hands(i, j) = _J_hands(i, j);
			_pre_Jdot_hands(i, j) = _Jdot_hands(i, j);
		}		
	}
	_Jdot_qdot = _Jdot_hands * _qdot;

	_x_left_hand.head(3) = Model._x_left_hand;
	_x_left_hand.tail(3) = CustomMath::GetBodyRotationAngle(Model._R_left_hand);
	_x_right_hand.head(3) = Model._x_right_hand;
	_x_right_hand.tail(3) = CustomMath::GetBodyRotationAngle(Model._R_right_hand);
	_xdot_left_hand = Model._xdot_left_hand;
	_xdot_right_hand = Model._xdot_right_hand;

	// Quaternion
	_q_left_hand = Model._q_left_hand;
	_q_right_hand = Model._q_right_hand;

	_qdot_left_hand.w() = CustomMath::VelLowpassFilter(_dt, 2.0 * PI * 20.0, _pre_q_left_hand.w(), _q_left_hand.w(), _pre_qdot_left_hand.w());
	_qdot_right_hand.w() = CustomMath::VelLowpassFilter(_dt, 2.0 * PI * 20.0, _pre_q_right_hand.w(), _q_right_hand.w(), _pre_qdot_right_hand.w());
	
	_pre_q_left_hand.w() = _q_left_hand.w();
	_pre_q_right_hand.w() = _q_right_hand.w();
	_pre_qdot_left_hand.w() = _qdot_left_hand.w();
	_pre_qdot_right_hand.w() = _qdot_right_hand.w();

	for (int i = 0; i < 3; i++)
	{
		_qdot_left_hand.vec().coeffRef(i) = CustomMath::VelLowpassFilter(_dt, 2.0 * PI * 20.0, _pre_q_left_hand.vec().coeffRef(i), _q_left_hand.vec().coeffRef(i), _pre_qdot_left_hand.vec().coeffRef(i));
		_qdot_right_hand.vec().coeffRef(i) = CustomMath::VelLowpassFilter(_dt, 2.0 * PI * 20.0, _pre_q_right_hand.vec().coeffRef(i), _q_right_hand.vec().coeffRef(i), _pre_qdot_right_hand.vec().coeffRef(i));
	
		_pre_q_left_hand.vec().coeffRef(i) = _q_left_hand.vec().coeffRef(i);
		_pre_q_right_hand.vec().coeffRef(i) = _q_right_hand.vec().coeffRef(i);
		_pre_qdot_left_hand.vec().coeffRef(i) = _qdot_left_hand.vec().coeffRef(i);
		_pre_qdot_right_hand.vec().coeffRef(i) = _qdot_right_hand.vec().coeffRef(i);
	}	

	_vec_qdot_left_hand(0) = _qdot_left_hand.w();
	_vec_qdot_right_hand(0) = _qdot_right_hand.w();
	_vec_qdot_left_hand.tail(3) = _qdot_left_hand.vec();
	_vec_qdot_right_hand.tail(3) = _qdot_right_hand.vec();

	// Orientation angular velocity & acceleration
	_w_left_hand = 2.0 * CustomMath::CalcAngularVelFromBodyFixedQuaternion(_q_left_hand) * _vec_qdot_left_hand;
	_w_right_hand = 2.0 * CustomMath::CalcAngularVelFromBodyFixedQuaternion(_q_right_hand) * _vec_qdot_right_hand;
	for (int i = 0; i < 3; i++)
	{
		_wdot_left_hand(i) = CustomMath::VelLowpassFilter(_dt, 2.0 * PI * 20.0, _pre_w_left_hand(i), _w_left_hand(i), _pre_wdot_left_hand(i)); //low-pass filter
		_wdot_right_hand(i) = CustomMath::VelLowpassFilter(_dt, 2.0 * PI * 20.0, _pre_w_right_hand(i), _w_right_hand(i), _pre_wdot_right_hand(i)); //low-pass filter

		_pre_w_left_hand(i) = _w_left_hand(i);
		_pre_w_right_hand(i) = _w_right_hand(i);
		_pre_wdot_left_hand(i) =_wdot_left_hand(i);
		_pre_wdot_right_hand(i) =_wdot_right_hand(i);
	}
	
}

void CController::JointControl()
{
	_torque.setZero();
	_torque = Model._A*(_kpj*(_q_des - _q) + _kdj*(_qdot_des - _qdot) ) + Model._bg;
}

void CController::OperationalSpaceControl()
{
	_torque.setZero();	
	_kp = 50.0;
	_kd = 15.0;

	_x_err_left_hand = _x_des_left_hand.head(3) - Model._x_left_hand;
	_R_des_left_hand = CustomMath::GetBodyRotationMatrix(_x_des_left_hand(3), _x_des_left_hand(4), _x_des_left_hand(5));	
	_R_err_left_hand = -CustomMath::getPhi(Model._R_left_hand, _R_des_left_hand);
	_x_err_right_hand = _x_des_right_hand.head(3) - Model._x_right_hand;
	_R_des_right_hand = CustomMath::GetBodyRotationMatrix(_x_des_right_hand(3), _x_des_right_hand(4), _x_des_right_hand(5));
	_R_err_right_hand = -CustomMath::getPhi(Model._R_right_hand, _R_des_right_hand);

	_xdot_err_left_hand = _xdot_des_left_hand.head(3) - Model._xdot_left_hand.segment(0, 3);
	_Rdot_err_left_hand = -Model._xdot_left_hand.segment(3, 3); //only daming for orientation
	_xdot_err_right_hand = _xdot_des_right_hand.head(3) - Model._xdot_right_hand.segment(0, 3);
	_Rdot_err_right_hand = -Model._xdot_right_hand.segment(3, 3); //only daming for orientation	
		
	// 1st: hands pos and ori, 2nd: joint dampings
	_Lambda_hands.setZero();
	_J_bar_T_hands.setZero();
	_J_bar_T_hands = CustomMath::pseudoInverseQR(_J_T_hands);
	_Lambda_hands = _J_bar_T_hands * Model._A * CustomMath::pseudoInverseQR(_J_hands);
	_Null_hands = _Id_15 - _J_T_hands * _Lambda_hands * _J_hands * Model._A.inverse();	

	_xddot_star.segment(0, 3) = _kp * _x_err_left_hand + _kd * _xdot_err_left_hand;//left hand position control
	_xddot_star.segment(3, 3) = _kp * _R_err_left_hand + _kd * _Rdot_err_left_hand;//left hand orientation control
	_xddot_star.segment(6, 3) = _kp * _x_err_right_hand + _kd * _xdot_err_right_hand;//right hand position control
	_xddot_star.segment(9, 3) = _kp * _R_err_right_hand + _kd * _Rdot_err_right_hand;//right hand orientation control

	safeWorkSpaceLimit();
	_xddot_star.segment(0, 3) = _kp * _x_err_left_hand + _kd * _xdot_err_left_hand + _acc_workspace_avoid_left;
	_xddot_star.segment(6, 3) = _kp * _x_err_right_hand + _kd * _xdot_err_right_hand + _acc_workspace_avoid_left;

	_torque = _J_T_hands * _Lambda_hands * _xddot_star + _Null_hands * (Model._A * (-_kdj * _qdot)) + Model._bg;

	_S_T.setZero(); //select wrist joints
	_S_T(5, 0) = 1.0;
	_S_T(6, 1) = 1.0;
	_S_T(7, 2) = 1.0;
	_S_T(12, 3) = 1.0;
	_S_T(13, 4) = 1.0;
	_S_T(14, 5) = 1.0;
	//_J_bar_T_hands_S_T = _J_bar_T_hands * _S_T;
	//_W_mat_S = _S_T.transpose() * (Model._A).inverse() * _S_T;
	//_W_mat_S.setIdentity();
	//cout << _W_mat_S << endl << endl;
	//cout << _J_bar_T_hands_S_T << endl << endl;
	//_J_tilde_T = CustomMath::WeightedPseudoInverse(_J_bar_T_hands_S_T, _W_mat_S, false);
	//_J_tilde_T = CustomMath::pseudoInverseQR(_J_bar_T_hands_S_T);
	_J_bar_T_hands_S_T = _J_hands* Model._A.inverse()* _S_T;
	_J_tilde_T = CustomMath::pseudoInverseQR(_J_bar_T_hands_S_T);
	//cout << _J_tilde_T << endl << endl;

	_xddot_reinforce.segment(0, 3) = 16000.0 * _x_err_left_hand + 40.0 * _xdot_err_left_hand;//left hand position control
	_xddot_reinforce.segment(3, 3) = 16000.0 * _R_err_left_hand + 40.0  * _Rdot_err_left_hand;//left hand orientation control
	_xddot_reinforce.segment(6, 3) = 16000.0 * _x_err_right_hand + 40.0  * _xdot_err_right_hand;//right hand position control
	_xddot_reinforce.segment(9, 3) = 16000.0 * _R_err_right_hand + 40.0 * _Rdot_err_right_hand;//right hand orientation control	

	_torque_reinforce = _S_T * _J_tilde_T * _Lambda_hands * _xddot_reinforce;

	for (int i = 0; i < 15; i++)
	{
		if (_torque_reinforce(i) > 5.0)
		{
			_torque_reinforce(i) = 5.0;
		}
		else if (_torque_reinforce(i) < -5.0)
		{
			_torque_reinforce(i) = -5.0;
		}
	}	

	_torque = _J_T_hands * _Lambda_hands * _xddot_star + _Null_hands * (Model._A * (-_kdj * _qdot)) + Model._bg + _torque_reinforce;

	//cout << (_S_T * _J_tilde_T * _Lambda_hands * _xddot_reinforce).transpose() << endl;

	//cout << _x_err_left_hand.transpose() << " and " << _x_err_right_hand.transpose() << endl <<endl;
}

void CController::OperationalSpaceControlWithoutBodyLink()
{
	_torque.setZero();
	_torque_a.setZero();
	_kp = 100.0;
	_kd = 20.0;
	_ka = 10.0;
	_ko = 10.0;

	_x_err_left_hand = _x_des_left_hand.head(3) - Model._x_left_hand;
	_R_des_left_hand = CustomMath::GetBodyRotationMatrix(_x_des_left_hand(3), _x_des_left_hand(4), _x_des_left_hand(5));	
	_R_err_left_hand = -CustomMath::getPhi(Model._R_left_hand, _R_des_left_hand);
	_x_err_right_hand = _x_des_right_hand.head(3) - Model._x_right_hand;
	_R_des_right_hand = CustomMath::GetBodyRotationMatrix(_x_des_right_hand(3), _x_des_right_hand(4), _x_des_right_hand(5));
	_R_err_right_hand = -CustomMath::getPhi(Model._R_right_hand, _R_des_right_hand);

	_xdot_err_left_hand = _xdot_des_left_hand.head(3) - Model._xdot_left_hand.segment(0, 3);
	_Rdot_err_left_hand = -Model._xdot_left_hand.segment(3, 3); //only daming for orientation
	_xdot_err_right_hand = _xdot_des_right_hand.head(3) - Model._xdot_right_hand.segment(0, 3);
	_Rdot_err_right_hand = -Model._xdot_right_hand.segment(3, 3); //only daming for orientation	
	safeWorkSpaceLimit();
	_xddot_star.segment(0, 3) = _kp * _x_err_left_hand + _kd * _xdot_err_left_hand + _acc_workspace_avoid_left;
	_xddot_star.segment(6, 3) = _kp * _x_err_right_hand + _kd * _xdot_err_right_hand + _acc_workspace_avoid_left;

	_q_des_vec_left_hand = _q_des_left_hand.vec();
	_q_des_vec_right_hand = _q_des_right_hand.vec();
	_q_des_scalar_left_hand = _q_des_left_hand.w();
	_q_des_scalar_right_hand = _q_des_right_hand.w();
	_q_des_skew_left_hand = CustomMath::skew(_q_des_vec_left_hand);
	_q_des_skew_right_hand = CustomMath::skew(_q_des_vec_right_hand);

	_q_err_left_hand = _q_des_scalar_left_hand * _q_left_hand.vec() - _q_left_hand.w() * _q_des_vec_left_hand + _q_des_skew_left_hand * _q_left_hand.vec();
	_q_err_right_hand = _q_des_scalar_right_hand * _q_right_hand.vec() - _q_right_hand.w() * _q_des_vec_right_hand + _q_des_skew_right_hand * _q_right_hand.vec();

	// Method 1
	// _Lambda_hands_prism.setZero();
	// _Lambda_hands_prism = CustomMath::pseudoInverseQR(_J_T_hands_prism) * Model._A * CustomMath::pseudoInverseQR(_J_hands_prism);
	// _Null_hands_prism = _Id_15 - _J_T_hands_prism * _Lambda_hands_prism * _J_hands_prism * Model._A.inverse();
	// cout << "_J_hands_prism Weighted pseudo inverse 1 : " << endl << CustomMath::WeightedPseudoInverse(_J_hands_prism, Model._A, false) << endl;
	// cout << "_J_hands_prism Weighted pseudo inverse 2 : " << endl << _Lambda_hands_prism * _J_hands_prism * Model._A.inverse() << endl;

	// _Lambda_hands.setZero();
	// _Lambda_hands = CustomMath::pseudoInverseQR(_J_T_hands) * Model._A * CustomMath::pseudoInverseQR(_J_hands);
	// _Null_hands = _Id_15 - _J_T_hands * _Lambda_hands * _J_hands * Model._A.inverse();
	_Lambda_hands.setZero();
	_Lambda_hands = CustomMath::pseudoInverseQR(_J_T_hands) * Model._A * CustomMath::pseudoInverseQR(_J_hands);

	_J_Winv_hands = Model._A.inverse() * _J_T_hands *_Lambda_hands;
	_J_Winv_T_hands = _Lambda_hands * _J_hands * Model._A.inverse();
	_J_K_hands = _J_Winv_T_hands * _S_T_hands;
	_J_K_T_hands = Model._A.block<14, 14>(1, 1).inverse() * _J_K_hands.transpose() * CustomMath::pseudoInverseQR(_J_K_hands.transpose()) * Model._A.block<14, 14>(1, 1) * CustomMath::pseudoInverseQR(_J_K_hands);

	//_Null_hands = _Id_15 - _J_T_hands * _Lambda_hands * _J_hands * Model._A.inverse();
	_Null_hands_K = _Id_14 - _J_K_T_hands * CustomMath::pseudoInverseQR(_J_K_T_hands) * Model._A.block<14, 14>(1, 1) * CustomMath::pseudoInverseQR(_J_K_T_hands.transpose()) * _J_K_T_hands.transpose() * Model._A.block<14, 14>(1, 1).inverse();

	_xddot_star.segment(0, 3) = _kp * _x_err_left_hand + _kd * _xdot_err_left_hand;//left hand position control
	//_xddot_star.segment(3, 3) = _ka * (_w_des_left_hand - _x_left_hand.tail(3)) - _ko * _q_err_left_hand;//left hand orientation quaternion control _R_err_left_hand
	//_xddot_star.segment(3, 3) = _ka * _R_err_left_hand - _ko * _q_err_left_hand;//left hand orientation quaternion control _R_err_left_hand
	_xddot_star.segment(3, 3) = _kp * _R_err_left_hand + _kd * _Rdot_err_left_hand;//left hand orientation control
	_xddot_star.segment(6, 3) = _kp * _x_err_right_hand + _kd * _xdot_err_right_hand;//right hand position control
	// _xddot_star.segment(9, 3) = _ka * (_w_des_right_hand - _x_right_hand.tail(3)) - _ko * _q_err_right_hand;//right hand orientation control _R_err_right_hand
	//_xddot_star.segment(9, 3) = _ka * _R_err_right_hand - _ko * _q_err_right_hand;//right hand orientation control _R_err_right_hand
	_xddot_star.segment(9, 3) = _kp * _R_err_right_hand + _kd * _Rdot_err_right_hand;//right hand orientation control

	_torque_a = _J_K_T_hands * _Lambda_hands * _xddot_star + _Null_hands_K * (Model._A.block<14, 14>(1, 1) * (-_kdj * _qdot.segment(1, 14)));
	Model._bg.head(1).setZero();
	_torque.segment(1, 14) = _torque_a + Model._bg.segment(1, 14);
	//cout << "Torque------" << endl << _torque << endl;
	//_torque = _Null_hands_prism * (_J_T_hands * _Lambda_hands * _xddot_star + _Null_hands * (Model._A * (-_kdj * _qdot))) + Model._bg;
	// _J_K_T_hands = Model._A.block<14, 14>(1, 1).inverse() * _J_K_hands.transpose() * CustomMath::pseudoInverseQR(_J_K_hands.transpose()) * Model._A.block<14, 14>(1, 1) * CustomMath::pseudoInverseQR(_J_K_hands.block<12, 7>(0, 0));
	// _J_K_T_hands.block<7, 12>(7, 0) = Model._A.block<7, 7>(12, 12).inverse() * _J_K_hands.transpose().block<7, 12>(7, 0) * CustomMath::pseudoInverseQR(_J_K_hands.transpose().block<7, 12>(7, 0)) * Model._A.block<7, 7>(12, 12) * CustomMath::pseudoInverseQR(_J_K_hands.block<12, 7>(0, 7));

	//_Null_hands = _Id_15 - _J_T_hands * _Lambda_hands * _J_hands * Model._A.inverse();
	// _Null_hands_K.block<7, 7>(0, 0) = _Id_14.block<7, 7>(0, 0) - _J_K_T_hands.block<7, 12>(0, 0) * CustomMath::pseudoInverseQR(_J_K_T_hands.block<7, 12>(0, 0)) * Model._A.block<7, 7>(1, 1) * CustomMath::pseudoInverseQR(_J_K_T_hands.transpose().block<12, 7>(0, 0)) * _J_K_T_hands.transpose().block<12, 7>(0, 0) * Model._A.block<7, 7>(1, 1).inverse();
	// _Null_hands_K.block<7, 7>(7, 7) = _Id_14.block<7, 7>(7, 7) - _J_K_T_hands.block<7, 12>(7, 0) * CustomMath::pseudoInverseQR(_J_K_T_hands.block<7, 12>(7, 0)) * Model._A.block<7, 7>(12, 12) * CustomMath::pseudoInverseQR(_J_K_T_hands.transpose().block<12, 7>(0, 7)) * _J_K_T_hands.transpose().block<12, 7>(0, 7) * Model._A.block<7, 7>(12, 12).inverse();
	// _torque_a.head(7) = _J_K_T_hands.block<7, 12>(0, 0) * _Lambda_hands * _xddot_star + _Null_hands_K.block<7, 7>(0, 0) * (Model._A.block<7, 7>(1, 1) * (-_kdj * _qdot.segment(1, 7)));
	// _torque_a.tail(7) = _J_K_T_hands.block<7, 12>(7, 0) * _Lambda_hands * _xddot_star + _Null_hands_K.block<7, 7>(7, 7) * (Model._A.block<7, 7>(12, 12) * (-_kdj * _qdot.segment(12, 7)));
	// Model._bg.head(1).setZero();
	// Model._bg.segment(8, 4).setZero();
	// Model._bg.segment(19, 16).setZero();
	// _torque.segment(1, 7) = _torque_a.head(7) + Model._bg.segment(1, 7);
	// _torque.segment(12, 7) = _torque_a.tail(7) + Model._bg.segment(12, 7);
}

void CController::HQPTaskSpaceControl()
{
	_torque.setZero();	

	_kp = 100.0;
	_kd = 20.0;

	_x_err_left_hand = _x_des_left_hand.head(3) - Model._x_left_hand;
	_R_des_left_hand = CustomMath::GetBodyRotationMatrix(_x_des_left_hand(3), _x_des_left_hand(4), _x_des_left_hand(5));	
	_R_err_left_hand = -CustomMath::getPhi(Model._R_left_hand, _R_des_left_hand);
	_x_err_right_hand = _x_des_right_hand.head(3) - Model._x_right_hand;
	_R_des_right_hand = CustomMath::GetBodyRotationMatrix(_x_des_right_hand(3), _x_des_right_hand(4), _x_des_right_hand(5));
	_R_err_right_hand = -CustomMath::getPhi(Model._R_right_hand, _R_des_right_hand);

	_xdot_err_left_hand = _xdot_des_left_hand.head(3) - Model._xdot_left_hand.segment(0, 3);
	_Rdot_err_left_hand = -Model._xdot_left_hand.segment(3, 3); //only daming for orientation
	_xdot_err_right_hand = _xdot_des_right_hand.head(3) - Model._xdot_right_hand.segment(0, 3);
	_Rdot_err_right_hand = -Model._xdot_right_hand.segment(3, 3); //only daming for orientation	

	_xddot_star.segment(0, 3) = _kp * _x_err_left_hand + _kd * _xdot_err_left_hand;//left hand position control
	_xddot_star.segment(3, 3) = _kp * _R_err_left_hand + _kd * _Rdot_err_left_hand;//left hand orientation control
	_xddot_star.segment(6, 3) = _kp * _x_err_right_hand + _kd * _xdot_err_right_hand;//right hand position control
	_xddot_star.segment(9, 3) = _kp * _R_err_right_hand + _kd * _Rdot_err_right_hand;//right hand orientation control

	safeWorkSpaceLimit();
	_xddot_star.segment(0, 3) = _kp * _x_err_left_hand + _kd * _xdot_err_left_hand + _acc_workspace_avoid_left;
	_xddot_star.segment(6, 3) = _kp * _x_err_right_hand + _kd * _xdot_err_right_hand + _acc_workspace_avoid_left;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Solve HQP   /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	double threshold = 0.0001;
	int max_iter = 1000;			
	//first priority task QP ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//set cost function x^T*H*x + g	
	_H1.setZero();
	for (int i = 0; i < 30; i++)
	{
		_H1(i, i) = 0.00001;
	}
	for (int i = 30; i < 42; i++)
	{
		_H1(i, i) = 1.0;
	}
	//_H1.block<12, 12>(30, 30) = _Id_12;
	_g1.setZero();	
	HQP_P1.UpdateMinProblem(_H1,_g1);
	
	//set A*x <= b	
	_A1.setZero();
	_lbA1.setZero();
	_ubA1.setZero();	
	_A1.block<15, 15>(0, 0) = Model._A;
	_A1.block<15, 15>(0, 15) = -_Id_15;
	_A1.block<12, 15>(15, 0) = _J_hands;
	_A1.block<12, 12>(15, 30) = -_Id_12;
	for (int i = 0; i < 15; i++)
	{
		_lbA1(i) = -Model._bg(i) - threshold;
		_ubA1(i) = -Model._bg(i) + threshold;
	}
	for (int i = 0; i < 12; i++)
	{
		_lbA1(i + 15) = -_Jdot_qdot(i) + _xddot_star(i) - threshold;
		_ubA1(i + 15) = -_Jdot_qdot(i) + _xddot_star(i) + threshold;
	}	
	HQP_P1.UpdateSubjectToAx(_A1, _lbA1, _ubA1);
	
	//set lb <= x <= ub	
	_lb1.setZero();
	_ub1.setZero();
	//joint acceleration limit (for joint position and velocity)
	for (int i = 0; i < 15; i++)
	{
		_lb1(i) = -500.0;
		_ub1(i) = 500.0;
	}
	//joint torque limit
	for (int i = 0; i < 15; i++)
	{
		_lb1(i + 15) = -2000.0;
		_ub1(i + 15) = 2000.0;
	}
	//task limit	
	for (int i = 0; i < 12; i++)
	{
		_lb1(i + 30) = -1000000.0;
		_ub1(i + 30) = 1000000.0;
	}
	_lb1(15) = Model._bg(0) - threshold;
    _ub1(15) = Model._bg(0) + threshold;
	HQP_P1.UpdateSubjectToX(_lb1, _ub1);

	//Solve
	HQP_P1.EnableEqualityCondition(0.0001);	
	HQP_P1.SolveQPoases(max_iter);
	_torque = HQP_P1._Xopt.segment(15, 15);
	//cout << "slack: " << HQP_P1._Xopt.segment(30, 12).transpose() << endl;

	//second priority task QP ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	//set cost function x^T*H*x + g
	_H2.setZero();
	for (int i = 0; i < 30; i++) //torque
	{
		_H2(i, i) = 0.00001;
	}
	for (int i = 30; i < 45; i++) //slack
	{
		_H2(i, i) = 100000.0;
	}
	_H2.block<15,15>(30,30) = Model._A;  //acceleration
	_g2.setZero();
	HQP_P2.UpdateMinProblem(_H2, _g2);

	//set A*x <= b	
	_A2.setZero();
	_lbA2.setZero();
	_ubA2.setZero();
	_A2.block<15, 15>(0, 0) = Model._A;
	_A2.block<15, 15>(0, 15) = -_Id_15;
	_A2.block<15, 15>(15, 0) = _Id_15;
	_A2.block<15, 15>(15, 30) = -_Id_15;
	_A2.block<12, 15>(30, 0) = _J_hands;
	
	VectorXd joint_acc_des(15);
	joint_acc_des.setZero();
	_kdj = 40.0;
	joint_acc_des = -_kdj * _qdot;
	joint_acc_des(0) = 0;
	//joint_acc_des(0) = 400.0 * (_q_home(0)- _q(0)) - _kdj * _qdot(0);
	for (int i = 0; i < 15; i++)
	{
		_lbA2(i) = -Model._bg(i) - threshold;
		_ubA2(i) = -Model._bg(i) + threshold;
		_lbA2(i + 15) = joint_acc_des(i) - threshold;
		_ubA2(i + 15) = joint_acc_des(i) + threshold;
	}
	for (int i = 0; i < 12; i++)
	{
		_lbA2(i + 30) = -_Jdot_qdot(i) + _xddot_star(i) + HQP_P1._Xopt(i+30) - threshold;
		_ubA2(i + 30) = -_Jdot_qdot(i) + _xddot_star(i) + HQP_P1._Xopt(i+30) + threshold;
	}
	HQP_P2.UpdateSubjectToAx(_A2, _lbA2, _ubA2);

	//set lb <= x <= ub
	_lb2.setZero();
	_ub2.setZero();
	//joint acceleration limit (for joint position and velocity)
	for (int i = 0; i < 15; i++)
	{
		_lb2(i) = -500.0;
		_ub2(i) = 500.0;
	}
	//joint torque limit
	for (int i = 0; i < 15; i++)
	{
		_lb2(i + 15) = -2000.0;
		_ub2(i + 15) = 2000.0;
	}
	//task limit	
	for (int i = 0; i < 15; i++)
	{
		_lb2(i + 30) = -1000000.0;
		_ub2(i + 30) = 1000000.0;
	}
	//_lb2(0) = -Model._A(0)*Model._bg(0) - threshold;
	//_ub2(0) = -Model._A(0)*Model._bg(0) + threshold;
	//_lb2(15) = 0 - threshold;
	//_ub2(15) = 0 + threshold;
	_lb2(15) = Model._bg(0) - threshold;
	_ub2(15) = Model._bg(0) + threshold;
	HQP_P2.UpdateSubjectToX(_lb2, _ub2);	
	//Solve
	HQP_P2.EnableEqualityCondition(0.0001);	
	HQP_P2.SolveQPoases(max_iter);
	_torque = HQP_P2._Xopt.segment(15, 15);
	//cout << "------------- torque ---------------------\n" << HQP_P2._Xopt.segment(15, 15). transpose() << endl;
}

void CController::ReducedHQPTaskSpaceControl()
{
	_torque.setZero();

	_kp = 100.0;
	_kd = 20.0;

	_x_err_left_hand = _x_des_left_hand.head(3) - Model._x_left_hand;
	_R_des_left_hand = CustomMath::GetBodyRotationMatrix(_x_des_left_hand(3), _x_des_left_hand(4), _x_des_left_hand(5));	
	_R_err_left_hand = -CustomMath::getPhi(Model._R_left_hand, _R_des_left_hand);
	_x_err_right_hand = _x_des_right_hand.head(3) - Model._x_right_hand;
	_R_des_right_hand = CustomMath::GetBodyRotationMatrix(_x_des_right_hand(3), _x_des_right_hand(4), _x_des_right_hand(5));
	_R_err_right_hand = -CustomMath::getPhi(Model._R_right_hand, _R_des_right_hand);

	_xdot_err_left_hand = _xdot_des_left_hand.head(3) - Model._xdot_left_hand.segment(0, 3);
	_Rdot_err_left_hand = -Model._xdot_left_hand.segment(3, 3); //only daming for orientation
	_xdot_err_right_hand = _xdot_des_right_hand.head(3) - Model._xdot_right_hand.segment(0, 3);
	_Rdot_err_right_hand = -Model._xdot_right_hand.segment(3, 3); //only daming for orientation	

	_xddot_star.segment(0, 3) = _kp * _x_err_left_hand + _kd * _xdot_err_left_hand; //left hand position control
	_xddot_star.segment(3, 3) = _kp * _R_err_left_hand + _kd * _Rdot_err_left_hand; //left hand orientation control
	_xddot_star.segment(6, 3) = _kp * _x_err_right_hand + _kd * _xdot_err_right_hand; //right hand position control
	_xddot_star.segment(9, 3) = _kp * _R_err_right_hand + _kd * _Rdot_err_right_hand; //right hand orientation control

	safeWorkSpaceLimit();
	_xddot_star.segment(0, 3) = _kp * _x_err_left_hand + _kd * _xdot_err_left_hand + _acc_workspace_avoid_left;
	_xddot_star.segment(6, 3) = _kp * _x_err_right_hand + _kd * _xdot_err_right_hand + _acc_workspace_avoid_left;
		

	if (_bool_safemode == false)
	{
		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// Solve rHQP   /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		double threshold = 0.001;
		int max_iter = 1000;
		//first priority task QP ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//set cost function x^T*H*x + g	
		_rH1.setZero();
		for (int i = 0; i < 15; i++) //torque
		{
			_rH1(i, i) = 1.0;
		}
		for (int i = 15; i < 27; i++) //slack
		{
			_rH1(i, i) = 1000000.0;
		}
		//_rH1.block<15, 15>(0, 0) = Model._A.inverse();
		_rg1.setZero();
		rHQP_P1.UpdateMinProblem(_rH1, _rg1);

		MatrixXd J_Ainv(12, 15);
		J_Ainv = _J_hands * Model._A.inverse();

		//set A*x <= b
		_rA1.setZero();
		_rlbA1.setZero();
		_rubA1.setZero();
		_rA1.block<12, 15>(0, 0) = J_Ainv;
		_rA1.block<12, 12>(0, 15) = -_Id_12;

		for (int i = 0; i < 12; i++)
		{
			_rlbA1(i) = -_Jdot_qdot(i) + _xddot_star(i) - threshold;
			_rubA1(i) = -_Jdot_qdot(i) + _xddot_star(i) + threshold;
		}
		rHQP_P1.UpdateSubjectToAx(_rA1, _rlbA1, _rubA1);

		//set lb <= x <= ub	
		_rlb1.setZero();
		_rub1.setZero();
		//joint torque limit
		for (int i = 0; i < 15; i++)
		{
			//torque limit
			//_rlb1(i) = Model._min_joint_torque(i) - Model._bg(i);
			//_rub1(i) = Model._max_joint_torque(i) - Model._bg(i);
			_rlb1(i) = Model._min_ctrl_joint_torque(i);
			_rub1(i) = Model._max_ctrl_joint_torque(i);

			//joint velocity limit
			//double k_tanh = 1.0;
			/*
			if (abs(Model._min_joint_velocity(i) - _qdot(i)) <= abs(Model._max_joint_velocity(i) - _qdot(i)))
			{
				if (_qdot(i) > Model._min_joint_velocity(i))
				{
					_rlb1(i) = -(Model._max_joint_torque(i) - Model._min_joint_torque(i)) * tanh(k_tanh * (_qdot(i) - Model._min_joint_velocity(i))) + Model._max_joint_torque(i);
				}
				else
				{
					_rlb1(i) = Model._max_joint_torque(i) - 0.05;
				}
				_rub1(i) = Model._max_joint_torque(i);
			}
			else
			{
				if (_qdot(i) < Model._max_joint_velocity(i))
				{
					_rub1(i) = -(Model._max_joint_torque(i) - Model._min_joint_torque(i)) * tanh(k_tanh * (_qdot(i) - Model._max_joint_velocity(i))) + Model._min_joint_torque(i);
				}
				else
				{
					_rub1(i) = Model._min_joint_torque(i) + 0.05;
				}
				_rlb1(i) = Model._min_joint_torque(i);
			}*/

			//joint position limit
	/*		k_tanh = 5.0;
			double tau_lb_tmp = 0.0;
			double tau_ub_tmp = 0.0;
			if (abs(Model._min_joint_position(i) - _q(i)) <= abs(Model._max_joint_position(i) - _q(i)))
			{
				if (_q(i) > Model._min_joint_position(i))
				{
					tau_lb_tmp = -(Model._max_joint_torque(i) - Model._min_joint_torque(i)) * tanh(k_tanh * (_q(i) - Model._min_joint_position(i))) + Model._max_joint_torque(i);
				}
				else
				{
					tau_lb_tmp = Model._max_joint_torque(i) - 0.05;
				}
				tau_ub_tmp = Model._max_joint_torque(i);
			}
			else
			{
				if (_q(i) < Model._max_joint_position(i))
				{
					tau_ub_tmp = -(Model._max_joint_torque(i) - Model._min_joint_torque(i)) * tanh(k_tanh * (_q(i) - Model._max_joint_position(i))) + Model._min_joint_torque(i);
				}
				else
				{
					tau_ub_tmp = Model._min_joint_torque(i) + 0.05;
				}
				tau_lb_tmp = Model._min_joint_torque(i);
			}
			_rlb1(i) = tau_lb_tmp;
			_rub1(i) = tau_ub_tmp;
			if (tau_lb_tmp > _rlb2(i))
			{
				_rlb2(i) = tau_lb_tmp;
			}
			if (tau_ub_tmp < _rub2(i))
			{
				_rub2(i) = tau_ub_tmp;
			}*/
		}
		_rlb1(0) = 0.0 - threshold;
		_rub1(0) = 0.0 + threshold;
		// NOTE: verify an effect of decreasing the wrist joint inequality constraints
		// _rlb1(6) = -1.0-Model._bg(6);
		// _rub1(6) = 1.0-Model._bg(6);
		// _rlb1(13) = -1.0-Model._bg(13);
		// _rub1(13) = 1.0-Model._bg(13);
		//task limit	
		for (int i = 0; i < 12; i++)
		{
			_rlb1(i + 15) = -1000000.0;
			_rub1(i + 15) = 1000000.0;
		}
		rHQP_P1.UpdateSubjectToX(_rlb1, _rub1);

		//Solve
		rHQP_P1.EnableEqualityCondition(0.0001);
		rHQP_P1.SolveQPoases(max_iter);
		//_torque = rHQP_P1._Xopt.segment(0, 15) + Model._bg;

		//second priority task QP ////////////////////////////////////////////////////////////////////////////////////////////////////////////////	
		//set cost function x^T*H*x + g	
		_rH2.setZero();
		for (int i = 0; i < 15; i++) //torque
		{
			_rH2(i, i) = 1.0;
		}
		for (int i = 15; i < 30; i++) //slack
		{
			_rH2(i, i) = 1000000.0;
		}
		_rH2(15, 15) = 0.0001;
		//_rH2.block<15, 15>(0, 0) = Model._A.inverse();
		//_rH2.block<15, 15>(15, 15) = Model._A;
		_rg2.setZero();
		rHQP_P2.UpdateMinProblem(_rH2, _rg2);

		//set A*x <= b	
		_rA2.setZero();
		_rlbA2.setZero();
		_rubA2.setZero();
		_rA2.block<15, 15>(0, 0) = Model._A.inverse();
		_rA2.block<15, 15>(0, 15) = -_Id_15;
		_rA2.block<12, 15>(15, 0) = J_Ainv;

		VectorXd joint_acc_des(15);
		joint_acc_des.setZero();
		_kdj = 20.0;
		joint_acc_des = -_kdj * _qdot;
		joint_acc_des(0) = 0.0;
		//joint_acc_des(0) = 400.0 * (_q_home(0)- _q(0)) - _kdj * _qdot(0);

		for (int i = 0; i < 15; i++)
		{
			_rlbA2(i) = joint_acc_des(i) - threshold;
			_rubA2(i) = joint_acc_des(i) + threshold;
		}
		for (int i = 0; i < 12; i++)
		{
			_rlbA2(i + 15) = -_Jdot_qdot(i) + _xddot_star(i) + rHQP_P1._Xopt(i + 15) - threshold;
			_rubA2(i + 15) = -_Jdot_qdot(i) + _xddot_star(i) + rHQP_P1._Xopt(i + 15) + threshold;
		}
		rHQP_P2.UpdateSubjectToAx(_rA2, _rlbA2, _rubA2);

		//set lb <= x <= ub
		_rlb2.setZero();
		_rub2.setZero();

		//joint torque limit
		for (int i = 0; i < 15; i++)
		{
			//_rlb2(i) = Model._min_joint_torque(i) - Model._bg(i);
			//_rub2(i) = Model._max_joint_torque(i) - Model._bg(i);
			_rlb2(i) = Model._min_ctrl_joint_torque(i) * 2.0;
			_rub2(i) = Model._max_ctrl_joint_torque(i) * 2.0;

			//double k_tanh = 1.0;
			/*
			//joint velocity limit
			if (abs(Model._min_joint_velocity(i) - _qdot(i)) <= abs(Model._max_joint_velocity(i) - _qdot(i)))
			{
				if (_qdot(i) > Model._min_joint_velocity(i))
				{
					_rlb2(i) = -(Model._max_joint_torque(i) - Model._min_joint_torque(i)) * tanh(k_tanh * (_qdot(i) - Model._min_joint_velocity(i))) + Model._max_joint_torque(i);
				}
				else
				{
					_rlb2(i) = Model._max_joint_torque(i) - 0.05;
				}
				_rub2(i) = Model._max_joint_torque(i);
			}
			else
			{
				if (_qdot(i) < Model._max_joint_velocity(i))
				{
					_rub2(i) = -(Model._max_joint_torque(i) - Model._min_joint_torque(i)) * tanh(k_tanh * (_qdot(i) - Model._max_joint_velocity(i))) + Model._min_joint_torque(i);
				}
				else
				{
					_rub2(i) = Model._min_joint_torque(i) + 0.05;
				}
				_rlb2(i) = Model._min_joint_torque(i);
			}*/

			//joint position limit
	/*		k_tanh = 5.0;
			double tau_lb_tmp = 0.0;
			double tau_ub_tmp = 0.0;
			if(abs(Model._min_joint_position(i) - _q(i)) <= abs(Model._max_joint_position(i) - _q(i)))
			{
			  if(_q(i) > Model._min_joint_position(i))
			  {
				  tau_lb_tmp = -(Model._max_joint_torque(i) - Model._min_joint_torque(i)) * tanh(k_tanh * (_q(i) - Model._min_joint_position(i))) + Model._max_joint_torque(i);
			  }
			  else
			  {
				  tau_lb_tmp = Model._max_joint_torque(i) - 0.05;
			  }
			  tau_ub_tmp = Model._max_joint_torque(i);
			}
			else
			{
			  if(_q(i) < Model._max_joint_position(i))
			  {
				  tau_ub_tmp = -(Model._max_joint_torque(i)- Model._min_joint_torque(i)) *tanh(k_tanh*(_q(i)- Model._max_joint_position(i))) + Model._min_joint_torque(i);
			  }
			  else
			  {
				  tau_ub_tmp = Model._min_joint_torque(i) + 0.05;
			  }
			  tau_lb_tmp = Model._min_joint_torque(i);
			}
			_rlb2(i) = tau_lb_tmp;
			_rub2(i) = tau_ub_tmp;
			if (tau_lb_tmp > _rlb2(i))
			{
				_rlb2(i) = tau_lb_tmp;
			}
			if (tau_ub_tmp < _rub2(i))
			{
				_rub2(i) = tau_ub_tmp;
			}*/
		}
		_rlb2(0) = 0.0 - threshold;
		_rub2(0) = 0.0 + threshold;
		// NOTE: verify an effect of decreasing the wrist joint inequality constraints
		// _rlb2(6) = -1.0-Model._bg(6);
		// _rub2(6) = 1.0-Model._bg(6);
		// _rlb2(13) = -1.0-Model._bg(13);
		// _rub2(13) = 1.0-Model._bg(13);	
		//task limit
		for (int i = 0; i < 15; i++)
		{
			_rlb2(i + 15) = -1000000.0;
			_rub2(i + 15) = 1000000.0;
		}
		rHQP_P2.UpdateSubjectToX(_rlb2, _rub2);

		//Solve
		//rHQP_P2.EnableEqualityCondition(0.0001);
		rHQP_P2.SolveQPoases(max_iter);

		if (rHQP_P1._num_state == 0 || rHQP_P2._num_state == 0)
		{
			_torque = rHQP_P2._Xopt.segment(0, 15) + Model._bg;
			//cout << "------------- torque ---------------------\n" << rHQP_P2._Xopt.segment(0, 15). transpose() + Model._bg.transpose() << endl;
		}
		else //when solving HQP failed
		{
			_bool_safemode = true;

			cout << "Fault: Cannot solve QP!!" << endl << endl;
		}
	}

}

void CController::safeModeReplaceTorque(bool bool_safemode)
{
	if (bool_safemode == true)
	{
		_torque.setZero();
		_torque = Model._A *0.5* _kdj * ( - _qdot) + Model._bg;
	}
}

void CController::safeWorkSpaceLimit()
{
	_dist_shoulder_hand_left = sqrt((Model._x_left_shoulder(0) - Model._x_left_hand(0)) * (Model._x_left_shoulder(0) - Model._x_left_hand(0)) + (Model._x_left_shoulder(1) - Model._x_left_hand(1)) * (Model._x_left_shoulder(1) - Model._x_left_hand(1)) + (Model._x_left_shoulder(2) - Model._x_left_hand(2)) * (Model._x_left_shoulder(2) - Model._x_left_hand(2)));
	_dist_shoulder_hand_right = sqrt((Model._x_right_shoulder(0) - Model._x_right_hand(0)) * (Model._x_right_shoulder(0) - Model._x_right_hand(0)) + (Model._x_right_shoulder(1) - Model._x_right_hand(1)) * (Model._x_right_shoulder(1) - Model._x_right_hand(1)) + (Model._x_right_shoulder(2) - Model._x_right_hand(2)) * (Model._x_right_shoulder(2) - Model._x_right_hand(2)));

	//cout << _dist_shoulder_hand_left << "  " << _dist_shoulder_hand_right << endl;

	_dir_hand_to_shoulder_left = (Model._x_left_hand - Model._x_left_shoulder)/ _dist_shoulder_hand_left;
	_dir_hand_to_shoulder_right = (Model._x_right_hand - Model._x_right_shoulder) / _dist_shoulder_hand_right;

	_workspace_avoid_gain = 10.0;	
	double dist_boundary = 0.6;
	double dist_margin = 0.03;
	double alpha = 0.0;
	alpha = CustomMath::SwitchFunction(dist_boundary, dist_boundary + dist_margin, _dist_shoulder_hand_left);

	if (_dist_shoulder_hand_left >= dist_boundary)
	{
		_acc_workspace_avoid_left = alpha* _workspace_avoid_gain* _dir_hand_to_shoulder_left*(dist_boundary - _dist_shoulder_hand_left) - alpha* _xddot_star.segment(0, 3);//
	}
	else
	{
		_acc_workspace_avoid_left.setZero();
	}

	if (_dist_shoulder_hand_right >= dist_boundary)
	{
		_acc_workspace_avoid_right = alpha * _workspace_avoid_gain * _dir_hand_to_shoulder_right * (dist_boundary - _dist_shoulder_hand_right) - alpha * _xddot_star.segment(6, 3);//
	}
	else
	{
		_acc_workspace_avoid_right.setZero();
	}
	
}

void CController::Initialize()
{
	_control_mode = 1; //1: joint space, 2: operational space

	_bool_init = true;

	_t = 0.0;
	_init_t = 0.0;

	_pre_t = 0.0;
	_dt = 0.0;

	_q.setZero(_dofj);
	_qdot.setZero(_dofj);
	_torque.setZero(_dofj);
	_torque_a.setZero(14);

	_pre_q.setZero(_dofj);
	_pre_qdot.setZero(_dofj);

	_q_home.setZero(_dofj);
	_q_home(0) = 0.0;
	_q_home(1) = 30.0 * DEG2RAD; //LShP
	_q_home(8) = -30.0 * DEG2RAD; //RShP
	_q_home(2) = 20.0 * DEG2RAD; //LShR
	_q_home(9) = -20.0 * DEG2RAD; //RShR
	_q_home(4) = 80.0 * DEG2RAD; //LElP
	_q_home(11) = -80.0 * DEG2RAD; //RElP
	_q_home(6) = -30.0 * DEG2RAD; //LWrP
	_q_home(13) = 30.0 * DEG2RAD; //RWrP

	// _q_limit.setZero(_dofj); // TEST(!!!!!!!!!!!!!!!!!!!!!!!!!)
	// _q_limit(0) = 0.0;			 // Body height
	// _q_limit(1) = -90.0 * DEG2RAD; // LShP
	// _q_limit(2) = 0.0 * DEG2RAD; // LShR
	// _q_limit(3) = 0.0 * DEG2RAD; // LShY
	// _q_limit(4) = 0.0 * DEG2RAD; // LElP
	// _q_limit(5) = 0.0 * DEG2RAD; // LWrY
	// _q_limit(6) = 0.0 * DEG2RAD; // LWrP
	// _q_limit(7) = 0.0 * DEG2RAD; // LWrR

	// _q_limit(8) = 90.0 * DEG2RAD; // RShP
	// _q_limit(9) = 0.0 * DEG2RAD; // RShR
	// _q_limit(10) = 0.0 * DEG2RAD; // RShY
	// _q_limit(11) = 0.0 * DEG2RAD; // RElP
	// _q_limit(12) = 0.0 * DEG2RAD; // RWrY
	// _q_limit(13) = 0.0 * DEG2RAD; // RWrP
	// _q_limit(14) = 0.0 * DEG2RAD; // RWrR
	_start_time = 0.0;
	_end_time = 0.0;
	_motion_time = 0.0;

	_q_des.setZero(_dofj);
	_qdot_des.setZero(_dofj);
	_q_goal.setZero(_dofj);
	_qdot_goal.setZero(_dofj);

	_x_left_hand.setZero(6);
	_x_right_hand.setZero(6);
	_xdot_left_hand.setZero(6);
	_xdot_right_hand.setZero(6);

	_x_goal_left_hand.setZero(6);
	_xdot_goal_left_hand.setZero(6);
	_x_des_left_hand.setZero(6);
	_xdot_des_left_hand.setZero(6);
	_x_goal_right_hand.setZero(6);
	_xdot_goal_right_hand.setZero(6);
	_x_des_right_hand.setZero(6);
	_xdot_des_right_hand.setZero(6);

	_R_des_left_hand.setZero();
	_R_des_right_hand.setZero();

	_xddot_star.setZero(12);
	_x_err_left_hand.setZero();
	_x_err_right_hand.setZero();
	_xdot_err_left_hand.setZero();
	_xdot_err_right_hand.setZero();
	_R_err_left_hand.setZero();
	_R_err_right_hand.setZero();
	_Rdot_err_left_hand.setZero();
	_Rdot_err_right_hand.setZero();

	_cnt_plan = 0;
	_bool_plan.setZero(20);
	_time_plan.resize(100);
	_time_plan.setConstant(5.0);
	reset_target(_time_plan(_cnt_plan), _q_home);

	_kpj = 400.0;
	_kdj = 40.0;
	_kp = 400.0;
	_kd = 40.0;

	_J_hands.setZero(12, 15);
	_J_hands_prism.setZero(1, 15);
	_J_hands_prism.col(0).setOnes(); // !!!!!!!!!!!!!!!!!!
	_J_K_hands.setZero(12, 14);
	_J_K_T_hands.setZero(14, 12);
	_J_Winv_hands.setZero(15, 12);
	_J_Winv_T_hands.setZero(12, 15);
	_Jdot_hands.setZero(12, 15);
	_Jdot_qdot.setZero(12);
	_pre_J_hands.setZero(12, 15);
	_pre_Jdot_hands.setZero(12, 15);
	_J_T_hands.setZero(15, 12);
	_J_T_hands_prism.setZero(15, 1);
	_J_T_hands_prism.row(0).setOnes();

	_Lambda_hands.setZero(12, 12);
	_Lambda_hands_prism.setZero(1, 1);
	_Null_hands.setZero(15, 15);
	_Null_hands_prism.setZero(15, 15); //!!!!!!!!!!!!!!!!!!
	_Null_hands_K.setZero(14, 14);
	_J_bar_T_hands.setZero(12, 15);
	_S_hands.setZero(14, 15);
	_S_T_hands.setZero(15, 14);

	_J_pos_hands.setZero(6, 15);
	_J_pos_T_hands.setZero(15, 6);
	_J_ori_hands.setZero(6, 15);
	_J_ori_T_hands.setZero(15, 6);
	_Lambda_pos_hands.setZero(6, 6);
	_Lambda_ori_hands.setZero(6, 6);
	_Null_hands_pos.setZero(15, 15);
	_Null_hands_ori.setZero(15, 15);

	_pos_goal_left_hand.setZero();
	_rpy_goal_left_hand.setZero();
	_pos_goal_right_hand.setZero();
	_rpy_goal_right_hand.setZero();

	_S_T.setZero(15, 6);
	_J_bar_T_hands_S_T.setZero(12,6);
	_W_mat_S.setZero(6, 6);
	_J_tilde_T.setZero(6,12);
	_xddot_reinforce.setZero(12);
	_torque_reinforce.setZero(15);

	_Id_15.setIdentity(15, 15);
	_Id_14.setIdentity(14, 14);
	_Id_12.setIdentity(12, 12);

	_S_hands.block<14, 14>(0, 1) = _Id_14;
	_S_T_hands = _S_hands.transpose();

	_bool_joint_motion = false;
	_bool_ee_motion = false;

	_zero_vec.setZero(_dofj);

	JointTrajectory.set_size(15);
	LeftHandTrajectory.set_size(6);
	RightHandTrajectory.set_size(6);

	//HQP
	HQP_P1.InitializeProblemSize(42,27); //variable size = (joint dof)*2+(task dof), constraint size = (joint dof) + (task dof) 
	_H1.setZero(HQP_P1._num_var, HQP_P1._num_var);
	_g1.setZero(HQP_P1._num_var);
	_A1.setZero(HQP_P1._num_cons, HQP_P1._num_var);
	_lbA1.setZero(HQP_P1._num_cons);
	_ubA1.setZero(HQP_P1._num_cons);
	_lb1.setZero(HQP_P1._num_var);
	_ub1.setZero(HQP_P1._num_var);
	HQP_P2.InitializeProblemSize(45,42); //variable size = (joint dof)*2+(task dof), constraint size = (joint dof) + (1st prioirty task dof)  + (2nd prioirty task dof) 
	_H2.setZero(HQP_P2._num_var, HQP_P2._num_var);
	_g2.setZero(HQP_P2._num_var);
	_A2.setZero(HQP_P2._num_cons, HQP_P2._num_var);
	_lbA2.setZero(HQP_P2._num_cons);
	_ubA2.setZero(HQP_P2._num_cons);
	_lb2.setZero(HQP_P2._num_var);
	_ub2.setZero(HQP_P2._num_var);


	//rHQP
	rHQP_P1.InitializeProblemSize(27, 12); //variable size = (joint dof)+(task dof), constraint size =(task dof) 
	_rH1.setZero(rHQP_P1._num_var, rHQP_P1._num_var);
	_rg1.setZero(rHQP_P1._num_var);
	_rA1.setZero(rHQP_P1._num_cons, rHQP_P1._num_var);
	_rlbA1.setZero(rHQP_P1._num_cons);
	_rubA1.setZero(rHQP_P1._num_cons);
	_rlb1.setZero(rHQP_P1._num_var);
	_rub1.setZero(rHQP_P1._num_var);
	rHQP_P2.InitializeProblemSize(30, 27); //variable size = (joint dof)+(task dof), constraint size =(1st prioirty task dof)  + (2nd prioirty task dof) 
	_rH2.setZero(rHQP_P2._num_var, rHQP_P2._num_var);
	_rg2.setZero(rHQP_P2._num_var);
	_rA2.setZero(rHQP_P2._num_cons, rHQP_P2._num_var);
	_rlbA2.setZero(rHQP_P2._num_cons);
	_rubA2.setZero(rHQP_P2._num_cons);
	_rlb2.setZero(rHQP_P2._num_var);
	_rub2.setZero(rHQP_P2._num_var);

	_bool_safemode = false;
	_dist_shoulder_hand_left = 0.0;
	_dist_shoulder_hand_right = 0.0;
	_workspace_avoid_gain = 0.0;
	_dir_hand_to_shoulder_left.setZero();
	_dir_hand_to_shoulder_right.setZero();
	_acc_workspace_avoid_left.setZero();
	_acc_workspace_avoid_right.setZero();

	// ROS
	_joy_vec_command.setZero(19);

	// Joystick
	_joy_button = "N/A";
	_cnt_joy_mode = 0;
	_bool_joy_button_push = false;
	_bool_joy_plan.setZero(100);
	for (int i = 10; i < 100; i++)
	{
		_time_plan(i) = 3.0;
	}

	// Quaternion
	_ka = 400;
	_ko = 40;
	_q_left_hand.vec().setZero();
	_q_right_hand.vec().setZero();
	_q_left_hand.w() = 0.0;
	_q_right_hand.w() = 0.0;
	_pre_q_left_hand.vec().setZero();
	_pre_q_right_hand.vec().setZero();
	_pre_q_left_hand.w() = 0.0;
	_pre_q_right_hand.w() = 0.0;
	_qdot_left_hand.vec().setZero();
	_qdot_right_hand.vec().setZero();
	_qdot_left_hand.w() = 0.0;
	_qdot_right_hand.w() = 0.0;
	_pre_qdot_left_hand.vec().setZero();
	_pre_qdot_right_hand.vec().setZero();
	_pre_qdot_left_hand.w() = 0.0;
	_pre_qdot_right_hand.w() = 0.0;

	_vec_qdot_left_hand.setZero();
	_vec_qdot_right_hand.setZero();

	_q_goal_left_hand.vec().setZero();
	_q_goal_right_hand.vec().setZero();
	_q_goal_left_hand.w() = 0.0;
	_q_goal_right_hand.w() = 0.0;
	_q_des_vec_left_hand.setZero();
	_q_des_vec_right_hand.setZero();
	_q_des_scalar_left_hand = 0.0;
	_q_des_scalar_right_hand = 0.0;
	_q_des_skew_left_hand.setZero();
	_q_des_skew_right_hand.setZero();
	_q_err_left_hand.setZero();
	_q_err_right_hand.setZero();

	// Orientation 
	_w_left_hand.setZero();
	_w_right_hand.setZero();
	_wdot_left_hand.setZero();
	_wdot_right_hand.setZero();
	_pre_w_left_hand.setZero();
	_pre_w_right_hand.setZero();
	_pre_wdot_left_hand.setZero();
	_pre_wdot_right_hand.setZero();
	_w_des_left_hand.setZero();
	_w_des_right_hand.setZero();

	_global_rotate.setZero(3, 3);
	_global_rotate.setIdentity(); //x방향이 앞뒤 방향이 되도록 변경하기 위하여...	
	_global_rotate = CustomMath::GetBodyRotationMatrix(0.0, 0.0, -90.0 * DEG2RAD);

	// File stream
	_idx = 0;
	_log_sim_torque.open("/home/kist/Octave/Dual-arm/Log/sim_torque.txt");
	_log_com_torque.open("/home/kist/Octave/Dual-arm/Log/com_torque.txt");
	_log_sim_task.open("/home/kist/Octave/Dual-arm/Log/sim_task.txt");
	_log_sim_task << "left x  " << "\t" << "left y  " << "\t" << "left z  " << "\t" << "left xd " << "\t" << "left yd " << "\t" << "left zd " << "\t" << "right x " << "\t" << "right y " << "\t" << "right z " << "\t" << "right xd" << "\t" << "right yd" << "\t" << "right zd" << endl;
	//remove("/home/kist/Octave/Dual-arm/Dataset/OSF.txt");
	//_ofs_OSF.open("/home/kist/Octave/Dual-arm/Dataset/OSF.txt");
	// remove("/home/kist/Octave/Dual-arm/Dataset/HQP.txt");
	// _ofs_HQP.open("/home/kist/Octave/Dual-arm/Dataset/HQP.txt");
	// remove("/home/kist/Octave/Dual-arm/Dataset/RHQP.txt");
	// _ofs_RHQP.open("/home/kist/Octave/Dual-arm/Dataset/RHQP.txt");
}
