#include "pch.h"
#include "RigidBody.h"


namespace dynamics {
	RigidBody::RigidBody()
	{
	}

	RigidBody::RigidBody(const RigidBodyParameter & parameter, const RigidBodyCondition & IC)
	{
		system_info.type = continous_RIGIDBODY;
		system_info.category = DYNAMICS;
		system_info.num_of_continuous_states = 13;// 3 for V_I, 3 for Omega_BI, 3 for X_I, 4 for q
		system_info.num_of_inputs = 6;// 0-2 F_B 3-6 M_B
		// loading initial condition:
		state.resize(system_info.num_of_continuous_states);
		state.segment(0, 3) = IC.V_I;
		state.segment(3, 3) = IC.Omega_BI;
		state.segment(6, 3) = IC.X_I;
		state.segment(9, 4) = mathauxiliary::GetQuaterionFromRulerAngle(IC.Euler);
		// loading parameters:
		J = parameter.J;
		m = parameter.m;
		J_inv = parameter.J.inverse();
		// output mapping: 0-2 V_I 3-5 X_I 6-8 Omega_BI 9-17 R_IB
		system_info.DIRECT_FEED_THROUGH = false;
		system_info.input_connection.resize(6, 2);
		system_info.num_of_outputs = 21;
		system_info.system_parameter_ok = true;
		output.resize(system_info.num_of_outputs);
		output.setZero();
	}

	void RigidBody::DifferentialEquation(const double & t, const VectorXd & state_temp, const VectorXd & input, VectorXd & derivative)
	{
		// state allocation:
		/*
		state(0,1,2) = V_I
		state(3,4,5) = omega_BI
		state(6,7,8) = X_I
		state(9,0,11,12) = quaternion
		*/
		Vector4d q = state_temp.segment(9, 4);

		derivative.segment(0, 3) = mathauxiliary::GetR_IBFromQuaterion(q)*input.segment(0, 3) / m;// dot_VI = R_IB F_B/m
		derivative.segment(3, 3) = J_inv * (input.segment(3, 3) - mathauxiliary::Hatmap(state_temp.segment(3, 3))*J*state_temp.segment(3, 3)); // dot_omega = J_inv *( M - omega^x J omega) 
		derivative.segment(6, 3) = state_temp.segment(0, 3);
		derivative.segment(9, 4) = 0.5*mathauxiliary::GetLmatrixFromQuaterion(q.normalized()).transpose()*state_temp.segment(3, 3);
		/*
		std::cout << " the force input is : " << std::endl;
		std::cout << input.segment(0, 3) << std::endl;
		std::cout << " the moment input is : " << std::endl;
		std::cout << input.segment(3, 3) << std::endl;
		std::cout << " the acc is : " << derivative.segment(0, 3) << std::endl;
		std::cout << " the angular acc is " << derivative.segment(6, 3) << std::endl;
		std::cout << "The rotation matrix is: " << std::endl;
		std::cout << mathauxiliary::GetR_IBFromQuaterion(q) << std::endl;*/

	}

	void RigidBody::OutputEquation(const double& t, const VectorXd & state_temp, const VectorXd & input, VectorXd & output_temp)
	{
		// output allocation:
		/*
		output(0,1,2) = V_I
		output(3,4,5) = omega_BI
		output(6,7,8) = X_I
		output(9,10,11)  = R_IB(:,1)
		output(12,13,14) = R_IB(:,2)
		output(15,16,17) = R_IB(:,3)
		output(18,19,20) = V_B
		*/
		output_temp.segment(0, 3) = state_temp.segment(0, 3);
		output_temp.segment(3, 3) = state_temp.segment(3, 3);
		output_temp.segment(6, 3) = state_temp.segment(6, 3);
		//std::cout << " the XI output is : " << output_temp.segment(6, 3) << std::endl;
		// calculate rotation matrix
		Vector4d q = state_temp.segment(9, 4);
		Matrix3d R_IB = mathauxiliary::GetRmatrixFromQuaterion(q.normalized())*mathauxiliary::GetLmatrixFromQuaterion(q.normalized()).transpose();
		output_temp.segment(9, 9) = mathauxiliary::ConvertRotationMatrixToVector(R_IB);
		output_temp.segment(18, 3) = R_IB.transpose()*state_temp.segment(0, 3);// R_BI*V_I
	}

	void RigidBody::DisplayParameters()
	{
		std::cout << "---------------------" << std::endl;
		std::cout << "rigidbody parameter:" << std::endl;
		std::cout << "J = " << std::endl << J << std::endl;
		std::cout << "m = " << std::endl << m << std::endl;
	}

	void RigidBody::DisplayInitialCondition()
	{
		std::cout << "---------------------" << std::endl;
		std::cout << "Rigidbody initial condition X_0:" << std::endl;
		std::cout << "The initial condition is:  " << std::endl;
		std::cout << state << std::endl;
	}

	void RigidBody::IncrementState()
	{
		// state allocation:
		/*
		state(0,1,2) = V_I
		state(3,4,5) = omega_BI
		state(6,7,8) = X_I
		state(9,0,11,12) = quaternion
		*/
		state.segment(0, 9) += solver_buffer_state_increment1.segment(0, 9);
		state.segment(9, 4) += solver_buffer_state_increment1.segment(9, 4);
		state.segment(9, 4).normalize();// normalize the quaternion
	}

	RigidBody::~RigidBody()
	{
	}
}