#include "pch.h"
#include "RigidBody.h"


namespace dynamics {

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
		system_info.NO_CONTINUOUS_STATE = false;
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
		q = state_temp.segment(9, 4);

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
		q = state_temp.segment(9, 4);
		R_IB = mathauxiliary::GetRmatrixFromQuaterion(q.normalized())*mathauxiliary::GetLmatrixFromQuaterion(q.normalized()).transpose();
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
	/*-----------------------------------------------------------------------------------------*/
	// rigid body kinematics
	RigidBodyKinematics::RigidBodyKinematics(const RigidBodyKinematicsInitialCondition& IC)
	{
		system_info.type = continous_RIGIDKINEMATICS;
		system_info.category = DYNAMICS;
		system_info.num_of_continuous_states = 13;// 3 for V_I, 3 for Omega_BI, 3 for X_I, 4 for q
		system_info.num_of_inputs = 6;// 
		// loading initial condition:
		InitialCondition = IC;
		state.resize(system_info.num_of_continuous_states);
		state.segment(KINEMATICS_STATE_VIx, 3) = IC.VI0;
		state.segment(KINEMATICS_STATE_OmegaBIx, 3) = IC.Omega0;
		state.segment(KINEMATICS_STATE_XIx, 3) = IC.XI0;
		state.segment(KINEMATICS_STATE_q0, 4) = mathauxiliary::GetQuaterionFromRulerAngle(IC.Euler0);

		// output mapping: 0-2 V_I 3-5 X_I 6-8 Omega_BI 9-17 R_IB
		system_info.DIRECT_FEED_THROUGH = false;
		system_info.input_connection.resize(6, 2);
		system_info.input_connection.setZero();
		system_info.num_of_outputs = 36;
		system_info.system_parameter_ok = true;
		system_info.NO_CONTINUOUS_STATE = false;
		output.resize(system_info.num_of_outputs);
		output.setZero();
	}

	void RigidBodyKinematics::DifferentialEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & derivative)
	{
		// dot_VI = AI
		derivative.segment(KINEMATICS_STATE_VIx, 3) = input.segment(KINEMATICS_INPUT_AIx, 3);
		// dot_Omega = Omega_DOT
		derivative.segment(KINEMATICS_STATE_OmegaBIx, 3) = input.segment(KINEMATICS_INPUT_OMEGA_DOTx, 3);
		// dot_XI = VI
		derivative.segment(KINEMATICS_STATE_XIx, 3) = state.segment(KINEMATICS_STATE_VIx, 3);
		// dot_q = 0.5 * L^T * omega
		derivative.segment(KINEMATICS_STATE_q0, 4) = 0.5*mathauxiliary::GetLmatrixFromQuaterion(state.segment(KINEMATICS_STATE_q0, 4).normalized()).transpose()*state.segment(KINEMATICS_STATE_OmegaBIx, 3);
	}
	void RigidBodyKinematics::OutputEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & output)
	{
		// calculate the rotation matrix first
		R_IB = mathauxiliary::GetRmatrixFromQuaterion(state.segment(KINEMATICS_STATE_q0, 4).normalized())*mathauxiliary::GetLmatrixFromQuaterion(state.segment(KINEMATICS_STATE_q0, 4).normalized()).transpose();
		output.segment(KINEMATICS_OUTPUT_VIx, 3) = state.segment(KINEMATICS_STATE_VIx, 3);
		output.segment(KINEMATICS_OUTPUT_VBx, 3) = R_IB.transpose() * output.segment(KINEMATICS_OUTPUT_VIx, 3);
		output.segment(KINEMATICS_OUTPUT_OmegaBIx, 3) = state.segment(KINEMATICS_STATE_OmegaBIx, 3);
		output.segment(KINEMATICS_OUTPUT_XIx, 3) = state.segment(KINEMATICS_STATE_XIx, 3);
		output.segment(KINEMATICS_OUTPUT_EulerRoll,3) = mathauxiliary::GetEulerAngleFromQuaterion(state.segment(KINEMATICS_STATE_q0, 4));
		output.segment(KINEMATICS_OUTPUT_R_IB00,9) = mathauxiliary::ConvertRotationMatrixToVector(R_IB);
		output.segment(KINEMATICS_OUTPUT_R_BI00,9) = mathauxiliary::ConvertRotationMatrixToVector(R_IB.transpose());
		// calculate euler angle rate. reference: http://www.stengel.mycpanel.princeton.edu/Quaternions.pdf
		sin_phi = sin(output(KINEMATICS_OUTPUT_EulerRoll));
		cos_phi = cos(output(KINEMATICS_OUTPUT_EulerRoll));
		if (abs(abs(output(KINEMATICS_OUTPUT_EulerPitch)) - M_PI / 2.0) < 0.0087) { // 0.5 deg in rad
			if (output(KINEMATICS_OUTPUT_EulerPitch) > 0) {
				tan_theta = 113.09; // tan(89.5 deg)
			} else {
				tan_theta = - 113.09; // tan(89.5 deg)
			}
			sec_theta = 113.09; // sec(89.5 deg)
		} else {
			tan_theta = tan(output(KINEMATICS_OUTPUT_EulerPitch));
			sec_theta = 1.0 / cos(output(KINEMATICS_OUTPUT_EulerPitch));
		}

		// q cos phi  - r sin phi = theta_dot
		output(KINEMATICS_OUTPUT_THETADOT) = output(KINEMATICS_OUTPUT_OmegaBIy) * cos_phi - output(KINEMATICS_OUTPUT_OmegaBIz) * sin_phi;
		// p + q sin phi tan theta + r cos phi tan theta
		output(KINEMATICS_OUTPUT_PHIDOT) = output(KINEMATICS_OUTPUT_OmegaBIx) + sin_phi * tan_theta * output(KINEMATICS_OUTPUT_OmegaBIy) + cos_phi * tan_theta * output(KINEMATICS_OUTPUT_OmegaBIz);
		// q sin phi sec theta + r cos phi sec theta
		output(KINEMATICS_OUTPUT_PSIDOT) = output(KINEMATICS_OUTPUT_OmegaBIy) *sin_phi * sec_theta + cos_phi * sec_theta * output(KINEMATICS_OUTPUT_OmegaBIz);
	}
	void RigidBodyKinematics::DisplayParameters()
	{
		std::cout << "-------NO parameters for kinematics block-------" << std::endl;
	}
	void RigidBodyKinematics::DisplayInitialCondition()
	{
		std::cout << "---------------------" << std::endl;
		std::cout << "Initial Velocity: " << InitialCondition.VI0 << std::endl;
		std::cout << "Initial Position: " << InitialCondition.XI0 << std::endl;
		std::cout << "Initial Omega: " << InitialCondition.Omega0<< std::endl;
		std::cout << "Initial Euler: " << InitialCondition.Euler0 << std::endl;
	}
	void RigidBodyKinematics::IncrementState()
	{
		/*
		state(0,1,2) = V_I
		state(3,4,5) = omega_BI
		state(6,7,8) = X_I
		state(9,0,11,12) = quaternion
		*/
		state.segment(0, 9) += solver_buffer_state_increment1.segment(0, 9);
		state.segment(KINEMATICS_STATE_q0, 4) += solver_buffer_state_increment1.segment(KINEMATICS_STATE_q0, 4);
		state.segment(KINEMATICS_STATE_q0, 4).normalize();// normalize the quaternion
	}

	RigidBodyKinematics::~RigidBodyKinematics()
	{
	}

	/*-----------------------------------------------------------------------------------------*/
	// rigid body dynamics
	RigidBodyDynamics::RigidBodyDynamics(const RigidBodyDynamicsParamter & parameter)
	{
		system_info.type = continous_RIGIDDYNAMICS;
		system_info.category = DYNAMICS;
		system_info.num_of_continuous_states = 0;
		system_info.num_of_inputs = 9; // 0-2 F_B 3-5 M_B 6-8 omega_b
		// loading parameters:
		J = parameter.J;
		m = parameter.m;
		J_inv = parameter.J.inverse();
		// output mapping: 0-2 A_I, 3 - 5 , Omega_DOT
		system_info.DIRECT_FEED_THROUGH = true;
		system_info.NO_CONTINUOUS_STATE = true;
		system_info.input_connection.resize(6, 2);
		system_info.input_connection.setZero();
		system_info.num_of_outputs = 24;
		system_info.system_parameter_ok = true;
		output.resize(system_info.num_of_outputs);
		output.setZero();

	}

	void RigidBodyDynamics::DifferentialEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & derivative)
	{
		// No differential equations for rigid dynamics
	}

	void RigidBodyDynamics::OutputEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & output)
	{
		output.segment(DYNAMICS_OUTPUT_AIx, 3) = input.segment(DYNAMICS_INPUT_FIx, 3) / m;
		output.segment(DYNAMICS_OUTPUT_OMEGA_DOTx, 3) = J_inv * (input.segment(DYNAMICS_INPUT_TBx, 3) 
												- mathauxiliary::Hatmap(input.segment(DYNAMICS_INPUT_OmegaBIx, 3))*J*input.segment(DYNAMICS_INPUT_OmegaBIx, 3)); 
		// dot_omega = J_inv *( M - omega^x J omega) 
	}

	void RigidBodyDynamics::DisplayParameters()
	{
		std::cout << "---------------------" << std::endl;
		std::cout << "Mass: " << m << std::endl;
		std::cout << "Moment of inertia: " << std::endl;
		std::cout << J << std::endl;
	}

	void RigidBodyDynamics::DisplayInitialCondition()
	{
		std::cout << "------- No initial condition for rigid dynamics -------" << std::endl;
	}

	void RigidBodyDynamics::IncrementState()
	{
		// no increment state for rigid dynamics
	}

	RigidBodyDynamics::~RigidBodyDynamics()
	{
	}
}