#include "pch.h"
#include "GroundContact.h"

groundcontact::SimpleGearNormalForce::SimpleGearNormalForce(const SimpleGearNormalForceParameter & param)
{
	system_info.type = groundcontact_SIMPLENORMAL;
	system_info.category = GROUNDCONTACT;
	system_info.num_of_continuous_states = 0;
	system_info.num_of_inputs = 25;
	system_info.DIRECT_FEED_THROUGH = true;
	system_info.input_connection.resize(6, 2);
	system_info.num_of_outputs = 23;
	system_info.system_parameter_ok = true;
	system_info.NO_CONTINUOUS_STATE = true;
	param_ = param;
	// load the direction of wheel
	nwb.setZero();
	nwb(0) = 1.0;
	npb.setZero();
	npb(1) = 1.0;
	Identity3 = Eigen::MatrixXd::Identity(3, 3);
	R_WB = Eigen::MatrixXd::Identity(3, 3); // initialize the steering matrix to an identity matrix
	// initialized the output vector
	output.resize(system_info.num_of_outputs);
	output.setZero();
}

void groundcontact::SimpleGearNormalForce::DifferentialEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & derivative)
{
	// non differential functions for normal force block
}

void groundcontact::SimpleGearNormalForce::OutputEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & output)
{
	output.setZero();
	if ((input(GEARNORMAL_INPUT_SWITCH) > 0.0) && ( input(GEARNORMAL_INPUT_PIz) - input(GEARNORMAL_INPUT_H) + param_.MaxHeight >0.0)) {
		// check whether the aircraft is under the height threshold
		mathauxiliary::ConvertVectorToRotationMatrix(input.segment(GEARNORMAL_INPUT_R_IB00, 9), R_IB);
		// calculate the direction of the gear compression
		ne = R_IB * param_.GearDirection;
		// check whether the direction of the gear compression is facing downwards.
		if (ne(mathauxiliary::VECTOR_Z) > param_.MinNz) {
			// calculate the relate of change of the rotation matirx
			R_IB_dot = R_IB* mathauxiliary::Hatmap(input.segment(GEARNORMAL_INPUT_OMEGAbx, 3));
			// calculate the natural position of the gear in inertial frame
			re = input.segment(GEARNORMAL_INPUT_PIx, 3) + R_IB * param_.GearPosition;
			// calculate the speed of the the natural position of the gear in inertial frame
			re_dot = input.segment(GEARNORMAL_INPUT_VIx, 3) + R_IB_dot * param_.GearPosition;
			// calculate the rotation speed of the gear direction
			ne_dot = R_IB_dot * param_.GearDirection;
			// calculate the compression distance
			output(GEARNORMAL_OUTPUT_COMPRESSION) = (input(GEARNORMAL_INPUT_H) - re(mathauxiliary::VECTOR_Z)) / ne(mathauxiliary::VECTOR_Z); 
			// calculate the wheel longtitudinal and lateral vector
			NGG_2 = Identity3 - input.segment(GEARNORMAL_INPUT_NGx, 3)* input.segment(GEARNORMAL_INPUT_NGx, 3).transpose();
			if (param_.isSteering) {// if steering is enabled, update the rotation matrix
				mathauxiliary::RotationMatrixFromYaw(input(GEARNORMAL_INPUT_STEERING)/57.3, R_WB);
			}
			output.segment(GEARNORMAL_OUTPUT_nwIx, 3) = (NGG_2 * R_IB * R_WB.transpose() * nwb).normalized(); // 
			output.segment(GEARNORMAL_OUTPUT_npIx, 3) = (NGG_2 * R_IB * R_WB.transpose() * npb).normalized(); // 

			output.segment(GEARNORMAL_OUTPUT_CONTACTPOINTx, 3) = R_IB * (output(GEARNORMAL_OUTPUT_COMPRESSION)*param_.GearDirection + param_.GearPosition);
			output(GEARNORMAL_OUTPUT_COMPRESSIONRATE) = -(ne_dot(mathauxiliary::VECTOR_Z) * output(GEARNORMAL_OUTPUT_COMPRESSION) - input(GEARNORMAL_INPUT_Hdot) + re_dot(mathauxiliary::VECTOR_Z)) / ne(mathauxiliary::VECTOR_Z);
			// calculate the speed of the contact point relative to the ground expressed in the body-fixed frame
			VgI = re_dot + output(GEARNORMAL_OUTPUT_COMPRESSIONRATE) * ne + output(GEARNORMAL_OUTPUT_COMPRESSION)* ne_dot;
			// calculate the wheel longitudinal and laternal speed.
			output(GEARNORMAL_OUTPUT_VGIw) = VgI.transpose()* output.segment(GEARNORMAL_OUTPUT_nwIx, 3);
			output(GEARNORMAL_OUTPUT_VGIp) = VgI.transpose()* output.segment(GEARNORMAL_OUTPUT_npIx, 3);
			// 
			if (output(GEARNORMAL_OUTPUT_COMPRESSION) < 0.0) { // if the compression distance is negative, the tie touches the ground
				if (output(GEARNORMAL_OUTPUT_COMPRESSIONRATE) < 0.0) { // compression speed is negative means that suspension is compressing
					output(GEARNORMAL_OUTPUT_F) =  param_.kCompress * output(GEARNORMAL_OUTPUT_COMPRESSION) + param_.dCompress * output(GEARNORMAL_OUTPUT_COMPRESSIONRATE);
				} else {
					output(GEARNORMAL_OUTPUT_F) =  param_.kCompress * output(GEARNORMAL_OUTPUT_COMPRESSION) + param_.dRebound * output(GEARNORMAL_OUTPUT_COMPRESSIONRATE);
				}
				output.segment(GEARNORMAL_OUTPUT_NIx, 3) = output(GEARNORMAL_OUTPUT_F) * input.segment(GEARNORMAL_INPUT_NGx, 3) * ne.transpose() *  input.segment(GEARNORMAL_INPUT_NGx, 3);
				output.segment(GEARNORMAL_OUTPUT_Nbx, 3) =  (R_IB.transpose() * input.segment(GEARNORMAL_INPUT_NGx, 3)) * ne.transpose() *  input.segment(GEARNORMAL_INPUT_NGx, 3) * output(GEARNORMAL_OUTPUT_F);
				output.segment(GEARNORMAL_OUTPUT_Mbx, 3) = mathauxiliary::Hatmap(output(GEARNORMAL_OUTPUT_COMPRESSION) * param_.GearDirection + param_.GearPosition) * output.segment(GEARNORMAL_OUTPUT_Nbx, 3);
			} 
		} 
	}
}

void groundcontact::SimpleGearNormalForce::IncrementState()
{
	// non incremental states for normal force block
}

void groundcontact::SimpleGearNormalForce::DisplayParameters()
{
	std::cout << "--------- Simple normal force block parameters ------------" << std::endl;
	std::cout << "The gear natural position is : " << std::endl << param_.GearPosition << std::endl;
	std::cout << "The gear direction is : " << std::endl << param_.GearDirection << std::endl;
	std::cout << "The compress stiffness is: " << param_.kCompress << "\n";
	std::cout << "The compress damping is: " << param_.dCompress << "\n";
	std::cout << "The rebound  damping is: " << param_.dRebound << "\n";
	std::cout << "The Max enable height is: " << param_.MaxHeight<< "\n";
	std::cout << "The minimum Nz is: " << param_.MinNz << "\n";
	std::cout << "The wheel rolling direction is : " << std::endl;
	std::cout << nwb << std::endl;
	std::cout << "The wheel plane normal vector is : " << std::endl;
	std::cout << npb << std::endl;
	if (param_.isSteering) {
		std::cout << "The steering in enabled. " << std::endl;
	}
	else
	{
		std::cout << "The steering is disabled. " << std::endl;
	}
	std::cout << "-------- End of block parameters -------- \n" << std::endl;
}

void groundcontact::SimpleGearNormalForce::DisplayInitialCondition()
{
	std::cout << "-------- No initial condition for simple normal force block -------- \n" << std::endl;
}

groundcontact::SimpleGearNormalForce::~SimpleGearNormalForce()
{
}

/*--------------------------- Lugre model --------------------------------*/

groundcontact::GearLuGreFriction::GearLuGreFriction(const GearLuGreFrictionParameter & parameter)
{
	system_info.type = groundcontact_LUGREMODEL;
	system_info.category = GROUNDCONTACT;
	system_info.num_of_continuous_states = 2;
	system_info.num_of_inputs = 14;
	system_info.DIRECT_FEED_THROUGH = true;
	system_info.input_connection.resize(6, 2);
	system_info.num_of_outputs = 6;
	system_info.system_parameter_ok = true;
	system_info.NO_CONTINUOUS_STATE = false;
	param_ = parameter;
	// 
	deltaFriction1 = param_.StaticFrictionCoefficient - param_.DynamicFrictionCoefficient;
	state.resize(system_info.num_of_continuous_states);
	state.setZero();

	// initialized the output vector
	output.resize(system_info.num_of_outputs);
	output.setZero();
}

void groundcontact::GearLuGreFriction::DifferentialEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & derivative)
{
	// calcuate the rolling friction first
	if (input(LUGREFRICTION_INPUT_BREAKING) > 0.0) {
		RollingStaticFriction = param_.StaticFrictionCoefficient;
		RollingDynamicFriction = param_.DynamicFrictionCoefficient;
	} else {
		RollingStaticFriction = param_.RollingFrictionCoefficient;
		RollingDynamicFriction = param_.RollingFrictionCoefficient;
	}

	if (input(LUGREFRICTION_INPUT_VwI) > param_.vlimit) {
		vw = param_.vlimit;
	}
	else if (input(LUGREFRICTION_INPUT_VwI) < -param_.vlimit ){
		vw = -param_.vlimit;
	} else {
		vw = input(LUGREFRICTION_INPUT_VwI);
	}

	if (input(LUGREFRICTION_INPUT_VpI) > param_.vlimit) {
		vp = param_.vlimit;
	}
	else if (input(LUGREFRICTION_INPUT_VpI) < -param_.vlimit) {
		vp = -param_.vlimit;
	} else {
		vp = input(LUGREFRICTION_INPUT_VpI);
	}

	// rolling channel

	derivative(LUGREFRICTION_STATE_ROLL_Z) = LugreZdynamics(state(LUGREFRICTION_STATE_ROLL_Z), vw, RollingDynamicFriction, RollingStaticFriction);
	derivative(LUGREFRICTION_STATE_SIDE_Z) = LugreZdynamics(state(LUGREFRICTION_STATE_SIDE_Z), vp, param_.DynamicFrictionCoefficient, param_.StaticFrictionCoefficient);

}

void groundcontact::GearLuGreFriction::OutputEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & output)
{
	if (input(LUGREFRICTION_INPUT_VwI) > param_.vlimit) {
		vw = param_.vlimit;
	}
	else if (input(LUGREFRICTION_INPUT_VwI) < -param_.vlimit) {
		vw = -param_.vlimit;
	}
	else {
		vw = input(LUGREFRICTION_INPUT_VwI);
	}

	if (input(LUGREFRICTION_INPUT_VpI) > param_.vlimit) {
		vp = param_.vlimit;
	}
	else if (input(LUGREFRICTION_INPUT_VpI) < -param_.vlimit) {
		vp = -param_.vlimit;
	}
	else {
		vp = input(LUGREFRICTION_INPUT_VpI);
	}
	
	if (input(LUGREFRICTION_INPUT_BREAKING) > 0.0) {
		RollingStaticFriction = param_.StaticFrictionCoefficient;
		RollingDynamicFriction = param_.DynamicFrictionCoefficient;
	}
	else {
		RollingStaticFriction = param_.RollingFrictionCoefficient;
		RollingDynamicFriction = param_.RollingFrictionCoefficient;
	}

	TotalRollingFriction = -(state(LUGREFRICTION_STATE_ROLL_Z) * param_.StiffnessSigma0 + param_.DampingSigma1 *  LugreZdynamics(state(LUGREFRICTION_STATE_ROLL_Z), vw, RollingDynamicFriction, RollingStaticFriction) * exp(-param_.SwitchSigmaD * vw*vw));
	TotalSideFriction = -(state(LUGREFRICTION_STATE_SIDE_Z) * param_.StiffnessSigma0 + param_.DampingSigma1 * LugreZdynamics(state(LUGREFRICTION_STATE_SIDE_Z), vp, RollingDynamicFriction, RollingStaticFriction) *exp(-param_.SwitchSigmaD * vp *vp));
	output.segment(LUGREFRICTION_OUTPUT_FIx, 3) = input.segment(LUGREFRICTION_INPUT_NIx, 3) + input.segment(LUGREFRICTION_INPUT_NIx, 3).norm() * (TotalRollingFriction * input.segment(LUGREFRICTION_INPUT_Nwx, 3) + TotalSideFriction * input.segment(LUGREFRICTION_INPUT_Npx, 3));
	CPI = input.segment(LUGREFRICTION_INPUT_CPx, 3);
	FI = output.segment(LUGREFRICTION_OUTPUT_FIx, 3);
	output.segment(LUGREFRICTION_OUTPUT_MIx, 3) = CPI.cross(FI);
}

void groundcontact::GearLuGreFriction::IncrementState()
{
	state += solver_buffer_state_increment1;
}

void groundcontact::GearLuGreFriction::DisplayParameters()
{
	std::cout << "--------- Gear Lugre friction block parameters ------------" << std::endl;
	std::cout << "The rolling coefficient is : " << param_.RollingFrictionCoefficient << "\n";
	std::cout << "The static   friction coefficient is : " << param_.StaticFrictionCoefficient << "\n";
	std::cout << "The dynamics friction coefficient is : " << param_.DynamicFrictionCoefficient << "\n";
	std::cout << "The stiffness sigma 0 is : " << param_.StiffnessSigma0 << "\n";
	std::cout << "The damping sigma 1 is : " << param_.DampingSigma1 << "\n";
	std::cout << "The switch sigma S is : " << param_.SwitchSigmaS << "\n";
	std::cout << "The switch sigma D is : " << param_.SwitchSigmaD << "\n";
	std::cout << "-------- End of block parameters -------- \n" << std::endl;
}

void groundcontact::GearLuGreFriction::DisplayInitialCondition()
{
	std::cout << "-------- The condition for Lugre friction block are set to zeros -------- \n" << std::endl;
}

groundcontact::GearLuGreFriction::~GearLuGreFriction()
{
}

double groundcontact::GearLuGreFriction::LugreZdynamics(const double& z, const double& v, const double& sigma_dynamic, const double& sigam_static)
{
	return v - param_.StiffnessSigma0 * z * abs(v) / (sigma_dynamic + (sigam_static - sigma_dynamic) * exp(-v*v*param_.SwitchSigmaS));
}

