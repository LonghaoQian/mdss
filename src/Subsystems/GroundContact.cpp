#include "pch.h"
#include "GroundContact.h"

groundcontact::SimpleGearNormalForce::SimpleGearNormalForce(const SimpleGearNormalForceParameter & param)
{
	system_info.type = groundcontact_SIMPLENORMAL;
	system_info.category = GROUNDCONTACT;
	system_info.num_of_continuous_states = 0;
	system_info.num_of_inputs = 24;
	system_info.DIRECT_FEED_THROUGH = true;
	system_info.input_connection.resize(6, 2);
	system_info.num_of_outputs = 12;
	system_info.system_parameter_ok = true;
	system_info.NO_CONTINUOUS_STATE = true;
	param_ = param;
	output.resize(system_info.num_of_outputs);
	output.setZero();
}

void groundcontact::SimpleGearNormalForce::DifferentialEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & derivative)
{
	// non differential functions for normal force block
}

void groundcontact::SimpleGearNormalForce::OutputEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & output)
{
	// TO DO: check the sign and reference frame consistency
	//std::cout << input(GEARNORMAL_INPUT_PIz) - input(GEARNORMAL_INPUT_H) + param_.MaxHeight << std::endl;
	//std::cout << input(GEARNORMAL_INPUT_PIz) - input(GEARNORMAL_INPUT_H) + param_.MaxHeight << std::endl;
	output.setZero();
	if ((input(GEARNORMAL_INPUT_SWITCH) > 0.0) && ( input(GEARNORMAL_INPUT_PIz) - input(GEARNORMAL_INPUT_H) + param_.MaxHeight >0.0)) {
		// the aircraft is under the height threshold
		mathauxiliary::ConvertVectorToRotationMatrix(input.segment(GEARNORMAL_INPUT_R_IB00, 9), R_IB);
		ne = R_IB * param_.GearDirection;
		//std::cout << ne(mathauxiliary::VECTOR_Z) << std::endl;
		if (ne(mathauxiliary::VECTOR_Z) > param_.MinNz) {
			R_IB_dot = R_IB* mathauxiliary::Hatmap(input.segment(GEARNORMAL_INPUT_OMEGAbx, 3));
			re = input.segment(GEARNORMAL_INPUT_PIx, 3) + R_IB * param_.GearPosition;
			re_dot = input.segment(GEARNORMAL_INPUT_VIx, 3) + R_IB_dot * param_.GearPosition;
			ne_dot = R_IB_dot * param_.GearDirection;
			output(GEARNORMAL_OUTPUT_COMPRESSION) = (input(GEARNORMAL_INPUT_H) - re(mathauxiliary::VECTOR_Z)) / ne(mathauxiliary::VECTOR_Z); // the compression distance
			if (output(GEARNORMAL_OUTPUT_COMPRESSION) < 0.0) { // if the compression distance is negative, the tie touches the ground
				output(GEARNORMAL_OUTPUT_COMPRESSIONRATE) = -(ne_dot(mathauxiliary::VECTOR_Z) * output(GEARNORMAL_OUTPUT_COMPRESSION) - input(GEARNORMAL_INPUT_Hdot) + re_dot(mathauxiliary::VECTOR_Z)) / ne(mathauxiliary::VECTOR_Z);
				if (output(GEARNORMAL_OUTPUT_COMPRESSIONRATE) < 0.0) { // compression speed is negative means that suspension is compressing
					output(GEARNORMAL_OUTPUT_F) =  param_.kCompress * output(GEARNORMAL_OUTPUT_COMPRESSION) + param_.dCompress * output(GEARNORMAL_OUTPUT_COMPRESSIONRATE);
				} else {
					output(GEARNORMAL_OUTPUT_F) =  param_.kCompress * output(GEARNORMAL_OUTPUT_COMPRESSION) + param_.dRebound * output(GEARNORMAL_OUTPUT_COMPRESSIONRATE);
				}
				output.segment(GEARNORMAL_OUTPUT_NIx, 3).setZero();// To Do; modify this later
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
	std::cout << "-------- End of block parameters -------- \n" << std::endl;
}

void groundcontact::SimpleGearNormalForce::DisplayInitialCondition()
{
	std::cout << "-------- No initial condition for simple normal force block parameters -------- \n" << std::endl;
}

groundcontact::SimpleGearNormalForce::~SimpleGearNormalForce()
{
}
