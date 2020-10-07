#include "pch.h"
#include "AeroForceMoment.h"


aero::AeroForceMoment1::AeroForceMoment1(const AerosForceParameter& param) 
	: param_(param),
	CL_(0.0),
	CD_(0.0),
	CY_(0.0)
{
	// set output dimensions
	system_info.type = aero_AROFORCEMENT;
	system_info.category = AERODYNAMICS;
	system_info.DIRECT_FEED_THROUGH = true;
	system_info.NO_CONTINUOUS_STATE = true;
	system_info.num_of_continuous_states = 0;
	system_info.num_of_inputs = 19;
	system_info.num_of_outputs = 6;
	system_info.system_parameter_ok = 0;
	ready_to_run = true;
	output.resize(system_info.num_of_outputs);
	output.setZero(system_info.num_of_outputs);

}

void aero::AeroForceMoment1::DifferentialEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & derivative)
{
	// No differentiation block for aero dynamic block
}

void aero::AeroForceMoment1::OutputEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & output)
{
	
	/*
	Input:
	0.dynamic pressure
	1.  alpha
	2.  beta
	3.  p_bar
	4.  q_bar
	5.  r_bar
	6.  alpha_dot_bar
	7.  beta_dot_bar
	8.  flap
	9.  spoiler
	10. airbreak
	11. da
	12. de
	13. dr
	14. Mach number
	15. relative height
	16. reference_point_x
	17. reference_point_y
	18. reference_point_z
	*/

	// prepare some variables for later use:
	QS = input(AEROFORCE_INPUT_DYNAMICPRESSURE) * param_.S;
	QScbar = QS * param_.c_bar_;
	QSb = QS * param_.b_;
	NormalizedRelativeHeight = input(AEROFORCE_INPUT_RELATIVEHEIGHT) / param_.b_;
	// get the aero input
	refer_point_cross_ = mathauxiliary::Hatmap(input.segment(AEROFORCE_INPUT_RX,3));

	// saturate the relative height
	mathauxiliary::SaturationElementalWise(NormalizedRelativeHeight, 1.1, 0); 

	// calculate R_WB
	CalculateR_BW(input(AEROFORCE_INPUT_AOA), input(AEROFORCE_INPUT_SIDESLIP));

	// calculate CL
	CL_ = param_.AeroCoefficient.CL0_
		+ param_.AeroCoefficient.CLadot_  * input(AEROFORCE_INPUT_AOARATE_FILTERED)
		+ param_.AeroCoefficient.CL_alpha_* input(AEROFORCE_INPUT_AOA)
		+ param_.AeroCoefficient.CLq_     * input(AEROFORCE_INPUT_Qbar)
		+ param_.AeroCoefficient.CLde_    * input(AEROFORCE_INPUT_ELEVATOR)
		+ param_.AeroCoefficient.CL_alpha_squared_ * input(AEROFORCE_INPUT_AOA) * input(AEROFORCE_INPUT_AOA)
		+ param_.AeroCoefficient.CL_alpha_cubed_ * input(AEROFORCE_INPUT_AOA) * input(AEROFORCE_INPUT_AOA) * input(AEROFORCE_INPUT_AOA);

	CD_ = param_.AeroCoefficient.CD0_
		+ param_.AeroCoefficient.CDbeta_   * abs(input(AEROFORCE_INPUT_SIDESLIP))
		+ param_.AeroCoefficient.CDde_     * abs(input(AEROFORCE_INPUT_ELEVATOR))
		+ param_.AeroCoefficient.CDDf_     *input(AEROFORCE_INPUT_FLAP)
		+ param_.AeroCoefficient.CD_alpha_ *input(AEROFORCE_INPUT_AOA);
		+ param_.AeroCoefficient.CD_alpha_squared_ * input(AEROFORCE_INPUT_AOA) * input(AEROFORCE_INPUT_AOA);

	CY_ = param_.AeroCoefficient.CYb_ *  input(AEROFORCE_INPUT_SIDESLIP)
		+ param_.AeroCoefficient.CYda_ * input(AEROFORCE_INPUT_AILERON)
		+ param_.AeroCoefficient.CYdr_ * input(AEROFORCE_INPUT_RUDDER)
		+ param_.AeroCoefficient.CYp_ *  input(AEROFORCE_INPUT_Pbar)
		+ param_.AeroCoefficient.CYr_*   input(AEROFORCE_INPUT_Rbar);

	/*
	Output:
	0-2:
	F_B
	3-5:
	M_B
	*/

	F_W(mathauxiliary::VECTOR_X) = -(QS * CD_); // Drag
	F_W(mathauxiliary::VECTOR_Y) = QS * CY_; //  Side
	F_W(mathauxiliary::VECTOR_Z) = -(QS * CL_); // lift


	Cl_ = param_.AeroCoefficient.Clb_ * input(AEROFORCE_INPUT_SIDESLIP)
		+ param_.AeroCoefficient.Clda_* input(AEROFORCE_INPUT_AILERON)
		+ param_.AeroCoefficient.Cldr_* input(AEROFORCE_INPUT_RUDDER)
		+ param_.AeroCoefficient.Clp_ *  input(AEROFORCE_INPUT_Pbar)
		+ param_.AeroCoefficient.Clr_ * input(AEROFORCE_INPUT_Rbar);

	Cm_ = param_.AeroCoefficient.Cmq_     *input(AEROFORCE_INPUT_Qbar)
		+ param_.AeroCoefficient.Cm0_
		+ param_.AeroCoefficient.Cmadot_  * input(AEROFORCE_INPUT_AOARATE_FILTERED)
		+ param_.AeroCoefficient.Cmalpha_ * input(AEROFORCE_INPUT_AOA)
		+ param_.AeroCoefficient.Cmde_    * input(AEROFORCE_INPUT_ELEVATOR)
		+ param_.AeroCoefficient.CmDf_    * input(AEROFORCE_INPUT_FLAP);

	Cn_ = param_.AeroCoefficient.Cnb_  * input(AEROFORCE_INPUT_SIDESLIP)
		+ param_.AeroCoefficient.Cnda_ * input(AEROFORCE_INPUT_AILERON)
		+ param_.AeroCoefficient.Cndr_ * input(AEROFORCE_INPUT_RUDDER)
		+ param_.AeroCoefficient.Cnp_  * input(AEROFORCE_INPUT_Pbar)
		+ param_.AeroCoefficient.Cnr_  * input(AEROFORCE_INPUT_Rbar);

	// transfer aero forces from the wind frame to the body-fixed frame
	output.segment(AEROFORCE_OUTPUT_FBx, 3) = R_WB.transpose() * F_W;

	// the moment at the reference point
	M_B0(mathauxiliary::VECTOR_X) = QSb * Cl_;
	M_B0(mathauxiliary::VECTOR_Y) = QScbar * Cm_;
	M_B0(mathauxiliary::VECTOR_Z) = QSb * Cn_;
	// the total moment around the center of mass
	output.segment(AEROFORCE_OUTPUT_MBx, 3) = refer_point_cross_ * output.segment(AEROFORCE_OUTPUT_FBx, 3) + M_B0;

}

void aero::AeroForceMoment1::IncrementState()
{
	// No increment states for aero force lib
}

void aero::AeroForceMoment1::DisplayParameters()
{
	// display aeroparameters
	std::cout << "The parameters of the aeroforce and aeromoment are: " << std::endl;
	std::cout << "S : " << param_.S << " m^2. Cbar : " << param_.c_bar_ <<" m. b : " << param_.b_<<" m " << std::endl;
}

void aero::AeroForceMoment1::DisplayInitialCondition()
{
	std::cout << "No Initial Condition for aero force and moment block" << std::endl;
}
void aero::AeroForceMoment1::CalculateR_BW(const double & alpha_, const double & beta_)
{
	double sinalpha = sin(alpha_);
	double sinbeta = sin(beta_);
	double cosalpha = cos(alpha_);
	double cosbeta = cos(beta_);
	R_WB(0, 0) = cosalpha * cosbeta;
	R_WB(0, 1) = sinbeta;
	R_WB(0, 2) = sinalpha * cosbeta;
	R_WB(1, 0) = -sinbeta * cosalpha;
	R_WB(1, 1) = cosbeta;
	R_WB(1, 2) = -sinalpha * sinbeta;
	R_WB(2, 0) = -sinalpha;
	R_WB(2, 1) = 0;
	R_WB(2, 2) = cosalpha;
}

aero::AeroForceMoment1::~AeroForceMoment1()
{
}
