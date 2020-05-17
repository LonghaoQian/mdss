#include "pch.h"
#include "AeroForceMoment1.h"


aero::AeroForceMoment1::AeroForceMoment1(const AerosForceParameter& param) 
	: param_(param),
	CL_(0.0),
	CD_(0.0),
	CY_(0.0)
{
	// set output dimensions
	system_info.type = aero_AROFORCEMENT_1;
	system_info.DIRECT_FEED_THROUGH = true;
	system_info.NO_CONTINUOUS_STATE = true;
	system_info.num_of_continuous_states = 0;
	system_info.num_of_inputs = 16;
	system_info.num_of_outputs = 6;
	system_info.system_parameter_ok = 0;
	ready_to_run = true;
	output.resize(system_info.num_of_outputs);
	output.setZero(system_info.num_of_outputs);

	refer_point_cross_ = mathauxiliary::Hatmap(param.aero_reference_point_);
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
	1.alpha
	2.beta
	3.p_bar
	4.q_bar
	5.r_bar
	6.alpha_dot_bar
	7.beta_dot_bar
	8. flap
	9. spoiler
	10. airbreak
	11. da
	12. de
	13. dr
	14. Mach number
	15. relative height
	*/
	// precalculate some variables
	double QS = input(0) * param_.S;
	double QScbar = QS * param_.c_bar_;
	double QSb = QS * param_.b_;
	double relative_height_ = input(15) / param_.b_;
	mathauxiliary::SaturationElementalWise(relative_height_, 1.1, 0); // TAS 
	// calculate R_WB
	CalculateR_BW(input(1), input(2));
	// calculate CL
	CL_ = param_.aeroparam_.CL0_ 
		+ param_.aeroparam_.CLadot_*input(6)
		+ param_.aeroparam_.CL_alpha_* input(1)
		+ param_.aeroparam_.CLq_ * input(4)
		+ param_.aeroparam_.CLde_ * input(12);

	CD_ = param_.aeroparam_.CD0_ 
		+ param_.aeroparam_.CDbeta_ * abs(input(2))
		+ param_.aeroparam_.CDde_ * abs(input(12))
		+ param_.aeroparam_.CDDf_*input(8)
		+ param_.aeroparam_.CD_alpha_*input(1);

	CY_ = param_.aeroparam_.CYb_ *  input(2)
		+ param_.aeroparam_.CYda_ * input(11)
		+ param_.aeroparam_.CYdr_ * input(13)
		+ param_.aeroparam_.CYp_ *  input(3)
		+ param_.aeroparam_.CYr_*   input(5);

/*
	CL_ = param_.aeroparam_.CL0_
		+ param_.aeroparam_.CL_alpha_beta.GetOutput(input(1), input(2)) * input(1) // CLalpha * alpha
		+ param_.aeroparam_.CLadot_.GetOutput(input(6))                            // CLalphadot * alpha _dot 
		+ param_.aeroparam_.CLde_.GetOutput(input(12))                             // CLde * de                  
		+ param_.aeroparam_.CLDf_.GetOutput(input(8))							  // CLDf * flap
		+ param_.aeroparam_.CL_M.GetOutput(input(14))                            // CL_M*mach
		+param_.aeroparam_.CLground_.GetOutput(relative_height_);

	CD_ = param_.aeroparam_.CD0_
		+ param_.aeroparam_.CD_alpha_flap_.GetOutput(input(1),input(8))
		+ abs(param_.aeroparam_.CDbeta_.GetOutput(input(2)))
		+ param_.aeroparam_.CDde_.GetOutput(input(12))
		+ param_.aeroparam_.CDground_.GetOutput(relative_height_)
		+ param_.aeroparam_.CDDf_.GetOutput(input(8));

	CY_ = param_.aeroparam_.CYb_.GetOutput(input(2))
		+ param_.aeroparam_.CYda_.GetOutput(input(11))
		+ param_.aeroparam_.CYdr_.GetOutput(input(13))
		+ param_.aeroparam_.CYp_.GetOutput(input(3))
		+ param_.aeroparam_.CYr_.GetOutput(input(5));
*/
	/*
	Output:
	0-2:
	F_B
	3-5:
	M_B
	*/
	F_W(0) = -(QS * CD_); // Drag
	F_W(1) = QS * CY_; //  Side
	F_W(2) = -(QS * CL_); // lift


	Cl_ = param_.aeroparam_.Clb_ * input(2) 
		+ param_.aeroparam_.Clda_* input(11)
		+ param_.aeroparam_.Cldr_* input(13)
		+ param_.aeroparam_.Clp_*  input(3)
		+ param_.aeroparam_.Clr_ * input(5);

	Cm_ = param_.aeroparam_.Cmq_*input(4)
		+ param_.aeroparam_.Cm0_
		+ param_.aeroparam_.Cmadot_  * input(6)
		+ param_.aeroparam_.Cmalpha_ * input(1)
		+ param_.aeroparam_.Cmde_ * input(12)
		+ param_.aeroparam_.CmDf_ * input(8);

	Cn_ = param_.aeroparam_.Cnb_  * input(2)
		+ param_.aeroparam_.Cnda_ * input(11)
		+ param_.aeroparam_.Cndr_ * input(13)
		+ param_.aeroparam_.Cnp_  * input(3)
		+ param_.aeroparam_.Cnr_ * input(5);

	output.segment(0, 3) = R_WB.transpose() * F_W;


	M_B0(0) = QSb * Cl_;
	M_B0(1) = QScbar * Cm_;
	M_B0(2) = QSb * Cn_;

	output.segment(3, 3) = refer_point_cross_ * output.segment(0, 3) + M_B0;


#ifdef AEOR_DEBUG
	std << cout << " dynamic pressure : " << input(0) << " [Pa] " << std::endl;
	std << cout << " alpha : " << input(1) << " [rad] " << std::endl;
	std << cout << " beta : " << input(2) << " [rad] " << std::endl;
	std << cout << " alpha : " << input(1) << " [rad] " << std::endl;

#endif // print debug output
}

void aero::AeroForceMoment1::IncrementState()
{
	// No increment state for aero force lib
}

void aero::AeroForceMoment1::DisplayParameters()
{
	// display aeroparameters
	
	std::cout << "S : " << param_.S << " m^2. Cbar : " << param_.c_bar_ <<" m. b : " << param_.b_<<" m " << std::endl;
	std::cout << "offset : " << param_.aero_reference_point_(0) << " m " << param_.aero_reference_point_(1) << " m "
		<<param_.aero_reference_point_(2) << " m " << std::endl;

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
