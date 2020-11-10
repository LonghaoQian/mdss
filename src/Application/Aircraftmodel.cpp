#include "pch.h"
#include "Aircraftmodel.h"
namespace  aircraft {
	AircraftDynamicModel::AircraftDynamicModel(const modelparameter & param, const initialcondition & IC)
	{
		// set the solver config
		SimInstance1.EditSolverConfig(param.Config);
		// save the initial condition
		currentIC = IC;
		// save the parameter
		parameter = param;
		// set up the model
		DefineRigidbody();

		DefineAerodynamics();

		DefineEngine();

		DefineAutopilot();

		ConnectSystems();

		SimInstance1.FlushMakeConnection();
		// define logging
		DefineLogging();
		// run preprocess
		modelok = SimInstance1.PreRunProcess();

		//SimInstance1.DisplayLoggerTagList(); // logge tag list display

		if (modelok) {
			std::cout << " All good, ready to run. Enter 1 to start.../ Enter other number to stop ..." << std::endl;
			//get the external input ready
			SimInstance1.ReshapeExternalInputVector(extern_input);
		}
		else {
			std::cout << "Initialization failed, check error messages!" << std::endl;
		}
		
	}

	AircraftDynamicModel::~AircraftDynamicModel()
	{
	}

	void AircraftDynamicModel::UpdateSimulation(const C172input & input)
	{
		
		
		if (modelok) { // if successful, run updates
			isRunning = true;
			SimInstance1.Run_Update(extern_input);
		}else {
			std::cout << "Initialization failed, can not run the model" << std::endl;
		}
	}

	void AircraftDynamicModel::EndSimulation()
	{
		SimInstance1.PostRunProcess();
		isRunning = false;
	}

	bool AircraftDynamicModel::ResetParameter(const modelparameter & param)
	{
		return false;
	}

	bool AircraftDynamicModel::ResetSimulation(const initialcondition & IC)
	{
		return false;
	}
	void AircraftDynamicModel::DefineRigidbody()
	{
		// define kinematics
		dynamics::RigidBodyKinematicsInitialCondition initial_condition;
		initial_condition.Euler0(mathauxiliary::EULER_ROLL)  = currentIC.plane.roll;
		initial_condition.Euler0(mathauxiliary::EULER_PITCH) = currentIC.plane.pitch;
		initial_condition.Euler0(mathauxiliary::EULER_YAW)   = currentIC.plane.yaw;
		initial_condition.Omega0(mathauxiliary::VECTOR_X) = currentIC.plane.omegax;
		initial_condition.Omega0(mathauxiliary::VECTOR_Y) = currentIC.plane.omegay;
		initial_condition.Omega0(mathauxiliary::VECTOR_Z) = currentIC.plane.omegaz;
		initial_condition.VI0(mathauxiliary::VECTOR_X) = currentIC.plane.inertialvelocityx;
		initial_condition.VI0(mathauxiliary::VECTOR_Y) = currentIC.plane.inertialvelocityy;
		initial_condition.VI0(mathauxiliary::VECTOR_Z) = currentIC.plane.inertialvelocityz;
		initial_condition.XI0(mathauxiliary::VECTOR_X) = currentIC.plane.inertialpositionx;
		initial_condition.XI0(mathauxiliary::VECTOR_Y) = currentIC.plane.inertialpositiony;
		initial_condition.XI0(mathauxiliary::VECTOR_Z) = currentIC.plane.inertialpositionz;
		Modelist.dynamics.planekinematics = SimInstance1.AddSubSystem(initial_condition);
		// define dynamics
		dynamics::RigidBodyDynamicsParamter dynamics_parameter;
		dynamics_parameter.J   = parameter.inertia.J;
		// ** TO DO: this is a temp measure, fuel weight will be added later
		dynamics_parameter.m   = parameter.inertia.EmptyWeight + parameter.inertia.Pilot1 + parameter.inertia.Pilot2 + parameter.inertia.Pilot3 + +parameter.inertia.Pilot4;
		Modelist.dynamics.planedynamics = SimInstance1.AddSubSystem(dynamics_parameter);
		// define gravity 
		mathblocks::ConstantParameter gravity_param;
		gravity_param.value.resize(3, 1);
		gravity_param.value(0) = 0.0;
		gravity_param.value(1) = 0.0;
		gravity_param.value(2) = 9.81*dynamics_parameter.m;
		Modelist.dynamics.gravityinertial = SimInstance1.AddSubSystem(gravity_param);
		// define gravity in body fixed frame
		mathblocks::MultiplicationParam multiple_1_param;
		multiple_1_param.Mode = mathblocks::MULTI_MATRIX;
		multiple_1_param.input1_dimension(mathblocks::MATRIX_ROW) = 3;
		multiple_1_param.input1_dimension(mathblocks::MATRIX_COL) = 3;
		multiple_1_param.input2_dimension(mathblocks::MATRIX_ROW) = 3;
		multiple_1_param.input2_dimension(mathblocks::MATRIX_COL) = 1;
		Modelist.dynamics.gravitybody = SimInstance1.AddSubSystem(multiple_1_param);

		mathblocks::MultiplicationParam multiple_2_param;
		multiple_2_param.Mode = mathblocks::MULTI_MATRIX;
		multiple_2_param.input1_dimension(mathblocks::MATRIX_ROW) = 3;
		multiple_2_param.input1_dimension(mathblocks::MATRIX_COL) = 3;
		multiple_2_param.input2_dimension(mathblocks::MATRIX_ROW) = 3;
		multiple_2_param.input2_dimension(mathblocks::MATRIX_COL) = 1;
		Modelist.dynamics.product = SimInstance1.AddSubSystem(multiple_2_param);

		mathblocks::CrossProductParameter cross_1_param;
		cross_1_param.mode = 0;
		Modelist.dynamics.crossproduct = SimInstance1.AddSubSystem(cross_1_param);
		// get Vb
		mathblocks::SumParameter sum_Vb_parameter;
		sum_Vb_parameter.input_dimensions = 3;
		sum_Vb_parameter.SignList.push_back(mathblocks::SUM_POSITIVE);
		sum_Vb_parameter.SignList.push_back(mathblocks::SUM_POSITIVE);
		Modelist.dynamics.Vbdot = SimInstance1.AddSubSystem(sum_Vb_parameter);
		// total force
		mathblocks::SumParameter sum_total_force_parameter;
		sum_total_force_parameter.input_dimensions = 3;                         // force is a 3 by 1 vector 
		sum_total_force_parameter.SignList.push_back(mathblocks::SUM_POSITIVE); // other forces
		sum_total_force_parameter.SignList.push_back(mathblocks::SUM_POSITIVE); // gravity
		Modelist.dynamics.sumtotalinertialforce = SimInstance1.AddSubSystem(sum_total_force_parameter);
		// height
		mathblocks::GainParameter height_param;
		height_param.K.resize(1, 1);
		height_param.K(0, 0) = - 1.0;
		height_param.Mode = mathblocks::ElementWise;
		height_param.num_of_inputs = 1;
		Modelist.dynamics.height = SimInstance1.AddSubSystem(height_param);
		// climbrate
		mathblocks::GainParameter climbrate_param;
		climbrate_param.K.resize(1, 1);
		climbrate_param.K(0, 0) = -1.0;
		climbrate_param.Mode = mathblocks::ElementWise;
		climbrate_param.num_of_inputs = 1;
		Modelist.dynamics.climbrate= SimInstance1.AddSubSystem(climbrate_param);
		// ACC filter 
		linearsystem::TransferFunctionParameter ACCxfilterparam;
		ACCxfilterparam.Numerator.resize(1);
		ACCxfilterparam.Numerator(0) = 30.0;
		ACCxfilterparam.Denominator.resize(2);
		ACCxfilterparam.Denominator(0) = 1.0;
		ACCxfilterparam.Denominator(1) = 30.0;
		Modelist.dynamics.ACCxfilter= SimInstance1.AddSubSystem(ACCxfilterparam);

		Modelist.dynamics.ACCyfilter = SimInstance1.AddSubSystem(ACCxfilterparam);
		Modelist.dynamics.ACCzfilter = SimInstance1.AddSubSystem(ACCxfilterparam);


		// total force
		mathblocks::SumParameter sum_total_bodyforce_parameter;
		sum_total_bodyforce_parameter.input_dimensions = 3;                         // 
		sum_total_bodyforce_parameter.SignList.push_back(mathblocks::SUM_POSITIVE); // aerodynamics
		sum_total_bodyforce_parameter.SignList.push_back(mathblocks::SUM_POSITIVE); // thrust
		Modelist.dynamics.sumtotalbodyforce = SimInstance1.AddSubSystem(sum_total_bodyforce_parameter);
		// from body-fixed frame to inertial frame
		mathblocks::MultiplicationParam multiple_3_param;
		multiple_3_param.Mode = mathblocks::MULTI_MATRIX;
		multiple_3_param.input1_dimension(mathblocks::MATRIX_ROW) = 3;
		multiple_3_param.input1_dimension(mathblocks::MATRIX_COL) = 3;
		multiple_3_param.input2_dimension(mathblocks::MATRIX_ROW) = 3;
		multiple_3_param.input2_dimension(mathblocks::MATRIX_COL) = 1;
		Modelist.dynamics.rotation2inertialframe = SimInstance1.AddSubSystem(multiple_3_param);


	}
	void AircraftDynamicModel::DefineAerodynamics()
	{
		// define aeroangle
		aero::AeroAngleParameter aeroangleparameter;
		aeroangleparameter.b_ = parameter.geometry.Span;
		aeroangleparameter.c_bar_ = parameter.geometry.MeanChord;
		aeroangleparameter.min_airspeed_ = parameter.aerodynamics.MinAirspeed;
		Modelist.aerodynamics.aeroangle = SimInstance1.AddSubSystem(aeroangleparameter);

		// define aero force
		aero::AerosForceParameter aeroforceparam;

		aeroforceparam.b_      = parameter.geometry.Span;
		aeroforceparam.c_bar_  = parameter.geometry.MeanChord;
		aeroforceparam.S       = parameter.geometry.ReferenceArea;

		aeroforceparam.AeroCoefficient.Lift.CL0_      = parameter.aerodynamics.lift.CL0;
		aeroforceparam.AeroCoefficient.Lift.CLadot_   = parameter.aerodynamics.lift.CLalpha_dot;
		aeroforceparam.AeroCoefficient.Lift.CLde_ = parameter.aerodynamics.lift.CLde;
		aeroforceparam.AeroCoefficient.Lift.CLq_ = parameter.aerodynamics.lift.CLq;
		aeroforceparam.AeroCoefficient.Lift.CL_alpha_ = parameter.aerodynamics.lift.CLalpha;
		aeroforceparam.AeroCoefficient.Lift.CL_alpha_squared_ = parameter.aerodynamics.lift.CLalpha_squared;
		aeroforceparam.AeroCoefficient.Lift.CL_alpha_cubed_ = parameter.aerodynamics.lift.CLalpha_cubed;
		aeroforceparam.AeroCoefficient.Lift.CL_flap_ = parameter.aerodynamics.lift.CLflap;
		aeroforceparam.AeroCoefficient.Lift.CL_flap_squared_ = parameter.aerodynamics.lift.CLflap_squared;

		aeroforceparam.AeroCoefficient.Drag.CD0_ = parameter.aerodynamics.drag.CD0_;
		aeroforceparam.AeroCoefficient.Drag.CDbeta_ = parameter.aerodynamics.drag.CDbeta_;
		aeroforceparam.AeroCoefficient.Drag.CDde_ = parameter.aerodynamics.drag.CDde_;
		aeroforceparam.AeroCoefficient.Drag.CD_flap_ = parameter.aerodynamics.drag.CD_flap_;
		aeroforceparam.AeroCoefficient.Drag.CD_flap_squared_ = parameter.aerodynamics.drag.CD_flap_squared_;
		aeroforceparam.AeroCoefficient.Drag.CDground_ = parameter.aerodynamics.drag.CDground_;
		aeroforceparam.AeroCoefficient.Drag.CD_alpha_ = parameter.aerodynamics.drag.CD_alpha_;
		aeroforceparam.AeroCoefficient.Drag.CD_alpha_squared_ = parameter.aerodynamics.drag.CD_alpha_squared_;
		aeroforceparam.AeroCoefficient.Drag.CD_flap_ = parameter.aerodynamics.drag.CD_flap_;
		aeroforceparam.AeroCoefficient.Drag.CD_flap_squared_ = parameter.aerodynamics.drag.CD_flap_squared_;

		aeroforceparam.AeroCoefficient.Side.CYb_ = parameter.aerodynamics.side.CYb;
		aeroforceparam.AeroCoefficient.Side.CYda_ = parameter.aerodynamics.side.CYda;
		aeroforceparam.AeroCoefficient.Side.CYdr_ = parameter.aerodynamics.side.CYdr;
		aeroforceparam.AeroCoefficient.Side.CYp_ = parameter.aerodynamics.side.CYp;
		aeroforceparam.AeroCoefficient.Side.CYr_ = parameter.aerodynamics.side.CYr;

		aeroforceparam.AeroCoefficient.Roll.Clb_ = parameter.aerodynamics.roll.Clb_;
		aeroforceparam.AeroCoefficient.Roll.Clda_ = parameter.aerodynamics.roll.Clda_;
		aeroforceparam.AeroCoefficient.Roll.Cldr_ = parameter.aerodynamics.roll.Cldr_;
		aeroforceparam.AeroCoefficient.Roll.Clp_ = parameter.aerodynamics.roll.Clp_;
		aeroforceparam.AeroCoefficient.Roll.Clr_ = parameter.aerodynamics.roll.Clr_;

		aeroforceparam.AeroCoefficient.Pitch.Cm0_ = parameter.aerodynamics.pitch.Cm0_;
		aeroforceparam.AeroCoefficient.Pitch.Cmadot_ = parameter.aerodynamics.pitch.Cmadot_;
		aeroforceparam.AeroCoefficient.Pitch.Cmalpha_ = parameter.aerodynamics.pitch.Cmalpha_;
		aeroforceparam.AeroCoefficient.Pitch.Cmde_ = parameter.aerodynamics.pitch.Cmde_;
		aeroforceparam.AeroCoefficient.Pitch.Cm_flap_ = parameter.aerodynamics.pitch.Cm_flap_;
		aeroforceparam.AeroCoefficient.Pitch.Cm_flap_squared_ = parameter.aerodynamics.pitch.Cm_flap_squared_;
		aeroforceparam.AeroCoefficient.Pitch.Cmq_ = parameter.aerodynamics.pitch.Cmq_;

		aeroforceparam.AeroCoefficient.Yaw.Cnb_  = parameter.aerodynamics.yaw.Cnb_;
		aeroforceparam.AeroCoefficient.Yaw.Cnda_ = parameter.aerodynamics.yaw.Cnda_;
		aeroforceparam.AeroCoefficient.Yaw.Cndr_ = parameter.aerodynamics.yaw.Cndr_;
		aeroforceparam.AeroCoefficient.Yaw.Cnp_  = parameter.aerodynamics.yaw.Cnp_;
		aeroforceparam.AeroCoefficient.Yaw.Cnr_  = parameter.aerodynamics.yaw.Cnr_;

		Modelist.aerodynamics.aeroforcemoment = SimInstance1.AddSubSystem(aeroforceparam);
		// atm 
		geographic::StandardAtmosphereParameter atom_param;
		atom_param.atmoshpere_name_ = "atom_data";
		Modelist.aerodynamics.atmosphere = SimInstance1.AddSubSystem(atom_param);
		// aoarate filter
		linearsystem::TransferFunctionParameter AOAfilterparam;
		AOAfilterparam.Numerator.resize(1);
		AOAfilterparam.Numerator(0) = 20.0;
		AOAfilterparam.Denominator.resize(2);
		AOAfilterparam.Denominator(0) = 1.0;
		AOAfilterparam.Denominator(1) = 20.0;
		Modelist.aerodynamics.filteredAOArate = SimInstance1.AddSubSystem(AOAfilterparam);
		// betarate filter
		linearsystem::TransferFunctionParameter Betafilterparam;
		Betafilterparam.Numerator.resize(1);
		Betafilterparam.Numerator(0) = 20.0;
		Betafilterparam.Denominator.resize(2);
		Betafilterparam.Denominator(0) = 1.0;
		Betafilterparam.Denominator(1) = 20.0;
		Modelist.aerodynamics.filteredBetarate = SimInstance1.AddSubSystem(Betafilterparam);

	}
	void AircraftDynamicModel::DefineEngine()
	{
		// propeller system
		propulsionsystem::PropellerChartFixedPitchParameter propeller_param;
		propeller_param.Chart = parameter.propeller.Chart;
		propeller_param.diameter = parameter.propeller.diameter;
		propeller_param.minimumAngularRate = parameter.propeller.minimumAngularRate;
		Modelist.engine.propeller = SimInstance1.AddSubSystem(propeller_param);

		// piston engine system
		propulsionsystem::PistonEngineParameter pistonengine_param;
		pistonengine_param.idle_RPM = parameter.pistonengine.idle_RPM;
		pistonengine_param.krho0 = parameter.pistonengine.krho0;
		pistonengine_param.krho1 = parameter.pistonengine.krho1;
		pistonengine_param.sfc = parameter.pistonengine.sfc;
		pistonengine_param.PowerMixtureChart = parameter.pistonengine.PowerMixtureChart;
		pistonengine_param.TorqueRPMChart = parameter.pistonengine.TorqueRPMChart;

		pistonengine_param.MixturePowerFactorSFCfactorChart = parameter.pistonengine.MixturePowerFactorSFCfactorChart;
		pistonengine_param.shaft_damping = parameter.pistonengine.shaft_damping;
		pistonengine_param.superchargerfactor = parameter.pistonengine.superchargerfactor;
		pistonengine_param.stater_breakaway_RPM = parameter.pistonengine.stater_breakaway_RPM;
		pistonengine_param.stater_zero_torque_RPM = parameter.pistonengine.stater_zero_torque_RPM;

		Modelist.engine.pistonengine = SimInstance1.AddSubSystem(pistonengine_param);
		// shaft dynamics

		mathblocks::GainParameter shaft_inertia_param;
		shaft_inertia_param.K.resize(1, 1);
		shaft_inertia_param.K(0, 0) = 1.0 / parameter.propeller.shaftinertia;
		shaft_inertia_param.Mode = mathblocks::ElementWise;
		shaft_inertia_param.num_of_inputs = 1;

		Modelist.engine.shaftinertia = SimInstance1.AddSubSystem(shaft_inertia_param);

		mathblocks::GainParameter omega_rps_param;
		omega_rps_param.K.resize(1, 1);
		omega_rps_param.K(0, 0) = 1.0 / (2.0*M_PI);
		omega_rps_param.Mode = mathblocks::ElementWise;
		omega_rps_param.num_of_inputs = 1;

		Modelist.engine.omega2rps = SimInstance1.AddSubSystem(omega_rps_param);

		linearsystem::IntegratorParameter shaftdynamics_param;
		shaftdynamics_param.num_of_channels = 1;
		linearsystem::IntegratorInitialCondition shaftdynamics_IC;
		shaftdynamics_IC.X_0.resize(1);
		shaftdynamics_IC.X_0(0) = 2.0 * M_PI * currentIC.engine.propellerRPM / 60.0;

		Modelist.engine.shaftdynamics = SimInstance1.AddSubSystem(shaftdynamics_param, shaftdynamics_IC);

		mathblocks::SumParameter total_torque_to_shaft_param;

		total_torque_to_shaft_param.input_dimensions = 1;
		total_torque_to_shaft_param.SignList.push_back(mathblocks::SUM_POSITIVE);
		total_torque_to_shaft_param.SignList.push_back(mathblocks::SUM_NEGATIVE);

		Modelist.engine.totaltorque = SimInstance1.AddSubSystem(total_torque_to_shaft_param);

		mathblocks::ConstantParameter propellerorientation_param;
		propellerorientation_param.value.resize(3);
		propellerorientation_param.value(0) = 1.0;
		propellerorientation_param.value(1) = 0.0;
		propellerorientation_param.value(2) = 0.0;

		Modelist.engine.propellerorientation = SimInstance1.AddSubSystem(propellerorientation_param);


		mathblocks::MultiplicationParam thrustforce_param;
		thrustforce_param.Mode = mathblocks::MULTI_SCALAR;
		thrustforce_param.input1_dimension(mathblocks::MATRIX_ROW) = 1;
		thrustforce_param.input1_dimension(mathblocks::MATRIX_COL) = 1;
		thrustforce_param.input2_dimension(mathblocks::MATRIX_ROW) = 3;
		thrustforce_param.input2_dimension(mathblocks::MATRIX_COL) = 1;


		Modelist.engine.thrustforce = SimInstance1.AddSubSystem(thrustforce_param);

		/* ------------------------------ some temp blocks -----------------------------*/
		mathblocks::ConstantParameter fixed_throttle_param;
		fixed_throttle_param.value.resize(1);
		fixed_throttle_param.value(0) = 0.5;
		Modelist.temp.fixedthrottle = SimInstance1.AddSubSystem(fixed_throttle_param);


		mathblocks::ConstantParameter fixedmxiture_param;

		fixedmxiture_param.value.resize(1);
		fixedmxiture_param.value(0) =0.0;

		Modelist.temp.fixedmixture = SimInstance1.AddSubSystem(fixedmxiture_param);

		mathblocks::ConstantParameter fixefuelstate_param;

		fixefuelstate_param.value.resize(1);
		fixefuelstate_param.value(0) = 1.0;

		Modelist.temp.fixedfuelstate = SimInstance1.AddSubSystem(fixefuelstate_param);
	}
	void AircraftDynamicModel::DefineAutopilot()
	{
	}
	void AircraftDynamicModel::DefineLogging()
	{
		SimInstance1.DefineDataLogging(Modelist.dynamics.planedynamics, dynamics::DYNAMICS_OUTPUT_AIx, "AIx");
		SimInstance1.DefineDataLogging(Modelist.dynamics.planedynamics, dynamics::DYNAMICS_OUTPUT_AIy, "AIy");
		SimInstance1.DefineDataLogging(Modelist.dynamics.planedynamics, dynamics::DYNAMICS_OUTPUT_AIz, "AIz");
		SimInstance1.DefineDataLogging(Modelist.dynamics.planedynamics, dynamics::DYNAMICS_OUTPUT_OMEGA_DOTx, "omega_dotx");
		SimInstance1.DefineDataLogging(Modelist.dynamics.planedynamics, dynamics::DYNAMICS_OUTPUT_OMEGA_DOTy, "omega_doty");
		SimInstance1.DefineDataLogging(Modelist.dynamics.planedynamics, dynamics::DYNAMICS_OUTPUT_OMEGA_DOTz, "omega_dotz");

		SimInstance1.DefineDataLogging(Modelist.dynamics.planekinematics, dynamics::KINEMATICS_OUTPUT_EulerRoll, "roll");
		SimInstance1.DefineDataLogging(Modelist.dynamics.planekinematics, dynamics::KINEMATICS_OUTPUT_EulerPitch, "pitch");
		SimInstance1.DefineDataLogging(Modelist.dynamics.planekinematics, dynamics::KINEMATICS_OUTPUT_EulerYaw, "yaw");

		SimInstance1.DefineDataLogging(Modelist.dynamics.planekinematics, dynamics::KINEMATICS_OUTPUT_OmegaBIx, "omegax");
		SimInstance1.DefineDataLogging(Modelist.dynamics.planekinematics, dynamics::KINEMATICS_OUTPUT_OmegaBIy, "omegay");
		SimInstance1.DefineDataLogging(Modelist.dynamics.planekinematics, dynamics::KINEMATICS_OUTPUT_OmegaBIz, "omegaz");

		SimInstance1.DefineDataLogging(Modelist.aerodynamics.aeroforcemoment, aero::AEROFORCE_OUTPUT_R_WB00, "RWB00");
		SimInstance1.DefineDataLogging(Modelist.aerodynamics.aeroforcemoment, aero::AEROFORCE_OUTPUT_R_WB01, "RWB01");
		SimInstance1.DefineDataLogging(Modelist.aerodynamics.aeroforcemoment, aero::AEROFORCE_OUTPUT_R_WB02, "RWB02");
		SimInstance1.DefineDataLogging(Modelist.aerodynamics.aeroforcemoment, aero::AEROFORCE_OUTPUT_R_WB10, "RWB10");
		SimInstance1.DefineDataLogging(Modelist.aerodynamics.aeroforcemoment, aero::AEROFORCE_OUTPUT_R_WB11, "RWB11");
		SimInstance1.DefineDataLogging(Modelist.aerodynamics.aeroforcemoment, aero::AEROFORCE_OUTPUT_R_WB12, "RWB12");
		SimInstance1.DefineDataLogging(Modelist.aerodynamics.aeroforcemoment, aero::AEROFORCE_OUTPUT_R_WB20, "RWB20");
		SimInstance1.DefineDataLogging(Modelist.aerodynamics.aeroforcemoment, aero::AEROFORCE_OUTPUT_R_WB21, "RWB21");
		SimInstance1.DefineDataLogging(Modelist.aerodynamics.aeroforcemoment, aero::AEROFORCE_OUTPUT_R_WB22, "RWB22");

		SimInstance1.DefineDataLogging(Modelist.dynamics.planekinematics, dynamics::KINEMATICS_OUTPUT_R_IB00, "RIB00");
		SimInstance1.DefineDataLogging(Modelist.dynamics.planekinematics, dynamics::KINEMATICS_OUTPUT_R_IB01, "RIB01");
		SimInstance1.DefineDataLogging(Modelist.dynamics.planekinematics, dynamics::KINEMATICS_OUTPUT_R_IB02, "RIB02");

		SimInstance1.DefineDataLogging(Modelist.dynamics.planekinematics, dynamics::KINEMATICS_OUTPUT_R_IB10, "RIB10");
		SimInstance1.DefineDataLogging(Modelist.dynamics.planekinematics, dynamics::KINEMATICS_OUTPUT_R_IB11, "RIB11");
		SimInstance1.DefineDataLogging(Modelist.dynamics.planekinematics, dynamics::KINEMATICS_OUTPUT_R_IB12, "RIB12");

		SimInstance1.DefineDataLogging(Modelist.dynamics.planekinematics, dynamics::KINEMATICS_OUTPUT_R_IB20, "RIB20");
		SimInstance1.DefineDataLogging(Modelist.dynamics.planekinematics, dynamics::KINEMATICS_OUTPUT_R_IB21, "RIB21");
		SimInstance1.DefineDataLogging(Modelist.dynamics.planekinematics, dynamics::KINEMATICS_OUTPUT_R_IB22, "RIB22");

		SimInstance1.DefineDataLogging(Modelist.dynamics.planekinematics, dynamics::KINEMATICS_OUTPUT_VBx, "Vbx");
		SimInstance1.DefineDataLogging(Modelist.dynamics.planekinematics, dynamics::KINEMATICS_OUTPUT_VBy, "Vby");
		SimInstance1.DefineDataLogging(Modelist.dynamics.planekinematics, dynamics::KINEMATICS_OUTPUT_VBz, "Vbz");

		SimInstance1.DefineDataLogging(Modelist.dynamics.planekinematics, dynamics::KINEMATICS_OUTPUT_VIx, "VIx");
		SimInstance1.DefineDataLogging(Modelist.dynamics.planekinematics, dynamics::KINEMATICS_OUTPUT_VIy, "VIy");
		SimInstance1.DefineDataLogging(Modelist.dynamics.planekinematics, dynamics::KINEMATICS_OUTPUT_VIz, "VIz");

		SimInstance1.DefineDataLogging(Modelist.dynamics.planekinematics, dynamics::KINEMATICS_OUTPUT_XIx, "XIx");
		SimInstance1.DefineDataLogging(Modelist.dynamics.planekinematics, dynamics::KINEMATICS_OUTPUT_XIy, "XIy");
		SimInstance1.DefineDataLogging(Modelist.dynamics.planekinematics, dynamics::KINEMATICS_OUTPUT_XIz, "XIz");

		SimInstance1.DefineDataLogging(Modelist.dynamics.Vbdot, mathauxiliary::VECTOR_X, "Vb_dotx");
		SimInstance1.DefineDataLogging(Modelist.dynamics.Vbdot, mathauxiliary::VECTOR_Y, "Vb_doty");
		SimInstance1.DefineDataLogging(Modelist.dynamics.Vbdot, mathauxiliary::VECTOR_Z, "Vb_dotz");

		SimInstance1.DefineDataLogging(Modelist.aerodynamics.atmosphere, geographic::AtmPressure, "pressure");
		SimInstance1.DefineDataLogging(Modelist.aerodynamics.atmosphere, geographic::AtmSoundSpeed, "soundspeed");
		SimInstance1.DefineDataLogging(Modelist.aerodynamics.atmosphere, geographic::AtmTemperature, "temperature");
		SimInstance1.DefineDataLogging(Modelist.aerodynamics.atmosphere, geographic::AtmDensity, "density");

		SimInstance1.DefineDataLogging(Modelist.dynamics.gravitybody, 0, "g0");
		SimInstance1.DefineDataLogging(Modelist.dynamics.gravitybody, 1, "g1");
		SimInstance1.DefineDataLogging(Modelist.dynamics.gravitybody, 2, "g2");
		SimInstance1.DefineDataLogging(Modelist.aerodynamics.aeroangle, aero::AERO_OUTPUT_TAS, "TAS");
		SimInstance1.DefineDataLogging(Modelist.aerodynamics.aeroangle, aero::AERO_OUTPUT_AOA, "AOA");
		SimInstance1.DefineDataLogging(Modelist.aerodynamics.aeroangle, aero::AERO_OUTPUT_AOARATE, "AOArate");
		SimInstance1.DefineDataLogging(Modelist.aerodynamics.aeroangle, aero::AERO_OUTPUT_SIDESLIP, "Sideslip");
		SimInstance1.DefineDataLogging(Modelist.aerodynamics.aeroangle, aero::AERO_OUTPUT_SIDESLIPRATE, "Sidesliprate");
		SimInstance1.DefineDataLogging(Modelist.aerodynamics.aeroangle, aero::AERO_OUTPUT_DYNAMICPRESSURE, "dynamicpressure");
		SimInstance1.DefineDataLogging(Modelist.aerodynamics.aeroangle, aero::AERO_OUTPUT_Pbar, "Pbar");
		SimInstance1.DefineDataLogging(Modelist.aerodynamics.aeroangle, aero::AERO_OUTPUT_Qbar, "Qbar");
		SimInstance1.DefineDataLogging(Modelist.aerodynamics.aeroangle, aero::AERO_OUTPUT_Rbar, "Rbar");
		SimInstance1.DefineDataLogging(Modelist.aerodynamics.aeroangle, aero::AERO_OUTPUT_GAMMA, "gamma");
		SimInstance1.DefineDataLogging(Modelist.aerodynamics.aeroforcemoment, aero::AEROFORCE_OUTPUT_FBx, "FBx");
		SimInstance1.DefineDataLogging(Modelist.aerodynamics.aeroforcemoment, aero::AEROFORCE_OUTPUT_FBy, "FBy");
		SimInstance1.DefineDataLogging(Modelist.aerodynamics.aeroforcemoment, aero::AEROFORCE_OUTPUT_FBz, "FBz");
		SimInstance1.DefineDataLogging(Modelist.aerodynamics.aeroforcemoment, aero::AEROFORCE_OUTPUT_MBx, "MBx");
		SimInstance1.DefineDataLogging(Modelist.aerodynamics.aeroforcemoment, aero::AEROFORCE_OUTPUT_MBy, "MBy");
		SimInstance1.DefineDataLogging(Modelist.aerodynamics.aeroforcemoment, aero::AEROFORCE_OUTPUT_MBz, "MBz");

		SimInstance1.DefineDataLogging(Modelist.engine.propeller, propulsionsystem::PROPELLER_OUTPUT_T, "Thrust");
		SimInstance1.DefineDataLogging(Modelist.engine.propeller, propulsionsystem::PROPELLER_OUTPUT_Q, "TorqueRequired");
		SimInstance1.DefineDataLogging(Modelist.engine.propeller, propulsionsystem::PROPELLER_OUTPUT_CT, "CT");
		SimInstance1.DefineDataLogging(Modelist.engine.propeller, propulsionsystem::PROPELLER_OUTPUT_CP, "CP");
		SimInstance1.DefineDataLogging(Modelist.engine.omega2rps, 0, "Shaftrps");
		SimInstance1.DefineDataLogging(Modelist.engine.pistonengine, propulsionsystem::PISTONENGINE_OUTPUT_Q, "TorqueAvaliable");
		SimInstance1.DefineDataLogging(Modelist.engine.pistonengine, propulsionsystem::PISTONENGINE_OUTPUT_FUELRATE, "FuelRate");

		SimInstance1.DefineDataLogging(Modelist.engine.thrustforce, mathauxiliary::VECTOR_X, "Tbx");
		SimInstance1.DefineDataLogging(Modelist.engine.thrustforce, mathauxiliary::VECTOR_Y, "Tby");
		SimInstance1.DefineDataLogging(Modelist.engine.thrustforce, mathauxiliary::VECTOR_Z, "Tbz");

		SimInstance1.DefineDataLogging(Modelist.dynamics.sumtotalinertialforce, mathauxiliary::VECTOR_X, "TOTALX");
		SimInstance1.DefineDataLogging(Modelist.dynamics.sumtotalinertialforce, mathauxiliary::VECTOR_Y, "TOTALY");
		SimInstance1.DefineDataLogging(Modelist.dynamics.sumtotalinertialforce, mathauxiliary::VECTOR_Z, "TOTALZ");
	}
	void AircraftDynamicModel::ConnectSystems()
	{
		/*---------------------------------- connect subsystems ------------------------------*/
		// signal output to dynamics:
		// connect dynamics to the input forces
		SimInstance1.BatchEditConnectionMatrix(Modelist.dynamics.planedynamics, dynamics::DYNAMICS_INPUT_FIx, dynamics::DYNAMICS_INPUT_FIz,  Modelist.dynamics.sumtotalinertialforce, mathauxiliary::VECTOR_X, mathauxiliary::VECTOR_Z);
		SimInstance1.BatchEditConnectionMatrix(Modelist.dynamics.planedynamics, dynamics::DYNAMICS_INPUT_TBx, dynamics::DYNAMICS_INPUT_TBz , Modelist.aerodynamics.aeroforcemoment, aero::AEROFORCE_OUTPUT_MBx, aero::AEROFORCE_OUTPUT_MBz);
		// connect dynamics to the angular velocity
		SimInstance1.BatchEditConnectionMatrix(Modelist.dynamics.planedynamics, dynamics::DYNAMICS_INPUT_OmegaBIx, dynamics::DYNAMICS_INPUT_OmegaBIz, Modelist.dynamics.planekinematics, dynamics::KINEMATICS_OUTPUT_OmegaBIx, dynamics::KINEMATICS_OUTPUT_OmegaBIz);
		// dynamics to kinematics
		SimInstance1.EditConnectionMatrix(Modelist.dynamics.planekinematics, dynamics::KINEMATICS_INPUT_OMEGA_DOTx, Modelist.dynamics.planedynamics, dynamics::DYNAMICS_OUTPUT_OMEGA_DOTx);
		SimInstance1.EditConnectionMatrix(Modelist.dynamics.planekinematics, dynamics::KINEMATICS_INPUT_OMEGA_DOTy, Modelist.dynamics.planedynamics, dynamics::DYNAMICS_OUTPUT_OMEGA_DOTy);
		SimInstance1.EditConnectionMatrix(Modelist.dynamics.planekinematics, dynamics::KINEMATICS_INPUT_OMEGA_DOTz, Modelist.dynamics.planedynamics, dynamics::DYNAMICS_OUTPUT_OMEGA_DOTz);
		SimInstance1.BatchEditConnectionMatrix(Modelist.dynamics.planekinematics, dynamics::KINEMATICS_INPUT_AIx, dynamics::KINEMATICS_INPUT_AIz, Modelist.dynamics.planedynamics, dynamics::DYNAMICS_OUTPUT_AIx, dynamics::DYNAMICS_OUTPUT_AIz);
		// height block
		SimInstance1.EditConnectionMatrix(Modelist.dynamics.height, 0, Modelist.dynamics.planekinematics, dynamics::KINEMATICS_OUTPUT_XIz);
		// climbrate
		SimInstance1.EditConnectionMatrix(Modelist.dynamics.climbrate, 0, Modelist.dynamics.planekinematics, dynamics::KINEMATICS_OUTPUT_VIz);

		// gravity in body-fixed frame (multiply the inertial gravity with the rotation matrix to get the gravity in the body-fixed frame)
		SimInstance1.BatchEditConnectionMatrix(Modelist.dynamics.gravitybody, 0, 8, Modelist.dynamics.planekinematics, dynamics::KINEMATICS_OUTPUT_R_BI00, dynamics::KINEMATICS_OUTPUT_R_BI22);
		SimInstance1.BatchEditConnectionMatrix(Modelist.dynamics.gravitybody, 9, 11, Modelist.dynamics.gravityinertial, 0, 2);
		// calculate Vb X omegaB
		SimInstance1.BatchEditConnectionMatrix(Modelist.dynamics.crossproduct, mathauxiliary::VECTOR_X, mathauxiliary::VECTOR_Z, Modelist.dynamics.planekinematics, dynamics::KINEMATICS_OUTPUT_VBx, dynamics::KINEMATICS_OUTPUT_VBz);
		SimInstance1.BatchEditConnectionMatrix(Modelist.dynamics.crossproduct, 3 + mathauxiliary::VECTOR_X, 3 + mathauxiliary::VECTOR_Z, Modelist.dynamics.planekinematics, dynamics::KINEMATICS_OUTPUT_OmegaBIx, dynamics::KINEMATICS_OUTPUT_OmegaBIz);
		// calculate R_BI VI_dot
		SimInstance1.BatchEditConnectionMatrix(Modelist.dynamics.product, 0, 8, Modelist.dynamics.planekinematics, dynamics::KINEMATICS_OUTPUT_R_BI00, dynamics::KINEMATICS_OUTPUT_R_BI22);
		SimInstance1.BatchEditConnectionMatrix(Modelist.dynamics.product, 9, 11, Modelist.dynamics.planedynamics, dynamics::DYNAMICS_OUTPUT_AIx, dynamics::DYNAMICS_OUTPUT_AIz);
		// calculate Vb_dot = R_BI VI_dot + Vb X omegaB
		SimInstance1.BatchEditConnectionMatrix(Modelist.dynamics.Vbdot, 0, 2, Modelist.dynamics.product, mathauxiliary::VECTOR_X, mathauxiliary::VECTOR_Z);
		SimInstance1.BatchEditConnectionMatrix(Modelist.dynamics.Vbdot, 3, 5, Modelist.dynamics.crossproduct, mathauxiliary::VECTOR_X, mathauxiliary::VECTOR_Z);

		// filter the body acc
		SimInstance1.EditConnectionMatrix(Modelist.dynamics.ACCxfilter, 0, Modelist.dynamics.Vbdot, mathauxiliary::VECTOR_X);
		SimInstance1.EditConnectionMatrix(Modelist.dynamics.ACCyfilter, 0, Modelist.dynamics.Vbdot, mathauxiliary::VECTOR_Y);
		SimInstance1.EditConnectionMatrix(Modelist.dynamics.ACCzfilter, 0, Modelist.dynamics.Vbdot, mathauxiliary::VECTOR_Z);

		// atmoshpere block to the height of the aircraft
		SimInstance1.EditConnectionMatrix(Modelist.aerodynamics.atmosphere, 0, Modelist.dynamics.height, 0);

		// connect the aero angle block
		SimInstance1.BatchEditConnectionMatrix(Modelist.aerodynamics.aeroangle, aero::AERO_INPUT_R_BI00, aero::AERO_INPUT_R_BI22, Modelist.dynamics.planekinematics, dynamics::KINEMATICS_OUTPUT_R_BI00, dynamics::KINEMATICS_OUTPUT_R_BI22);
		SimInstance1.EditConnectionMatrix(Modelist.aerodynamics.aeroangle, aero::AERO_INPUT_HORIZONTALWINDSPEED, simulationcontrol::external, 0); // leave the wind speed as external for now
		SimInstance1.EditConnectionMatrix(Modelist.aerodynamics.aeroangle, aero::AERO_INPUT_WINDDIRECTION,       simulationcontrol::external, 0); // leave the wind direction as external for now
		SimInstance1.EditConnectionMatrix(Modelist.aerodynamics.aeroangle, aero::AERO_INPUT_VERTICALWINDSPEED,   simulationcontrol::external, 0); // leave the vertical wind as external for now
		SimInstance1.EditConnectionMatrix(Modelist.aerodynamics.aeroangle, aero::AERO_INPUT_RHO, Modelist.aerodynamics.atmosphere, geographic::AtmDensity);
		SimInstance1.EditConnectionMatrix(Modelist.aerodynamics.aeroangle, aero::AERO_INPUT_SOUNDSPEED, Modelist.aerodynamics.atmosphere, geographic::AtmSoundSpeed);
		SimInstance1.BatchEditConnectionMatrix(Modelist.aerodynamics.aeroangle, aero::AERO_INPUT_P, aero::AERO_INPUT_R, Modelist.dynamics.planekinematics, dynamics::KINEMATICS_OUTPUT_OmegaBIx, dynamics::KINEMATICS_OUTPUT_OmegaBIz);
		SimInstance1.BatchEditConnectionMatrix(Modelist.aerodynamics.aeroangle, aero::AERO_INPUT_Vbx, aero::AERO_INPUT_Vbz, Modelist.dynamics.planekinematics, dynamics::KINEMATICS_OUTPUT_VBx, dynamics::KINEMATICS_OUTPUT_VBz);
		SimInstance1.BatchEditConnectionMatrix(Modelist.aerodynamics.aeroangle, aero::AERO_INPUT_VIx, aero::AERO_INPUT_VIz, Modelist.dynamics.planekinematics, dynamics::KINEMATICS_OUTPUT_VIx, dynamics::KINEMATICS_OUTPUT_VIz);
		SimInstance1.EditConnectionMatrix(Modelist.aerodynamics.aeroangle, aero::AERO_INPUT_Vbdotx, Modelist.dynamics.ACCxfilter, 0);
		SimInstance1.EditConnectionMatrix(Modelist.aerodynamics.aeroangle, aero::AERO_INPUT_Vbdoty, Modelist.dynamics.ACCyfilter, 0);
		SimInstance1.EditConnectionMatrix(Modelist.aerodynamics.aeroangle, aero::AERO_INPUT_Vbdotz, Modelist.dynamics.ACCzfilter, 0);

		// connect the AOArate and beta rate to filters

		SimInstance1.EditConnectionMatrix(Modelist.aerodynamics.filteredAOArate,  0, Modelist.aerodynamics.aeroangle, aero::AERO_OUTPUT_AOARATE);
		SimInstance1.EditConnectionMatrix(Modelist.aerodynamics.filteredBetarate, 0, Modelist.aerodynamics.aeroangle, aero::AERO_OUTPUT_SIDESLIPRATE);

		// connect the aero force block
		SimInstance1.EditConnectionMatrix(Modelist.aerodynamics.aeroforcemoment, aero::AEROFORCE_INPUT_AOA, Modelist.aerodynamics.aeroangle, aero::AERO_OUTPUT_AOA);
		SimInstance1.EditConnectionMatrix(Modelist.aerodynamics.aeroforcemoment, aero::AEROFORCE_INPUT_SIDESLIP, Modelist.aerodynamics.aeroangle, aero::AERO_OUTPUT_SIDESLIP);
		SimInstance1.EditConnectionMatrix(Modelist.aerodynamics.aeroforcemoment, aero::AEROFORCE_INPUT_Pbar, Modelist.aerodynamics.aeroangle, aero::AERO_OUTPUT_Pbar);
		SimInstance1.EditConnectionMatrix(Modelist.aerodynamics.aeroforcemoment, aero::AEROFORCE_INPUT_Qbar, Modelist.aerodynamics.aeroangle, aero::AERO_OUTPUT_Qbar);
		SimInstance1.EditConnectionMatrix(Modelist.aerodynamics.aeroforcemoment, aero::AEROFORCE_INPUT_Rbar, Modelist.aerodynamics.aeroangle, aero::AERO_OUTPUT_Rbar);
		SimInstance1.EditConnectionMatrix(Modelist.aerodynamics.aeroforcemoment, aero::AEROFORCE_INPUT_MACHNUMBER, Modelist.aerodynamics.aeroangle, aero::AERO_OUTPUT_MACHNUMBER);
		SimInstance1.EditConnectionMatrix(Modelist.aerodynamics.aeroforcemoment, aero::AEROFORCE_INPUT_DYNAMICPRESSURE, Modelist.aerodynamics.aeroangle, aero::AERO_OUTPUT_DYNAMICPRESSURE);

		SimInstance1.EditConnectionMatrix(Modelist.aerodynamics.aeroforcemoment, aero::AEROFORCE_INPUT_AOARATE_FILTERED, Modelist.aerodynamics.aeroangle, aero::AERO_OUTPUT_AOARATE);
		// SimInstance1.EditConnectionMatrix(Modelist.aerodynamics.aeroforcemoment, aero::AEROFORCE_INPUT_SIDESLIPRATE_FILTERED, Modelist.aerodynamics.filteredBetarate, 0);

		// connect engine block
		// connect the propeller to the shaft dynamics
		SimInstance1.EditConnectionMatrix(Modelist.engine.propeller, propulsionsystem::PROPELLER_INPUT_N, Modelist.engine.omega2rps, 0);
		SimInstance1.EditConnectionMatrix(Modelist.engine.propeller, propulsionsystem::PROPELLER_INPUT_V, Modelist.aerodynamics.aeroangle, aero::AERO_OUTPUT_TAS);
		SimInstance1.EditConnectionMatrix(Modelist.engine.propeller, propulsionsystem::PROPELLER_INPUT_RHO, Modelist.aerodynamics.atmosphere, geographic::AtmDensity);
		// connect the shaft input to the total torque
		SimInstance1.EditConnectionMatrix(Modelist.engine.omega2rps, 0, Modelist.engine.shaftdynamics, 0); // the output of the inertial from rad/s to rps
		SimInstance1.EditConnectionMatrix(Modelist.engine.shaftdynamics, 0, Modelist.engine.shaftinertia, 0);
		SimInstance1.EditConnectionMatrix(Modelist.engine.shaftinertia, 0, Modelist.engine.totaltorque, 0);
		SimInstance1.EditConnectionMatrix(Modelist.engine.totaltorque, 0, Modelist.engine.pistonengine, propulsionsystem::PISTONENGINE_OUTPUT_Q);
		SimInstance1.EditConnectionMatrix(Modelist.engine.totaltorque, 1, Modelist.engine.propeller, propulsionsystem::PROPELLER_OUTPUT_Q);
		// connect the piston engine to the shaft

		SimInstance1.EditConnectionMatrix(Modelist.engine.pistonengine, propulsionsystem::PISTONENGINE_INPUT_SHAFTRPS, Modelist.engine.omega2rps, 0);
		SimInstance1.EditConnectionMatrix(Modelist.engine.pistonengine, propulsionsystem::PISTONENGINE_INPUT_MANIFOLD, Modelist.aerodynamics.atmosphere, geographic::AtmDensity);
		SimInstance1.EditConnectionMatrix(Modelist.engine.pistonengine, propulsionsystem::PISTONENGINE_INPUT_THROTTLE, Modelist.temp.fixedthrottle, 0);
		SimInstance1.EditConnectionMatrix(Modelist.engine.pistonengine, propulsionsystem::PISTONENGINE_INPUT_MIXTURE,  Modelist.temp.fixedmixture, 0);
		SimInstance1.EditConnectionMatrix(Modelist.engine.pistonengine, propulsionsystem::PISTONENGINE_INPUT_FUELSTATE,Modelist.temp.fixedfuelstate, 0);

		// get the thrust force
		SimInstance1.EditConnectionMatrix(Modelist.engine.thrustforce, 0, Modelist.engine.propeller, propulsionsystem::PROPELLER_OUTPUT_T);
		SimInstance1.BatchEditConnectionMatrix(Modelist.engine.thrustforce, 1, 3, Modelist.engine.propellerorientation, mathauxiliary::VECTOR_X, mathauxiliary::VECTOR_Z);

		// connect all body forces
		SimInstance1.BatchEditConnectionMatrix(Modelist.dynamics.sumtotalbodyforce, 0, 2, Modelist.aerodynamics.aeroforcemoment, aero::AEROFORCE_OUTPUT_FBx, aero::AEROFORCE_OUTPUT_FBz);
		SimInstance1.BatchEditConnectionMatrix(Modelist.dynamics.sumtotalbodyforce, 3, 5, Modelist.engine.thrustforce, mathauxiliary::VECTOR_X, mathauxiliary::VECTOR_Z);

		// rotate body forces to inertial frame
		SimInstance1.BatchEditConnectionMatrix(Modelist.dynamics.rotation2inertialframe, 0, 8, Modelist.dynamics.planekinematics, dynamics::KINEMATICS_OUTPUT_R_IB00, dynamics::KINEMATICS_OUTPUT_R_IB22);
		SimInstance1.BatchEditConnectionMatrix(Modelist.dynamics.rotation2inertialframe, 9, 11, Modelist.dynamics.sumtotalbodyforce, mathauxiliary::VECTOR_X, mathauxiliary::VECTOR_Z);
	
		// connect all inertial forces
		SimInstance1.BatchEditConnectionMatrix(Modelist.dynamics.sumtotalinertialforce, 0, 2, Modelist.dynamics.rotation2inertialframe, mathauxiliary::VECTOR_X, mathauxiliary::VECTOR_Z);
		SimInstance1.BatchEditConnectionMatrix(Modelist.dynamics.sumtotalinertialforce, 3, 5, Modelist.dynamics.gravityinertial, mathauxiliary::VECTOR_X, mathauxiliary::VECTOR_Z);	

	}
}