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

		DefineLandingGear();

		DefineAutopilot();

		ConnectSystems();

		SimInstance1.FlushMakeConnection();
		// define logging
		DefineLogging();
		// run preprocess

		SimInstance1.DisplaySystemParameter(Modelist.dynamics.loadfactorfluy);

		modelok = SimInstance1.PreRunProcess();
		// get the external input mapping for specific inputs
		speedcommandindex     = SimInstance1.GetExternalInputIndex(Modelist.autothrottle.SumTASError, 1);
		trimthrottle          = SimInstance1.GetExternalInputIndex(Modelist.autothrottle.SumTotoalThrottle, 1);
		autothrottlePIDenable = SimInstance1.GetExternalInputIndex(Modelist.autothrottle.PIDThrottleCom, 0);
		altcommand            = SimInstance1.GetExternalInputIndex(Modelist.pitchCAS.SumAltitError, 0);
		nosegearswitch        = SimInstance1.GetExternalInputIndex(Modelist.landinggear.NoseGearNormalForce, groundcontact::GEARNORMAL_INPUT_SWITCH);
		leftgearswitch        = SimInstance1.GetExternalInputIndex(Modelist.landinggear.LeftGearNormalForce, groundcontact::GEARNORMAL_INPUT_SWITCH);
		rightgeaswitch        = SimInstance1.GetExternalInputIndex(Modelist.landinggear.RightGearNormalForce, groundcontact::GEARNORMAL_INPUT_SWITCH);
		nosegearbreak         = SimInstance1.GetExternalInputIndex(Modelist.landinggear.NoseGearFrictionForce, groundcontact::LUGREFRICTION_INPUT_BREAKING);
		leftgearbreak         = SimInstance1.GetExternalInputIndex(Modelist.landinggear.LeftGearFrictionForce, groundcontact::LUGREFRICTION_INPUT_BREAKING);
		rightgearbreak        = SimInstance1.GetExternalInputIndex(Modelist.landinggear.RightGearFrictionForce, groundcontact::LUGREFRICTION_INPUT_BREAKING);
		steering              = SimInstance1.GetExternalInputIndex(Modelist.landinggear.NoseGearNormalForce, groundcontact::GEARNORMAL_INPUT_STEERING);

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
			// determine autopilot state
			if (input.autopilot.autopilotmaster) {
				// master off

			}
			else {
				extern_input(speedcommandindex) =     input.autopilot.autothrottle.targetspeed;
				extern_input(trimthrottle) =          input.autopilot.autothrottle.trimthrottle;
				extern_input(autothrottlePIDenable) = input.autopilot.autothrottle.ON;
				extern_input(altcommand) =            input.autopilot.pitchCAS.commandaltitude;
			}
			// load gear switch
			if (input.gear.geardown) {
				extern_input(nosegearswitch) = 1.0;
				extern_input(leftgearswitch) = 1.0;
				extern_input(rightgeaswitch) = 1.0;
			}
			else {
				extern_input(nosegearswitch) = -1.0;
				extern_input(leftgearswitch) = -1.0;
				extern_input(rightgeaswitch) = -1.0;
			}
			// load break
			if (input.gear.gearbreak) {
				extern_input(nosegearbreak)  = 1.0;
				extern_input(leftgearbreak)  = 1.0;
				extern_input(rightgearbreak) = 1.0;
			} else {
				extern_input(nosegearbreak) = -1.0;
				extern_input(leftgearbreak) = -1.0;
				extern_input(rightgearbreak) = -1.0;
			}
			// load steering
			extern_input(steering) = input.gear.steering;

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
		gravity_param.value(2) = gravityacc *dynamics_parameter.m;
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
		sum_total_force_parameter.SignList.push_back(mathblocks::SUM_POSITIVE); // gear
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
		// acceleration filter, to 
		linearsystem::TransferFunctionParameter ACCxfilterparam;
		ACCxfilterparam.Numerator.resize(1);
		ACCxfilterparam.Numerator(0) = 30.0;
		ACCxfilterparam.Denominator.resize(2);
		ACCxfilterparam.Denominator(0) = 1.0;
		ACCxfilterparam.Denominator(1) = 30.0;
		// use the same parameters for all three filters
		Modelist.dynamics.ACCxfilter= SimInstance1.AddSubSystem(ACCxfilterparam);
		Modelist.dynamics.ACCyfilter = SimInstance1.AddSubSystem(ACCxfilterparam);
		Modelist.dynamics.ACCzfilter = SimInstance1.AddSubSystem(ACCxfilterparam);
		// the total force
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
		// the current mass TO DO: change it to varying mass later..
		mathblocks::ConstantParameter planecurrentmass_param;
		planecurrentmass_param.value.resize(1);
		planecurrentmass_param.value(0) = 1.0 /(gravityacc*dynamics_parameter.m);
		Modelist.dynamics.planecurrentweight = SimInstance1.AddSubSystem(planecurrentmass_param);
		// calculate the load factor in body-fixed frame
		mathblocks::MultiplicationParam loadfactorbody_param;
		loadfactorbody_param.Mode = mathblocks::MULTI_SCALAR;
		loadfactorbody_param.input1_dimension(mathblocks::LOOKUP_INPUT_ROW) = 1;
		loadfactorbody_param.input1_dimension(mathblocks::LOOKUP_INPUT_COL) = 1;
		loadfactorbody_param.input2_dimension(mathblocks::LOOKUP_INPUT_ROW) = 3;
		loadfactorbody_param.input2_dimension(mathblocks::LOOKUP_INPUT_COL) = 1;
		Modelist.dynamics.loadfactor = SimInstance1.AddSubSystem(loadfactorbody_param);
		// invert the y and z value to get the load factor in Forward-Left-Up frame
		mathblocks::GainParameter  loadfactorfluy_param;
		loadfactorfluy_param.K.resize(1, 1);
		loadfactorfluy_param.K(0, 0) = -1.0;
		loadfactorfluy_param.Mode = mathblocks::ElementWise;
		loadfactorfluy_param.num_of_inputs = 1;
		Modelist.dynamics.loadfactorfluy = SimInstance1.AddSubSystem(loadfactorfluy_param);
		Modelist.dynamics.loadfactorfluz = SimInstance1.AddSubSystem(loadfactorfluy_param); // use the same paramter for z direction
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
		// define the subsystem for pitch autopilot

		mathblocks::GainParameter GainThetadot1_param;
		GainThetadot1_param.K.resize(1, 1);
		GainThetadot1_param.K(0, 0) = parameter.autopilot.pitchCAS.GainThetadot1;
		GainThetadot1_param.Mode = mathblocks::ElementWise;
		GainThetadot1_param.num_of_inputs = 1;
		Modelist.pitchCAS.GainThetadot1 = SimInstance1.AddSubSystem(GainThetadot1_param);

		mathblocks::GainParameter GainThetadot2_param;
		GainThetadot2_param.K.resize(1, 1);
		GainThetadot2_param.K(0, 0) = parameter.autopilot.pitchCAS.GainThetadot2;
		GainThetadot2_param.Mode = mathblocks::ElementWise;
		GainThetadot2_param.num_of_inputs = 1;
		Modelist.pitchCAS.GainThetadot2 = SimInstance1.AddSubSystem(GainThetadot2_param);

		mathblocks::GainParameter GainDeCom_param;
		GainDeCom_param.K.resize(1, 1);
		GainDeCom_param.K(0, 0) = parameter.autopilot.pitchCAS.GainDeCom;
		GainDeCom_param.Mode = mathblocks::ElementWise;
		GainDeCom_param.num_of_inputs = 1;
		Modelist.pitchCAS.GainDeCom = SimInstance1.AddSubSystem(GainDeCom_param);

		linearsystem::IntegratorParameter Deintegral_param;
		Deintegral_param.num_of_channels = 1;
		linearsystem::IntegratorInitialCondition Deintegral_IC;
		Deintegral_IC.X_0.resize(1);
		Deintegral_IC.X_0(0) = 0.0;
		Modelist.pitchCAS.Deintegral = SimInstance1.AddSubSystem(Deintegral_param, Deintegral_IC);

		discontinuoussystem::SaturationParameter SaturationDeIntegral_param;
		SaturationDeIntegral_param.type = discontinuoussystem::SATURATION_BOTH;
		SaturationDeIntegral_param.num_of_channels = 1;
		SaturationDeIntegral_param.lower_bound = -parameter.autopilot.pitchCAS.SaturationDeIntegral;
		SaturationDeIntegral_param.upper_bound = parameter.autopilot.pitchCAS.SaturationDeIntegral;
		Modelist.pitchCAS.SaturationDeIntegral = SimInstance1.AddSubSystem(SaturationDeIntegral_param);

		mathblocks::GainParameter GainDeIntegral_param;
		GainDeIntegral_param.K.resize(1, 1);
		GainDeIntegral_param.K(0, 0) = parameter.autopilot.pitchCAS.GainDeIntegral;
		GainDeIntegral_param.Mode = mathblocks::ElementWise;
		GainDeIntegral_param.num_of_inputs = 1;
		Modelist.pitchCAS.GainDeIntegral = SimInstance1.AddSubSystem(GainDeIntegral_param);

		mathblocks::SumParameter  SumDeCom_param;
		SumDeCom_param.input_dimensions = 1;
		SumDeCom_param.SignList.push_back(mathblocks::SUM_POSITIVE);
		SumDeCom_param.SignList.push_back(mathblocks::SUM_NEGATIVE);
		Modelist.pitchCAS.SumDeCom = SimInstance1.AddSubSystem(SumDeCom_param);

		mathblocks::SumParameter  SumDeltaDz_param;
		SumDeltaDz_param.input_dimensions = 1;
		SumDeltaDz_param.SignList.push_back(mathblocks::SUM_NEGATIVE);
		SumDeltaDz_param.SignList.push_back(mathblocks::SUM_POSITIVE);
		Modelist.pitchCAS.SumDeltaDz = SimInstance1.AddSubSystem(SumDeltaDz_param);

		mathblocks::DivisionParameter  ProductGammaPhi_param;
		ProductGammaPhi_param.SignList.push_back(mathblocks::DIVISION_PRODUCT);
		ProductGammaPhi_param.SignList.push_back(mathblocks::DIVISION_DIVISION);
		Modelist.pitchCAS.ProductGammaPhi = SimInstance1.AddSubSystem(ProductGammaPhi_param);

		mathblocks::TrigonometryParameter CosGamma_param;
		CosGamma_param.num_of_channels = 2;
		CosGamma_param.type = mathblocks::COS;
		Modelist.pitchCAS.CosGammaRoll = SimInstance1.AddSubSystem(CosGamma_param);

		mathblocks::SumParameter  SumCstar1_param;
		SumCstar1_param.input_dimensions = 1;
		SumCstar1_param.SignList.push_back(mathblocks::SUM_POSITIVE);
		SumCstar1_param.SignList.push_back(mathblocks::SUM_NEGATIVE);
		SumCstar1_param.SignList.push_back(mathblocks::SUM_NEGATIVE);
		Modelist.pitchCAS.SumCstar1 = SimInstance1.AddSubSystem(SumCstar1_param);

		discontinuoussystem::SaturationParameter cosphilimit_param;
		cosphilimit_param.type = discontinuoussystem::SATURATION_BOTH;
		cosphilimit_param.num_of_channels = 1;
		cosphilimit_param.lower_bound = 0.5;
		cosphilimit_param.upper_bound = 1.0;
		Modelist.pitchCAS.SaturationCosPhi = SimInstance1.AddSubSystem(cosphilimit_param);

		// define the altitude holding
		mathblocks::GainParameter GainVS_param;
		GainVS_param.K.resize(1, 1);
		GainVS_param.K(0,0) = parameter.autopilot.pitchCAS.VSGain;
		GainVS_param.Mode = mathblocks::ElementWise;
		GainVS_param.num_of_inputs = 1;
		Modelist.pitchCAS.GainVS = SimInstance1.AddSubSystem(GainVS_param);

		mathblocks::GainParameter GainAltitError_param;
		GainAltitError_param.K.resize(1, 1);
		GainAltitError_param.K(0, 0) = parameter.autopilot.pitchCAS.AltitudeErrorToVSGain;
		GainAltitError_param.Mode = mathblocks::ElementWise;
		GainAltitError_param.num_of_inputs = 1;
		Modelist.pitchCAS.GainAltitError = SimInstance1.AddSubSystem(GainAltitError_param);

		discontinuoussystem::SaturationParameter SaturationAltit_param;
		SaturationAltit_param.type = discontinuoussystem::SATURATION_BOTH;
		SaturationAltit_param.num_of_channels = 1;
		SaturationAltit_param.upper_bound = parameter.autopilot.pitchCAS.AltitudeErrorLimit;
		SaturationAltit_param.lower_bound = -parameter.autopilot.pitchCAS.AltitudeErrorLimit;
		Modelist.pitchCAS.SaturationAltit = SimInstance1.AddSubSystem(SaturationAltit_param);

		mathblocks::SumParameter SumAltitError_param;
		SumAltitError_param.input_dimensions = 1;
		SumAltitError_param.SignList.push_back(mathblocks::SUM_POSITIVE);
		SumAltitError_param.SignList.push_back(mathblocks::SUM_NEGATIVE);
		Modelist.pitchCAS.SumAltitError = SimInstance1.AddSubSystem(SumAltitError_param);
	

		mathblocks::SumParameter  SumVSError_param;
		SumVSError_param.input_dimensions = 1;
		SumVSError_param.SignList.push_back(mathblocks::SUM_POSITIVE);
		SumVSError_param.SignList.push_back(mathblocks::SUM_NEGATIVE);
		Modelist.pitchCAS.SumVSError = SimInstance1.AddSubSystem(SumVSError_param);


		// define the auto throttle

		mathblocks::SumParameter  SumTASError_param;
		SumTASError_param.input_dimensions = 1;
		SumTASError_param.SignList.push_back(mathblocks::SUM_NEGATIVE);
		SumTASError_param.SignList.push_back(mathblocks::SUM_POSITIVE);
		Modelist.autothrottle.SumTASError = SimInstance1.AddSubSystem(SumTASError_param);

		discontinuoussystem::SaturationParameter SaturateTASError_param;
		SaturateTASError_param.type = discontinuoussystem::SATURATION_BOTH;
		SaturateTASError_param.num_of_channels = 1;
		SaturateTASError_param.upper_bound = parameter.autopilot.autothrottle.TASErrorLimit;
		SaturateTASError_param.lower_bound = - parameter.autopilot.autothrottle.TASErrorLimit;
		Modelist.autothrottle.SaturateTASError = SimInstance1.AddSubSystem(SaturateTASError_param);

		mathblocks::GainParameter GainTASError_param;
		GainTASError_param.K.resize(1, 1);
		GainTASError_param.K(0, 0) = parameter.autopilot.autothrottle.TASErrorGain;
		GainTASError_param.Mode = mathblocks::ElementWise;
		GainTASError_param.num_of_inputs = 1;
		Modelist.autothrottle.GainTASError = SimInstance1.AddSubSystem(GainTASError_param);

		linearsystem::PIDcontrollerParameter PIDThrottleCom_param;
		PIDThrottleCom_param.integration_control_on = false;
		PIDThrottleCom_param.Kd = parameter.autopilot.autothrottle.Kd;
		PIDThrottleCom_param.Ki = parameter.autopilot.autothrottle.Ki;
		PIDThrottleCom_param.Kp = parameter.autopilot.autothrottle.Kp;
		PIDThrottleCom_param.num_of_channels = 1;
		PIDThrottleCom_param.Tf = 100.0;
		Modelist.autothrottle.PIDThrottleCom = SimInstance1.AddSubSystem(PIDThrottleCom_param);

		mathblocks::SumParameter SumTotoalThrottle_param;
		SumTotoalThrottle_param.input_dimensions = 1;
		SumTotoalThrottle_param.SignList.push_back(mathblocks::SUM_POSITIVE);
		SumTotoalThrottle_param.SignList.push_back(mathblocks::SUM_POSITIVE);
		Modelist.autothrottle.SumTotoalThrottle = SimInstance1.AddSubSystem(SumTotoalThrottle_param);


	}
	void AircraftDynamicModel::DefineLandingGear()
	{
		// define landing gear modual
		groundcontact::SimpleGearNormalForceParameter nose_gear_param;
		nose_gear_param.MinNz = parameter.gear.nosegear.param.NeMin;
		nose_gear_param.MaxHeight = parameter.gear.nosegear.param.Hmax;
		nose_gear_param.kCompress = parameter.gear.nosegear.param.Stiffness;
		nose_gear_param.dRebound = parameter.gear.nosegear.param.ReboundDamping;
		nose_gear_param.dCompress = parameter.gear.nosegear.param.CompressDamping;
		nose_gear_param.GearDirection(mathauxiliary::VECTOR_X) = parameter.gear.nosegear.param.Nex;
		nose_gear_param.GearDirection(mathauxiliary::VECTOR_Y) = parameter.gear.nosegear.param.Ney;
		nose_gear_param.GearDirection(mathauxiliary::VECTOR_Z) = parameter.gear.nosegear.param.Nez;
		nose_gear_param.GearPosition(mathauxiliary::VECTOR_X) = parameter.gear.nosegear.param.Rex;
		nose_gear_param.GearPosition(mathauxiliary::VECTOR_Y) = parameter.gear.nosegear.param.Rey;
		nose_gear_param.GearPosition(mathauxiliary::VECTOR_Z) = parameter.gear.nosegear.param.Rez;
		nose_gear_param.isSteering = true;
		Modelist.landinggear.NoseGearNormalForce = SimInstance1.AddSubSystem(nose_gear_param);

		groundcontact::SimpleGearNormalForceParameter left_gear_param;
		left_gear_param.MinNz = parameter.gear.leftgear.param.NeMin;
		left_gear_param.MaxHeight = parameter.gear.leftgear.param.Hmax;
		left_gear_param.kCompress = parameter.gear.leftgear.param.Stiffness;
		left_gear_param.dRebound = parameter.gear.leftgear.param.ReboundDamping;
		left_gear_param.dCompress = parameter.gear.leftgear.param.CompressDamping;
		left_gear_param.GearDirection(mathauxiliary::VECTOR_X) = parameter.gear.leftgear.param.Nex;
		left_gear_param.GearDirection(mathauxiliary::VECTOR_Y) = parameter.gear.leftgear.param.Ney;
		left_gear_param.GearDirection(mathauxiliary::VECTOR_Z) = parameter.gear.leftgear.param.Nez;
		left_gear_param.GearPosition(mathauxiliary::VECTOR_X) = parameter.gear.leftgear.param.Rex;
		left_gear_param.GearPosition(mathauxiliary::VECTOR_Y) = parameter.gear.leftgear.param.Rey;
		left_gear_param.GearPosition(mathauxiliary::VECTOR_Z) = parameter.gear.leftgear.param.Rez;
		left_gear_param.isSteering = false;
		Modelist.landinggear.LeftGearNormalForce = SimInstance1.AddSubSystem(left_gear_param);

		groundcontact::SimpleGearNormalForceParameter right_gear_param;
		right_gear_param.MinNz = parameter.gear.rightgear.param.NeMin;
		right_gear_param.MaxHeight = parameter.gear.rightgear.param.Hmax;
		right_gear_param.kCompress = parameter.gear.rightgear.param.Stiffness;
		right_gear_param.dRebound = parameter.gear.rightgear.param.ReboundDamping;
		right_gear_param.dCompress = parameter.gear.rightgear.param.CompressDamping;
		right_gear_param.GearDirection(mathauxiliary::VECTOR_X) = parameter.gear.rightgear.param.Nex;
		right_gear_param.GearDirection(mathauxiliary::VECTOR_Y) = parameter.gear.rightgear.param.Ney;
		right_gear_param.GearDirection(mathauxiliary::VECTOR_Z) = parameter.gear.rightgear.param.Nez;
		right_gear_param.GearPosition(mathauxiliary::VECTOR_X) = parameter.gear.rightgear.param.Rex;
		right_gear_param.GearPosition(mathauxiliary::VECTOR_Y) = parameter.gear.rightgear.param.Rey;
		right_gear_param.GearPosition(mathauxiliary::VECTOR_Z) = parameter.gear.rightgear.param.Rez;
		right_gear_param.isSteering = false;
		Modelist.landinggear.RightGearNormalForce = SimInstance1.AddSubSystem(right_gear_param);

		groundcontact::GearLuGreFrictionParameter nose_gear_friction_param;
		nose_gear_friction_param.StiffnessSigma0 = parameter.gear.rightgear.param.Sigma0;
		nose_gear_friction_param.DampingSigma1 = sqrt(nose_gear_friction_param.StiffnessSigma0);
		nose_gear_friction_param.SwitchSigmaD = 1.0 / parameter.gear.rightgear.param.VrelaxationRoll;
		nose_gear_friction_param.SwitchSigmaS = 1.0 / parameter.gear.rightgear.param.VrelaxationRoll;
		nose_gear_friction_param.DynamicFrictionCoefficient = parameter.gear.rightgear.param.SigmaDynamic;
		nose_gear_friction_param.RollingFrictionCoefficient = parameter.gear.rightgear.param.SigmaRoll;
		nose_gear_friction_param.StaticFrictionCoefficient = parameter.gear.rightgear.param.SigmaStatic;
		nose_gear_friction_param.vlimit = parameter.gear.rightgear.param.Vlimit;


		Modelist.landinggear.NoseGearFrictionForce = SimInstance1.AddSubSystem(nose_gear_friction_param);
		Modelist.landinggear.LeftGearFrictionForce = SimInstance1.AddSubSystem(nose_gear_friction_param);
		Modelist.landinggear.RightGearFrictionForce = SimInstance1.AddSubSystem(nose_gear_friction_param);



		mathblocks::MultiplicationParam gear_force_to_inertial_param;
		gear_force_to_inertial_param.input1_dimension(mathblocks::MATRIX_ROW) = 3;
		gear_force_to_inertial_param.input1_dimension(mathblocks::MATRIX_COL) = 3;
		gear_force_to_inertial_param.input2_dimension(mathblocks::MATRIX_ROW) = 3;
		gear_force_to_inertial_param.input2_dimension(mathblocks::MATRIX_COL) = 1;
		gear_force_to_inertial_param.Mode = mathblocks::MULTI_MATRIX;

		Modelist.landinggear.GearMomentToBody = SimInstance1.AddSubSystem(gear_force_to_inertial_param);

		mathblocks::SumParameter total_gear_force_param;
		total_gear_force_param.input_dimensions = 3;
		total_gear_force_param.SignList.push_back(mathblocks::SUM_POSITIVE);
		total_gear_force_param.SignList.push_back(mathblocks::SUM_POSITIVE);
		total_gear_force_param.SignList.push_back(mathblocks::SUM_POSITIVE);

		Modelist.landinggear.TotalGearForce = SimInstance1.AddSubSystem(total_gear_force_param);
		Modelist.landinggear.TotalGearMoment = SimInstance1.AddSubSystem(total_gear_force_param);

		mathblocks::ConstantParameter Ng_param;
		Ng_param.value.resize(3, 1);
		Ng_param.value(mathauxiliary::VECTOR_X) = 0.0;
		Ng_param.value(mathauxiliary::VECTOR_Y) = 0.0;
		Ng_param.value(mathauxiliary::VECTOR_Z) = -1.0;

		Modelist.landinggear.GroundNormal = SimInstance1.AddSubSystem(Ng_param);


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

		SimInstance1.DefineDataLogging(Modelist.dynamics.planekinematics, dynamics::KINEMATICS_OUTPUT_VBx, "Vbx");
		SimInstance1.DefineDataLogging(Modelist.dynamics.planekinematics, dynamics::KINEMATICS_OUTPUT_VBy, "Vby");
		SimInstance1.DefineDataLogging(Modelist.dynamics.planekinematics, dynamics::KINEMATICS_OUTPUT_VBz, "Vbz");

		SimInstance1.DefineDataLogging(Modelist.dynamics.planekinematics, dynamics::KINEMATICS_OUTPUT_VIx, "VIx");
		SimInstance1.DefineDataLogging(Modelist.dynamics.planekinematics, dynamics::KINEMATICS_OUTPUT_VIy, "VIy");
		SimInstance1.DefineDataLogging(Modelist.dynamics.planekinematics, dynamics::KINEMATICS_OUTPUT_VIz, "VIz");

		SimInstance1.DefineDataLogging(Modelist.dynamics.planekinematics, dynamics::KINEMATICS_OUTPUT_XIx, "XIx");
		SimInstance1.DefineDataLogging(Modelist.dynamics.planekinematics, dynamics::KINEMATICS_OUTPUT_XIy, "XIy");
		SimInstance1.DefineDataLogging(Modelist.dynamics.planekinematics, dynamics::KINEMATICS_OUTPUT_XIz, "XIz");
		SimInstance1.DefineDataLogging(Modelist.dynamics.climbrate, 0, "climbrate");
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

		SimInstance1.DefineDataLogging(Modelist.dynamics.loadfactor, mathauxiliary::VECTOR_X, "loadfactorbodyx");
		SimInstance1.DefineDataLogging(Modelist.dynamics.loadfactorfluy, 0, "loadfactorbodyy");
		SimInstance1.DefineDataLogging(Modelist.dynamics.loadfactorfluz, 0, "loadfactorbodyz");

		SimInstance1.DefineDataLogging(Modelist.autothrottle.SumTotoalThrottle, 0, "ThrottleCmd");
		SimInstance1.DefineDataLogging(Modelist.dynamics.height, 0, "height");
	}
	void AircraftDynamicModel::ConnectSystems()
	{
		/*---------------------------------- connect subsystems ------------------------------*/
		// signal output to dynamics:
		// connect dynamics to the input forces
		SimInstance1.BatchEditConnectionMatrix(Modelist.dynamics.planedynamics, dynamics::DYNAMICS_INPUT_FIx, dynamics::DYNAMICS_INPUT_FIz,  Modelist.dynamics.sumtotalinertialforce, mathauxiliary::VECTOR_X, mathauxiliary::VECTOR_Z);
		SimInstance1.BatchEditConnectionMatrix(Modelist.dynamics.planedynamics, dynamics::DYNAMICS_INPUT_TBx, dynamics::DYNAMICS_INPUT_TBz , Modelist.dynamics.sumtotalbodymoment, aero::AEROFORCE_OUTPUT_MBx, aero::AEROFORCE_OUTPUT_MBz);
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
		//SimInstance1.EditConnectionMatrix(Modelist.engine.pistonengine, propulsionsystem::PISTONENGINE_INPUT_THROTTLE, Modelist.temp.fixedthrottle, 0);
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
		SimInstance1.BatchEditConnectionMatrix(Modelist.dynamics.sumtotalinertialforce, 6, 8, Modelist.landinggear.TotalGearForce, mathauxiliary::VECTOR_X, mathauxiliary::VECTOR_Z);
		
		// connect all body moments
		SimInstance1.BatchEditConnectionMatrix(Modelist.dynamics.sumtotalbodymoment, 0, 2, Modelist.aerodynamics.aeroforcemoment, aero::AEROFORCE_OUTPUT_MBx, aero::AEROFORCE_OUTPUT_MBz);
		SimInstance1.BatchEditConnectionMatrix(Modelist.dynamics.sumtotalbodymoment, 3, 5, Modelist.landinggear.GearMomentToBody, 0, 2);
		
		// connect load factor
		SimInstance1.BatchEditConnectionMatrix(Modelist.dynamics.loadfactor, 1, 3, Modelist.dynamics.sumtotalbodyforce, mathauxiliary::VECTOR_X, mathauxiliary::VECTOR_Z);
		SimInstance1.EditConnectionMatrix(Modelist.dynamics.loadfactor, 0, Modelist.dynamics.planecurrentweight, 0);
		
		// invert the y and z components of the load factor
		SimInstance1.EditConnectionMatrix(Modelist.dynamics.loadfactorfluy, 0, Modelist.dynamics.loadfactor, mathauxiliary::VECTOR_Y);
		SimInstance1.EditConnectionMatrix(Modelist.dynamics.loadfactorfluz, 0, Modelist.dynamics.loadfactor, mathauxiliary::VECTOR_Z);

		// connect the pitch CAS
		SimInstance1.EditConnectionMatrix(Modelist.pitchCAS.CosGammaRoll, 0, Modelist.aerodynamics.aeroangle, aero::AERO_OUTPUT_GAMMA);// 0 for gamma
		SimInstance1.EditConnectionMatrix(Modelist.pitchCAS.CosGammaRoll, 1, Modelist.dynamics.planekinematics, dynamics::KINEMATICS_OUTPUT_EulerRoll);// 1 for roll
		SimInstance1.EditConnectionMatrix(Modelist.pitchCAS.SaturationCosPhi, 0, Modelist.pitchCAS.CosGammaRoll, 1);
		SimInstance1.EditConnectionMatrix(Modelist.pitchCAS.ProductGammaPhi, 0, Modelist.pitchCAS.CosGammaRoll, 0);
		SimInstance1.EditConnectionMatrix(Modelist.pitchCAS.ProductGammaPhi, 1, Modelist.pitchCAS.SaturationCosPhi, 0); // 1 from saturation
		SimInstance1.EditConnectionMatrix(Modelist.pitchCAS.SumDeltaDz, 0, Modelist.pitchCAS.ProductGammaPhi, 0);
		SimInstance1.EditConnectionMatrix(Modelist.pitchCAS.SumDeltaDz, 1, Modelist.dynamics.loadfactorfluz, 0);
		SimInstance1.EditConnectionMatrix(Modelist.pitchCAS.GainThetadot1, 0, Modelist.dynamics.planekinematics, dynamics::KINEMATICS_OUTPUT_THETADOT);
		SimInstance1.EditConnectionMatrix(Modelist.pitchCAS.GainThetadot2, 0, Modelist.dynamics.planekinematics, dynamics::KINEMATICS_OUTPUT_THETADOT);
		//
		
		SimInstance1.EditConnectionMatrix(Modelist.pitchCAS.SumCstar1, 0, Modelist.pitchCAS.GainDeCom, 0);
		SimInstance1.EditConnectionMatrix(Modelist.pitchCAS.SumCstar1, 1, Modelist.pitchCAS.GainThetadot2, 0);
		SimInstance1.EditConnectionMatrix(Modelist.pitchCAS.SumCstar1, 2, Modelist.pitchCAS.SumDeltaDz, 0);
		// 
		SimInstance1.EditConnectionMatrix(Modelist.pitchCAS.Deintegral, 0, Modelist.pitchCAS.SumCstar1, 0);
		SimInstance1.EditConnectionMatrix(Modelist.pitchCAS.SaturationDeIntegral, 0, Modelist.pitchCAS.Deintegral, 0);
		SimInstance1.EditConnectionMatrix(Modelist.pitchCAS.SumDeCom, 0, Modelist.pitchCAS.GainThetadot1, 0);
		SimInstance1.EditConnectionMatrix(Modelist.pitchCAS.SumDeCom, 1, Modelist.pitchCAS.SaturationDeIntegral, 0);
		// connect the controller to aircraft input
		SimInstance1.EditConnectionMatrix(Modelist.aerodynamics.aeroforcemoment, aero::AEROFORCE_INPUT_ELEVATOR, Modelist.pitchCAS.SumDeCom,0);
		// connect the 
		SimInstance1.EditConnectionMatrix(Modelist.autothrottle.SumTASError, 0, Modelist.aerodynamics.aeroangle, aero::AERO_OUTPUT_TAS);
		SimInstance1.EditConnectionMatrix(Modelist.autothrottle.SumTASError, 1, simulationcontrol::external, 0);

		SimInstance1.EditConnectionMatrix(Modelist.autothrottle.SaturateTASError, 0, Modelist.autothrottle.SumTASError, 0);
		SimInstance1.EditConnectionMatrix(Modelist.autothrottle.GainTASError, 0, Modelist.autothrottle.SaturateTASError, 0);
		SimInstance1.EditConnectionMatrix(Modelist.autothrottle.PIDThrottleCom, 0, simulationcontrol::external, 0);
		SimInstance1.EditConnectionMatrix(Modelist.autothrottle.PIDThrottleCom, 1, Modelist.autothrottle.GainTASError, 0);

		SimInstance1.EditConnectionMatrix(Modelist.autothrottle.SumTotoalThrottle, 0, Modelist.autothrottle.PIDThrottleCom, 0);
		SimInstance1.EditConnectionMatrix(Modelist.autothrottle.SumTotoalThrottle, 1, simulationcontrol::external, 0);
		SimInstance1.EditConnectionMatrix(Modelist.engine.pistonengine, propulsionsystem::PISTONENGINE_INPUT_THROTTLE, Modelist.autothrottle.SumTotoalThrottle, 0);
		// connect the altitude controller
		SimInstance1.EditConnectionMatrix(Modelist.pitchCAS.SumAltitError, 0, simulationcontrol::external, 0);
		SimInstance1.EditConnectionMatrix(Modelist.pitchCAS.SumAltitError, 1, Modelist.dynamics.height, 0);
		SimInstance1.EditConnectionMatrix(Modelist.pitchCAS.SaturationAltit, 0, Modelist.pitchCAS.SumAltitError, 0);
		SimInstance1.EditConnectionMatrix(Modelist.pitchCAS.GainAltitError, 0, Modelist.pitchCAS.SaturationAltit, 0);
		SimInstance1.EditConnectionMatrix(Modelist.pitchCAS.SumVSError, 0, Modelist.pitchCAS.GainAltitError, 0);
		SimInstance1.EditConnectionMatrix(Modelist.pitchCAS.SumVSError, 1, Modelist.dynamics.climbrate, 0);
		SimInstance1.EditConnectionMatrix(Modelist.pitchCAS.GainVS, 0, Modelist.pitchCAS.SumVSError, 0);
		SimInstance1.EditConnectionMatrix(Modelist.pitchCAS.GainDeCom, 0, Modelist.pitchCAS.GainVS, 0);


		// connect landing gear normal force to aircraft
		SimInstance1.EditConnectionMatrix(Modelist.landinggear.NoseGearNormalForce, groundcontact::GEARNORMAL_INPUT_STEERING, simulationcontrol::external, 0);
		SimInstance1.EditConnectionMatrix(Modelist.landinggear.NoseGearNormalForce, groundcontact::GEARNORMAL_INPUT_SWITCH, simulationcontrol::external, 0);
		SimInstance1.EditConnectionMatrix(Modelist.landinggear.NoseGearNormalForce, groundcontact::GEARNORMAL_INPUT_H, simulationcontrol::external, 0);
		SimInstance1.EditConnectionMatrix(Modelist.landinggear.NoseGearNormalForce, groundcontact::GEARNORMAL_INPUT_Hdot, simulationcontrol::external, 0);
		SimInstance1.BatchEditConnectionMatrix(Modelist.landinggear.NoseGearNormalForce, groundcontact::GEARNORMAL_INPUT_NGx, groundcontact::GEARNORMAL_INPUT_NGz, Modelist.landinggear.GroundNormal, mathauxiliary::VECTOR_X, mathauxiliary::VECTOR_Z);
		SimInstance1.BatchEditConnectionMatrix(Modelist.landinggear.NoseGearNormalForce, groundcontact::GEARNORMAL_INPUT_OMEGAbx, groundcontact::GEARNORMAL_INPUT_OMEGAbz, Modelist.dynamics.planekinematics, dynamics::KINEMATICS_OUTPUT_OmegaBIx, dynamics::KINEMATICS_OUTPUT_OmegaBIz);
		SimInstance1.BatchEditConnectionMatrix(Modelist.landinggear.NoseGearNormalForce, groundcontact::GEARNORMAL_INPUT_PIx, groundcontact::GEARNORMAL_INPUT_PIz, Modelist.dynamics.planekinematics, dynamics::KINEMATICS_OUTPUT_XIx, dynamics::KINEMATICS_OUTPUT_XIz);
		SimInstance1.BatchEditConnectionMatrix(Modelist.landinggear.NoseGearNormalForce, groundcontact::GEARNORMAL_INPUT_R_IB00, groundcontact::GEARNORMAL_INPUT_R_IB22, Modelist.dynamics.planekinematics, dynamics::KINEMATICS_OUTPUT_R_IB00, dynamics::KINEMATICS_OUTPUT_R_IB22);
		SimInstance1.BatchEditConnectionMatrix(Modelist.landinggear.NoseGearNormalForce, groundcontact::GEARNORMAL_INPUT_VIx, groundcontact::GEARNORMAL_INPUT_VIz, Modelist.dynamics.planekinematics, dynamics::KINEMATICS_OUTPUT_VIx, dynamics::KINEMATICS_OUTPUT_VIz);

		SimInstance1.EditConnectionMatrix(Modelist.landinggear.LeftGearNormalForce, groundcontact::GEARNORMAL_INPUT_SWITCH, simulationcontrol::external, 0);
		SimInstance1.EditConnectionMatrix(Modelist.landinggear.LeftGearNormalForce, groundcontact::GEARNORMAL_INPUT_H, simulationcontrol::external, 0);
		SimInstance1.EditConnectionMatrix(Modelist.landinggear.LeftGearNormalForce, groundcontact::GEARNORMAL_INPUT_Hdot, simulationcontrol::external, 0);
		SimInstance1.BatchEditConnectionMatrix(Modelist.landinggear.LeftGearNormalForce, groundcontact::GEARNORMAL_INPUT_NGx, groundcontact::GEARNORMAL_INPUT_NGz, Modelist.landinggear.GroundNormal, mathauxiliary::VECTOR_X, mathauxiliary::VECTOR_Z);
		SimInstance1.BatchEditConnectionMatrix(Modelist.landinggear.LeftGearNormalForce, groundcontact::GEARNORMAL_INPUT_OMEGAbx, groundcontact::GEARNORMAL_INPUT_OMEGAbz, Modelist.dynamics.planekinematics, dynamics::KINEMATICS_OUTPUT_OmegaBIx, dynamics::KINEMATICS_OUTPUT_OmegaBIz);
		SimInstance1.BatchEditConnectionMatrix(Modelist.landinggear.LeftGearNormalForce, groundcontact::GEARNORMAL_INPUT_PIx, groundcontact::GEARNORMAL_INPUT_PIz, Modelist.dynamics.planekinematics, dynamics::KINEMATICS_OUTPUT_XIx, dynamics::KINEMATICS_OUTPUT_XIz);
		SimInstance1.BatchEditConnectionMatrix(Modelist.landinggear.LeftGearNormalForce, groundcontact::GEARNORMAL_INPUT_R_IB00, groundcontact::GEARNORMAL_INPUT_R_IB22, Modelist.dynamics.planekinematics, dynamics::KINEMATICS_OUTPUT_R_IB00, dynamics::KINEMATICS_OUTPUT_R_IB22);
		SimInstance1.BatchEditConnectionMatrix(Modelist.landinggear.LeftGearNormalForce, groundcontact::GEARNORMAL_INPUT_VIx, groundcontact::GEARNORMAL_INPUT_VIz, Modelist.dynamics.planekinematics, dynamics::KINEMATICS_OUTPUT_VIx, dynamics::KINEMATICS_OUTPUT_VIz);

		SimInstance1.EditConnectionMatrix(Modelist.landinggear.RightGearNormalForce, groundcontact::GEARNORMAL_INPUT_SWITCH, simulationcontrol::external, 0);
		SimInstance1.EditConnectionMatrix(Modelist.landinggear.RightGearNormalForce, groundcontact::GEARNORMAL_INPUT_H, simulationcontrol::external, 0);
		SimInstance1.EditConnectionMatrix(Modelist.landinggear.RightGearNormalForce, groundcontact::GEARNORMAL_INPUT_Hdot, simulationcontrol::external, 0);
		SimInstance1.BatchEditConnectionMatrix(Modelist.landinggear.RightGearNormalForce, groundcontact::GEARNORMAL_INPUT_NGx, groundcontact::GEARNORMAL_INPUT_NGz, Modelist.landinggear.GroundNormal, mathauxiliary::VECTOR_X, mathauxiliary::VECTOR_Z);
		SimInstance1.BatchEditConnectionMatrix(Modelist.landinggear.RightGearNormalForce, groundcontact::GEARNORMAL_INPUT_OMEGAbx, groundcontact::GEARNORMAL_INPUT_OMEGAbz, Modelist.dynamics.planekinematics, dynamics::KINEMATICS_OUTPUT_OmegaBIx, dynamics::KINEMATICS_OUTPUT_OmegaBIz);
		SimInstance1.BatchEditConnectionMatrix(Modelist.landinggear.RightGearNormalForce, groundcontact::GEARNORMAL_INPUT_PIx, groundcontact::GEARNORMAL_INPUT_PIz, Modelist.dynamics.planekinematics, dynamics::KINEMATICS_OUTPUT_XIx, dynamics::KINEMATICS_OUTPUT_XIz);
		SimInstance1.BatchEditConnectionMatrix(Modelist.landinggear.RightGearNormalForce, groundcontact::GEARNORMAL_INPUT_R_IB00, groundcontact::GEARNORMAL_INPUT_R_IB22, Modelist.dynamics.planekinematics, dynamics::KINEMATICS_OUTPUT_R_IB00, dynamics::KINEMATICS_OUTPUT_R_IB22);
		SimInstance1.BatchEditConnectionMatrix(Modelist.landinggear.RightGearNormalForce, groundcontact::GEARNORMAL_INPUT_VIx, groundcontact::GEARNORMAL_INPUT_VIz, Modelist.dynamics.planekinematics, dynamics::KINEMATICS_OUTPUT_VIx, dynamics::KINEMATICS_OUTPUT_VIz);

		// connect the landing gear friction to normal force
		SimInstance1.BatchEditConnectionMatrix(Modelist.landinggear.NoseGearFrictionForce, groundcontact::LUGREFRICTION_INPUT_CPx, groundcontact::LUGREFRICTION_INPUT_CPz, Modelist.landinggear.NoseGearNormalForce, groundcontact::GEARNORMAL_OUTPUT_CONTACTPOINTx, groundcontact::GEARNORMAL_OUTPUT_CONTACTPOINTz);
		SimInstance1.BatchEditConnectionMatrix(Modelist.landinggear.NoseGearFrictionForce, groundcontact::LUGREFRICTION_INPUT_NIx, groundcontact::LUGREFRICTION_INPUT_NIz, Modelist.landinggear.NoseGearNormalForce, groundcontact::GEARNORMAL_OUTPUT_NIx, groundcontact::GEARNORMAL_OUTPUT_NIz);
		SimInstance1.BatchEditConnectionMatrix(Modelist.landinggear.NoseGearFrictionForce, groundcontact::LUGREFRICTION_INPUT_Npx, groundcontact::LUGREFRICTION_INPUT_Npz, Modelist.landinggear.NoseGearNormalForce, groundcontact::GEARNORMAL_OUTPUT_npIx, groundcontact::GEARNORMAL_OUTPUT_npIz);
		SimInstance1.BatchEditConnectionMatrix(Modelist.landinggear.NoseGearFrictionForce, groundcontact::LUGREFRICTION_INPUT_Nwx, groundcontact::LUGREFRICTION_INPUT_Nwz, Modelist.landinggear.NoseGearNormalForce, groundcontact::GEARNORMAL_OUTPUT_nwIx, groundcontact::GEARNORMAL_OUTPUT_nwIz);
		SimInstance1.EditConnectionMatrix(Modelist.landinggear.NoseGearFrictionForce, groundcontact::LUGREFRICTION_INPUT_VpI, Modelist.landinggear.NoseGearNormalForce, groundcontact::GEARNORMAL_OUTPUT_VGIp);
		SimInstance1.EditConnectionMatrix(Modelist.landinggear.NoseGearFrictionForce, groundcontact::LUGREFRICTION_INPUT_VwI, Modelist.landinggear.NoseGearNormalForce, groundcontact::GEARNORMAL_OUTPUT_VGIw);
		SimInstance1.EditConnectionMatrix(Modelist.landinggear.NoseGearFrictionForce, groundcontact::LUGREFRICTION_INPUT_BREAKING, simulationcontrol::external, 0);

		// connect the left gear friction

		SimInstance1.BatchEditConnectionMatrix(Modelist.landinggear.LeftGearFrictionForce, groundcontact::LUGREFRICTION_INPUT_CPx, groundcontact::LUGREFRICTION_INPUT_CPz, Modelist.landinggear.LeftGearNormalForce, groundcontact::GEARNORMAL_OUTPUT_CONTACTPOINTx, groundcontact::GEARNORMAL_OUTPUT_CONTACTPOINTz);
		SimInstance1.BatchEditConnectionMatrix(Modelist.landinggear.LeftGearFrictionForce, groundcontact::LUGREFRICTION_INPUT_NIx, groundcontact::LUGREFRICTION_INPUT_NIz, Modelist.landinggear.LeftGearNormalForce, groundcontact::GEARNORMAL_OUTPUT_NIx, groundcontact::GEARNORMAL_OUTPUT_NIz);
		SimInstance1.BatchEditConnectionMatrix(Modelist.landinggear.LeftGearFrictionForce, groundcontact::LUGREFRICTION_INPUT_Npx, groundcontact::LUGREFRICTION_INPUT_Npz, Modelist.landinggear.LeftGearNormalForce, groundcontact::GEARNORMAL_OUTPUT_npIx, groundcontact::GEARNORMAL_OUTPUT_npIz);
		SimInstance1.BatchEditConnectionMatrix(Modelist.landinggear.LeftGearFrictionForce, groundcontact::LUGREFRICTION_INPUT_Nwx, groundcontact::LUGREFRICTION_INPUT_Nwz, Modelist.landinggear.LeftGearNormalForce, groundcontact::GEARNORMAL_OUTPUT_nwIx, groundcontact::GEARNORMAL_OUTPUT_nwIz);
		SimInstance1.EditConnectionMatrix(Modelist.landinggear.LeftGearFrictionForce, groundcontact::LUGREFRICTION_INPUT_VpI, Modelist.landinggear.LeftGearNormalForce, groundcontact::GEARNORMAL_OUTPUT_VGIp);
		SimInstance1.EditConnectionMatrix(Modelist.landinggear.LeftGearFrictionForce, groundcontact::LUGREFRICTION_INPUT_VwI, Modelist.landinggear.LeftGearNormalForce, groundcontact::GEARNORMAL_OUTPUT_VGIw);
		SimInstance1.EditConnectionMatrix(Modelist.landinggear.LeftGearFrictionForce, groundcontact::LUGREFRICTION_INPUT_BREAKING, simulationcontrol::external, 0);
		// connect the right gear friction
		SimInstance1.BatchEditConnectionMatrix(Modelist.landinggear.RightGearFrictionForce, groundcontact::LUGREFRICTION_INPUT_CPx, groundcontact::LUGREFRICTION_INPUT_CPz, Modelist.landinggear.RightGearNormalForce, groundcontact::GEARNORMAL_OUTPUT_CONTACTPOINTx, groundcontact::GEARNORMAL_OUTPUT_CONTACTPOINTz);
		SimInstance1.BatchEditConnectionMatrix(Modelist.landinggear.RightGearFrictionForce, groundcontact::LUGREFRICTION_INPUT_NIx, groundcontact::LUGREFRICTION_INPUT_NIz, Modelist.landinggear.RightGearNormalForce, groundcontact::GEARNORMAL_OUTPUT_NIx, groundcontact::GEARNORMAL_OUTPUT_NIz);
		SimInstance1.BatchEditConnectionMatrix(Modelist.landinggear.RightGearFrictionForce, groundcontact::LUGREFRICTION_INPUT_Npx, groundcontact::LUGREFRICTION_INPUT_Npz, Modelist.landinggear.RightGearNormalForce, groundcontact::GEARNORMAL_OUTPUT_npIx, groundcontact::GEARNORMAL_OUTPUT_npIz);
		SimInstance1.BatchEditConnectionMatrix(Modelist.landinggear.RightGearFrictionForce, groundcontact::LUGREFRICTION_INPUT_Nwx, groundcontact::LUGREFRICTION_INPUT_Nwz, Modelist.landinggear.RightGearNormalForce, groundcontact::GEARNORMAL_OUTPUT_nwIx, groundcontact::GEARNORMAL_OUTPUT_nwIz);
		SimInstance1.EditConnectionMatrix(Modelist.landinggear.RightGearFrictionForce, groundcontact::LUGREFRICTION_INPUT_VpI, Modelist.landinggear.RightGearNormalForce, groundcontact::GEARNORMAL_OUTPUT_VGIp);
		SimInstance1.EditConnectionMatrix(Modelist.landinggear.RightGearFrictionForce, groundcontact::LUGREFRICTION_INPUT_VwI, Modelist.landinggear.RightGearNormalForce, groundcontact::GEARNORMAL_OUTPUT_VGIw);
		SimInstance1.EditConnectionMatrix(Modelist.landinggear.RightGearFrictionForce, groundcontact::LUGREFRICTION_INPUT_BREAKING, simulationcontrol::external, 0);

		SimInstance1.BatchEditConnectionMatrix(Modelist.landinggear.TotalGearForce, mathauxiliary::VECTOR_X, mathauxiliary::VECTOR_Z, Modelist.landinggear.NoseGearFrictionForce, groundcontact::LUGREFRICTION_OUTPUT_FIx, groundcontact::LUGREFRICTION_OUTPUT_FIz);
		SimInstance1.BatchEditConnectionMatrix(Modelist.landinggear.TotalGearForce, mathauxiliary::VECTOR_X + 3, mathauxiliary::VECTOR_Z + 3, Modelist.landinggear.LeftGearFrictionForce, groundcontact::LUGREFRICTION_OUTPUT_FIx, groundcontact::LUGREFRICTION_OUTPUT_FIz);
		SimInstance1.BatchEditConnectionMatrix(Modelist.landinggear.TotalGearForce, mathauxiliary::VECTOR_X + 6, mathauxiliary::VECTOR_Z + 6, Modelist.landinggear.RightGearFrictionForce, groundcontact::LUGREFRICTION_OUTPUT_FIx, groundcontact::LUGREFRICTION_OUTPUT_FIz);

		SimInstance1.BatchEditConnectionMatrix(Modelist.landinggear.TotalGearMoment, mathauxiliary::VECTOR_X, mathauxiliary::VECTOR_Z, Modelist.landinggear.NoseGearFrictionForce, groundcontact::LUGREFRICTION_OUTPUT_MIx, groundcontact::LUGREFRICTION_OUTPUT_MIz);
		SimInstance1.BatchEditConnectionMatrix(Modelist.landinggear.TotalGearMoment, mathauxiliary::VECTOR_X + 3, mathauxiliary::VECTOR_Z + 3, Modelist.landinggear.LeftGearFrictionForce, groundcontact::LUGREFRICTION_OUTPUT_MIx, groundcontact::LUGREFRICTION_OUTPUT_MIz);
		SimInstance1.BatchEditConnectionMatrix(Modelist.landinggear.TotalGearMoment, mathauxiliary::VECTOR_X + 6, mathauxiliary::VECTOR_Z + 6, Modelist.landinggear.RightGearFrictionForce, groundcontact::LUGREFRICTION_OUTPUT_MIx, groundcontact::LUGREFRICTION_OUTPUT_MIz);

		SimInstance1.BatchEditConnectionMatrix(Modelist.landinggear.GearMomentToBody, 0, 8,  Modelist.dynamics.planekinematics, dynamics::KINEMATICS_OUTPUT_R_BI00, dynamics::KINEMATICS_OUTPUT_R_BI22);
		SimInstance1.BatchEditConnectionMatrix(Modelist.landinggear.GearMomentToBody, 9, 11, Modelist.landinggear.TotalGearMoment, mathauxiliary::VECTOR_X, mathauxiliary::VECTOR_Z);

	}
}