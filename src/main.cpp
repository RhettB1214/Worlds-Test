#include "main.h"
#include "lemlib/api.hpp"
#include "definitions.hpp"



/*Variable Defintions*/

	/*Controller Variable Definitions*/
		bool lastKnownStateR1 =	false;
		bool lastKnownStateR2 = false;
		bool lastKnownStateUp = false;
	/*End of Controller Variable Definitions*/	
	

	/*Pnuematic Toggle Variable Definitions*/
		bool leftWingToggle = false;
		bool rightWingToggle = false;
		bool ptoToggle = false;
	/*End of Pneumatic Toggle Variable Defintions*/

/*End of Variable Defintions*/


/*Device Defintions*/
	/*Motor Defintions*/
		pros::Motor fLD (FLD, SPEEDBOX, false);
		pros::Motor mLD (MLD, SPEEDBOX, false);
		pros::Motor bLD (BLD, SPEEDBOX, false);
		pros::Motor fRD (FRD, SPEEDBOX, true);
		pros::Motor mRD (MRD, SPEEDBOX, true);
		pros::Motor bRD (BRD, SPEEDBOX, true);

		pros::Motor lIM (LIM, SPEEDBOX, true);
		pros::Motor rIM (RIM, SPEEDBOX, false);
	/*End of Motor Defintions*/

	/*Motor Group Definitions*/
		pros::MotorGroup lDrive({fLD, mLD, bLD});
		pros::MotorGroup rDrive({fRD, mRD, bRD});
		
		pros::MotorGroup intake({lIM, rIM});
	/*End of Motor Group Definitions*/

	/*Sensor Definitions*/
		pros::Imu imu(IMU_PORT);

		pros::Rotation HoriOdom(HODOM_ROT, false);
		pros::Rotation VertOdom(VODOM_ROT, false);
	/*End of Sensor Definitions*/

	/*ADI Definitions*/
		pros::ADIDigitalOut leftHoriWing(LEFTW_ADIDO);
		pros::ADIDigitalOut rightHoriWing(RIGHTW_ADIDO);
		pros::ADIDigitalOut vertWing(VERTW_ADIDO);

		pros::ADIDigitalOut ptoPiston(PTO_ADIDO);
	/*End of ADI Defintions*/

	/*Controller Definitions*/
		pros::Controller master(pros::E_CONTROLLER_MASTER);
	/*End of Controller Definitions*/
/*End of Device Defintions*/

/*LemLib Chassis Definitions*/

	/*LemLib Chassis Sensor Defintions*/
		lemlib::TrackingWheel HoriWheel(&HoriOdom, 2, 1.5, 1);
		lemlib::TrackingWheel VertWheel(&VertOdom, 2, 0.25, 1);
	/*End of LemLib Chassis Sensor Defintions*/

	/*LemLib Drivetrain Initilization*/
		lemlib::Drivetrain drivetrain
		{
			&lDrive, /*Pointer to the left drive channel*/
			&rDrive, /*Pointer to the right drive channel*/
			10, /*Track Width*/
			2.75, /*Wheel Diameter*/
			600, /*Wheel RPM*/
			8 /*Chase Power*/
		};
	/*End of LemLib Drivetrain Initilization*/

	/*LemLib Odometry Initilization*/
		lemlib::OdomSensors odomSensors
		{
			&VertWheel, /*Vertical Tracking Wheel*/
			nullptr, /*No Tracking Wheel*/
			&HoriWheel, /*Horizontal Tracking Wheel*/
			nullptr, /*No Tracking Wheel*/
			&imu /*Inertial Sensor*/
		};
	/*End of LemLib Odometry Initilization*/

	/*Lateral (Forwards/Backwards) PID Initilization*/
		lemlib::ControllerSettings lateralController
		(
			8,  //16, // kP
			0, //3 // kI
			32, //80, // kD
			0, //4 // Windup Range
			1, // smallErrorRange
			100, // smallErrorTimeout
			3, // largeErrorRange
			500, // largeErrorTimeout
			10 // Slew Rate
		);
	/*End of Lateral (Forwards/Backwards) PID Initilization*/


	/*Angular (Turning) PID Initilization*/
		lemlib::ControllerSettings angularController
		(
			4,  //7 // kP
			0, // kI
			40, //60 // kD
			0, // Windup Range
			1, // smallErrorRange
			100, // smallErrorTimeout
			3, // largeErrorRange
			500, // largeErrorTimeout
			10 // Slew Rate
		);
	/*End of Angular (Turning) PID Initilization*/

	/*Throttle Curve Initialzition*/
		lemlib::ExpoDriveCurve throttleCurve
		(
			0, //Joystick Deadband
			0, //Minimum output where dt will move out of 127
			1.012 //Expo Curve Gains
		);
	/*End of Throttle Curve Initialzition*/


	/*LemLib Chassis Initilization*/
		lemlib::Chassis drive(drivetrain, lateralController, angularController, odomSensors, &throttleCurve);
	/*End of LemLib Chassis Initilization*/


/*End of LemLib Chassis Initializations*/



/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() 
{
	drive.calibrate();
	intake.set_brake_modes(HOLD);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() 
{
	lDrive.set_brake_modes(HOLD);
	rDrive.set_brake_modes(HOLD);
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() 
{
	lDrive.set_brake_modes(COAST);
	rDrive.set_brake_modes(COAST);

	while(true)
	{

		/*Drive train control*/
		drive.tank(masterLeftY, masterRightY);


		/*Intake control*/
		if (masterL1) /*If Left bumper is pressed*/
		{
			/*Outtake*/
			intake.move(127);
		}
		else if (masterL2) /*If Left trigger is pressed*/
		{	
			/*Intake*/
			intake.move(-127);
		}
		else
		{
			/*Do nothing*/
			intake.move(0);
		}

		/*Wing Control*/
		if (masterR1 != lastKnownStateR1) //Left Wing Control
		{
			lastKnownStateR1 = masterR1;
			if (masterR1)
			{
				leftWingToggle = !leftWingToggle; //Toggles the variable that controls the left wing when R1 is pressed
				leftHoriWing.set_value(leftWingToggle); //Sets the left wing to the toggled value
			}

		}
		if (masterR2 != lastKnownStateR2) //Right Wing Control
		{
			lastKnownStateR2 = masterR2;
			if (masterR2)
			{
				rightWingToggle = !rightWingToggle; //Toggles the variable that controls the right wing when R2 is pressed
				rightHoriWing.set_value(rightWingToggle); //Sets the right wing to the toggled value
			}

		}
		if (masterB) //Both Wing Hold Control
		{
			rightHoriWing.set_value(true); //Sets the right wing to true
			leftHoriWing.set_value(true); //Sets the left wing to true
			rightWingToggle = false; //Sets the right wing to false
			leftWingToggle = false; //Sets the left wing to false
		}
		else if(masterB == false) 
		{
			rightHoriWing.set_value(rightWingToggle); //Sets the right wing to the toggled value
			leftHoriWing.set_value(leftWingToggle); //Sets the left wing to the toggled value
		}

		/*PTO Control*/
		if (masterUp != lastKnownStateUp) //PTO Control
		{
			if (masterUp)
			{
				ptoToggle = !ptoToggle; //Toggles the variable that controls the PTO when Up is pressed
				ptoPiston.set_value(ptoToggle); //Sets the PTO to the toggled value
			}

		}


	}
	
}
