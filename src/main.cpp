#include "main.h"
#include "lemlib/api.hpp"
#include "definitions.hpp"


ASSET(AWP1_txt);
ASSET(AWP2_txt);
ASSET(RedFar1_txt);
ASSET(RedFar2_txt);
ASSET(RedFar3_txt);
ASSET(disrupt_txt);

/*Variable Defintions*/

	/*Controller Variable Definitions*/
		bool lastKnownStateR1 =	false;
		bool lastKnownStateR2 = false;
		bool lastKnownStateRight = false;
		bool lastKnownStateB = false;
		bool lastKnownStateDown = false;
	/*End of Controller Variable Definitions*/	
	

	/*Pneumatic Toggle Variable Definitions*/
		bool leftWingToggle = false;
		bool rightWingToggle = false;
		bool vertToggle = false;
		bool ptoToggle = false;
		bool hangReleased = false;
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

		pros::Motor lIM (LIM, SPEEDBOX, false);
		pros::Motor rIM (RIM, SPEEDBOX, true);
	/*End of Motor Defintions*/

	/*Motor Group Definitions*/
		pros::MotorGroup lDrive({fLD, mLD, bLD});
		pros::MotorGroup rDrive({fRD, mRD, bRD});
		
		pros::MotorGroup intake({lIM, rIM});
	/*End of Motor Group Definitions*/

	/*Sensor Definitions*/
		pros::Imu imu(IMU_PORT);

		pros::Rotation HoriOdom(HODOM_ROT, true);
		pros::Rotation VertOdom(VODOM_ROT, true);
	/*End of Sensor Definitions*/

	/*ADI Definitions*/
		pros::ADIDigitalOut leftHoriWing(LEFTW_ADIDO);
		pros::ADIDigitalOut rightHoriWing(RIGHTW_ADIDO);
		pros::ADIDigitalOut vertWing(VERTW_ADIDO);

		pros::ADIDigitalOut ptoPiston(PTO_ADIDO);
		pros::ADIDigitalOut hangPiston(HANG_ADIDO);
	/*End of ADI Defintions*/

	/*Controller Definitions*/
		pros::Controller master(pros::E_CONTROLLER_MASTER);
	/*End of Controller Definitions*/
/*End of Device Defintions*/

/*LemLib Chassis Definitions*/

	/*LemLib Chassis Sensor Defintions*/
		lemlib::TrackingWheel HoriWheel(&HoriOdom, 2.125, 1.5, 1);
		lemlib::TrackingWheel VertWheel(&VertOdom, 2.125, 0.25, 1);
	/*End of LemLib Chassis Sensor Defintions*/

	/*LemLib Drivetrain Initilization*/
		lemlib::Drivetrain drivetrain
		{
			&lDrive, /*Pointer to the left drive channel*/
			&rDrive, /*Pointer to the right drive channel*/
			10, /*Track Width*/
			2.75, /*Wheel Diameter*/
			600, /*Wheel RPM*/
			3 /*Chase Power*/
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
			10,  //16, // kP
			0, //3 // kI
			43, //80, // kD
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
			60.35, //60 // kD
			0, // Windup Range
			1, // smallErrorRange
			100, // smallErrorTimeout
			1, // largeErrorRange
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

void screen() {
    // loop forever
    while (true) {
        lemlib::Pose pose = drive.getPose(); // get the current position of the robot
        pros::lcd::print(0, "x: %f", pose.x); // print the x position
        pros::lcd::print(1, "y: %f", pose.y); // print the y position
        pros::lcd::print(2, "heading: %f", pose.theta); // print the heading
        pros::delay(10);
    }
}



/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() 
{
	pros::lcd::initialize(); // initialize brain screen
	drive.calibrate();
	intake.set_brake_modes(COAST);
	pros::Task screenTask(screen); // create a task to print the position to the screen
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
	drive.setBrakeMode(HOLD);

	//Elim Disruptor Auton
	drive.setPose(-36, -61, 180);
	
	intake.move(127);
	drive.moveToPoint(-36, -18, 1000, {.forwards = false, .minSpeed = 100});
	drive.turnToHeading(270, 300);
	drive.waitUntilDone();
	vertWing.set_value(1);
	intake.move(-127);
	drive.moveToPoint(-5, -18, 1000, {.forwards = false, .minSpeed = 100});
	drive.waitUntilDone();
	intake.move(0);
	drive.moveToPoint(-12, -18, 1000);
	drive.turnToHeading(90, 500);
	drive.moveToPoint(-50, -18, 1000, {.forwards = false, .minSpeed = 100});
	drive.moveToPoint(-25, -18, 500);
	drive.turnToHeading(270, 500);




	/*
	//5 Ball Rush Auton
	drive.setPose(48.026, -55.261, 318);
	
	//Hit Preload to the side of the goal with Wings
	rightHoriWing.set_value(1);
	pros::delay(100);
	rightHoriWing.set_value(0);

	//Start intaking and grab center bar triball
	intake.move(100);
	drive.follow(RedFar1_txt, 15, 1500);
	drive.waitUntilDone();
	drive.turnToHeading(270, 300);
	drive.waitUntilDone();
	intake.move(0);

	
	//Deploy vertical wings and score center offensive triball
	vertWing.set_value(1);
	pros::delay(100);
	drive.moveToPoint(50, 0, 1000, {.forwards = false});
	drive.waitUntilDone();

	//Back up, retract vertical wings, and score triball that's in the intake
	drive.moveToPoint(25, 0, 300);
	drive.waitUntilDone();
	vertWing.set_value(0);
	pros::delay(400);
	drive.moveToPoint(50, 0, 1000);
	drive.waitUntil(5);
	intake.move(-127);
	drive.waitUntilDone();


	//Grab offensive bar triball
	drive.moveToPoint(35, 0, 500, {.forwards = false});
	drive.waitUntilDone();
	intake.move(127);
	drive.turnToPoint(35, -60, 500);
	drive.follow(RedFar2_txt, 15, 1250);
	

	//Drive to matchload bar and remove triball that starts in the ml zone
	drive.moveToPoint(25, -24, 500, {.forwards = false});
	intake.move(0);
	drive.turnToPoint(65, -65, 500);
	drive.waitUntilDone();
	pros::delay(250);
	drive.moveToPose(56, -44.25, 135, 1500);
	drive.turnToHeading(45, 500);
	drive.waitUntilDone();
	vertWing.set_value(1);
	pros::delay(200);
	drive.turnToPoint(-12, 60, 500);
	drive.waitUntilDone();

	//Outtake the triball thats in the intake and score the three triballs on the side of the Goal
	vertWing.set_value(0);
	pros::delay(200);
	rightHoriWing.set_value(1);
	leftHoriWing.set_value(1);
	drive.turnToHeading(35, 500, {.maxSpeed = 80});
	drive.waitUntil(20);
	intake.move(-127);
	drive.moveToPoint(62, -24, 750, {.minSpeed = 127});
	drive.waitUntilDone();
	rightHoriWing.set_value(0);
	leftHoriWing.set_value(0);
	drive.moveToPoint(62, -36, 500, {.forwards = false, .minSpeed = 127});
	drive.turnToHeading(180, 500);
	drive.moveToPoint(62, -20, 750, {.forwards = false, .minSpeed = 127});
	drive.moveToPoint(62, -38, 500, {.minSpeed = 127});*/



	/*//AWP Auton
	//Sets the initial pose of the robot
	drive.setPose(-46, -57, 135);

	//Pushes preload triball into goal
	drive.moveToPose(-57, -26, 180, 2500, {.forwards = false, .minSpeed = 100});
	
	//Drives back and descores triball from matchload zone
	drive.moveToPose(-45, -57.25, 135, 2000);
	drive.waitUntilDone();
	vertWing.set_value(true);
	pros::delay(100);
	drive.turnToHeading(90, 1000);
	drive.waitUntilDone();
	vertWing.set_value(false);

	//Drives to touch elevation bar
	drive.moveToPose(-6, -64, 90, 2000);*/
	
	
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
	drive.setBrakeMode(COAST);

	while(true)
	{

		/*Drive train control*/
		drive.tank(masterLeftY, masterRightY);


		/*Intake control*/
		if (masterL1) /*If Left bumper is pressed*/
		{
			/*Outtake*/
			intake.move(-127);
		}
		else if (masterL2) /*If Left trigger is pressed*/
		{	
			/*Intake*/
			intake.move(127);
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
		if (masterY) //Both Wing Hold Control
		{
			rightWingToggle = false; //Sets the right wing to false
			leftWingToggle = false; //Sets the left wing to false
			leftHoriWing.set_value(true); //Sets the left wing to true
			rightHoriWing.set_value(true); //Sets the right wing to true
			leftHoriWing.set_value(true); //Sets the left wing to true
			
		}
		else if(masterY == false) 
		{
			rightHoriWing.set_value(rightWingToggle); //Sets the right wing to the toggled value
			leftHoriWing.set_value(leftWingToggle); //Sets the left wing to the toggled value
		}

		if (masterB != lastKnownStateB) //Vertical Wing Control
		{
			lastKnownStateB = masterB;
			if (masterB)
			{
				vertToggle = !vertToggle; //Toggles the variable that controls the vertical wing when X is pressed
				vertWing.set_value(vertToggle); //Sets the vertical wing to the toggled value
			}

		}




		/*PTO Control*/
		if (masterRight != lastKnownStateRight) //PTO Control
		{
			lastKnownStateRight = masterRight;
			if (masterRight)
			{
				ptoToggle = !ptoToggle; //Toggles the variable that controls the PTO when Up is pressed
				ptoPiston.set_value(ptoToggle); //Sets the PTO to the toggled value

				if(ptoToggle)
				{
					drive.setBrakeMode(HOLD); //Sets the drivetrain motors to hold if PTO is toggled on
				}
				else
				{
					drive.setBrakeMode(COAST); //Sets the drivetrain motors to coast if PTO is toggled off
				}
			}

		}

		/*Hang Release Control*/
		if (masterDown != lastKnownStateDown) //Hang Release Control
		{
			lastKnownStateDown = masterDown;
			if (masterDown && !hangReleased)
			{
				hangPiston.set_value(1); //Sets the hang piston to true when Down is pressed
			}

		}

	}
}