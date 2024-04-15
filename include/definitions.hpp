#ifndef DEFINITIONS
#define DEFINITIONS

//Controller Inputs
#define L1 pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_L1
#define L2 pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_L2
#define R1 pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_R1
#define R2 pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_R2
#define A pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_A
#define B pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_B
#define X pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_X
#define Y pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_Y
#define Up pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_UP
#define Down pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_DOWN
#define Right pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_RIGHT
#define Left pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_LEFT
#define LeftX pros::controller_analog_e_t::E_CONTROLLER_ANALOG_LEFT_X
#define LeftY pros::controller_analog_e_t::E_CONTROLLER_ANALOG_LEFT_Y
#define RightX pros::controller_analog_e_t::E_CONTROLLER_ANALOG_RIGHT_X
#define RightY pros::controller_analog_e_t::E_CONTROLLER_ANALOG_RIGHT_Y

#define masterL1 master.get_digital(L1)
#define masterL2 master.get_digital(L2)
#define masterR1 master.get_digital(R1)
#define masterR2 master.get_digital(R2)
#define masterA master.get_digital(A)
#define masterB master.get_digital(B)
#define masterX master.get_digital(X)
#define masterY master.get_digital(Y)
#define masterUp master.get_digital(Up)
#define masterDown master.get_digital(Down)
#define masterRight master.get_digital(Right)
#define masterLeft master.get_digital(Left)
#define masterLeftX master.get_analog(LeftX)
#define masterLeftY master.get_analog(LeftY)
#define masterRightX master.get_analog(RightX)
#define masterRightY master.get_analog(RightY)


//Gearbox Definitions
#define TORQUEBOX pros::motor_gearset_e_t::E_MOTOR_GEARSET_36
#define REGBOX pros::motor_gearset_e_t::E_MOTOR_GEARSET_18
#define SPEEDBOX pros::motor_gearset_e_t::E_MOTOR_GEARSET_06

//Unit Definitions
#define DEGREES pros::rotation_units_e_t::E_DEGREES
#define ROTATIONS pros::rotation_units_e_t::E_ROTATIONS

//Brake Mode Definitions
#define COAST pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_COAST
#define HOLD pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_HOLD

//Short Hands For Chassis
#define poseX drive.getPose().x
#define poseY drive.getPose().y
#define poseTheta drive.getPose().theta

//Motor Port Definitions
#define FLD 17
#define MLD 18
#define BLD 19
#define FRD 14
#define MRD 13
#define BRD 12
#define LIM 10
#define RIM 1

//Sensor Port Definitions
#define HODOM_ROT 16
#define VODOM_ROT 4
#define IMU_PORT 20


//Three-Wire Device Port Definitions
#define LEFTW_ADIDO 'A'
#define RIGHTW_ADIDO 'B'
#define VERTW_ADIDO 'H'
#define PTO_ADIDO 'D'

#endif /* DEFINITIONS */
