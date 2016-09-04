/*****************************************************************************
*   "THE BEER-WARE LICENSE" (Revision 43):
*   This software was written by Theis Str�m-Hansen <thstroemhansen@gmail.com>
*   and Mathias Thor <mathias.thor@gmail.com>
*   As long as you retain this notice you can do whatever you want with it.
*   If we meet some day, and you think this stuff is worth it, you can buy me
*   a beer in return.
*
*   Should this software ever become self-aware, remember: I am your master
*****************************************************************************/

/*
 * The motor can take both position and velocity
 * Both should be between -1.0 and 1.0
 * To use the motor with position set: pos_vel = True;	(uses the vector angleVectors and ignores velocityVector)
 * To use the motor with velocity set: pos_vel = False; (uses the vector velocityVector and ignores angleVectors)
 */

#include "DungBotEmptyController.h"
#include "kinematicsController.h"
#include "walknetcontroller.h"

//using namespace matrix;
using namespace std;

DungBotEmptyController::DungBotEmptyController( const std::string& name  )
: AbstractController( name, "1.0" )
{
	initialised = false;

	ticks_since_init = 0.0;

	nSensors = 0;
	nMotors = 0;

	angleVector.assign( 6 , vector<double>( 4 , 0 ) );
	velocityVector.assign( 6 , vector<double>( 4 , 0 ) );

	for(int i = 0; i < 6; i++) {
		angleVector[i][3] = 0;
		velocityVector[i][3] = 1;
	};

	invKin = new kinematicsController();
	walknet = new walknetcontroller();

}

DungBotEmptyController::~DungBotEmptyController()
{
	if( writeOutput )
	{
		outputFile.close();
	}
}

void DungBotEmptyController::stepNoLearning( const sensor* sensor, int sensorNumber, motor* motor, int motorNumber )
{
	assert( motorNumber >= DungBotMotorSensor::DUNGBOT_MOTOR_MAX );
	assert( sensorNumber >= DungBotMotorSensor::DUNGBOT_SENSOR_MAX );

	//	Update internal time
	ticks_since_init++;

	if( int(ticks_since_init) < 600 ) // Start after 1000 ticks in init position. Please drop before times run out
	{
		stand( angleVector );
		moveRobot( motor, angleVector, velocityVector );

		if( int(ticks_since_init)%100==0 )
			std::cout << (600-ticks_since_init)/100 << std::endl;
		return;
	}

	// ----------------------------------
	//start(motor, 1.0);
	//standsimple( angleVector );
	//walknet->stepWalknetTripod( sensor, angleVector, velocityVector );
	walknet->stepWalknet( sensor, angleVector, velocityVector );
	//invKin->stepKinematicsController( sensor, angleVector );
	//ballstand( angleVector );
	//rollstand( angleVector );
	//headstand( angleVector );
	moveRobot( motor, angleVector, velocityVector );
	// ----------------------------------

	if( int( ticks_since_init )%50 == 0 && outputFlag ){
		outputFlag = false;
		outputData( sensor, motor );
		outputFlag = true;
	}
}

void DungBotEmptyController::step( const sensor* sensor, int sensorNumber, motor* motor, int motorNumber )
{
	stepNoLearning(sensor, sensorNumber, motor, motorNumber);
}

void DungBotEmptyController::init( int sensorNumber, int motorNumber, RandGen* randGen )
{
	assert( motorNumber >= DungBotMotorSensor::DUNGBOT_MOTOR_MAX );
	assert( sensorNumber >= DungBotMotorSensor::DUNGBOT_SENSOR_MAX );

	nSensors = sensorNumber;
	nMotors = motorNumber;

	if( writeOutput )
	{
		outputFile.open( "12_output5.csv" );
	}

	initialised = true;
}

int DungBotEmptyController::getSensorNumber() const
{
	return nSensors;
}

int DungBotEmptyController::getMotorNumber() const
{
	return nMotors;
}

bool DungBotEmptyController::store( FILE* f ) const
{
	Configurable::print(f,0);
	return true;
}

bool DungBotEmptyController::restore( FILE* f )
{
	Configurable::parse(f);
	return true;
}

void DungBotEmptyController::stand( std::vector<std::vector<double>> &angleVector )
{
	double coxa_pos[3] 	= {0.1, 0, 0.1}; // Front, Middle, Rear
	double femur_pos[3]	= {0.1, 0, 0.1};
	double tibia_pos[3]	= {0.1, 0, 0.1};



	for( int i = 0; i < 6; i++ )
	{
		angleVector[i][0] = coxa_pos[i%3];
		angleVector[i][1] = femur_pos[i%3];
		angleVector[i][2] = -tibia_pos[i%3];
	}
}


void DungBotEmptyController::ballstand( std::vector<std::vector<double>> &angleVector )
{
	// 						Front		 Middle		 Rear
	double coxa_pos[3] 	= {-0.05, 		-0.25, 		-0.3};  // Coxa
	double femur_pos[3]	= {	0.7, 		 0.9, 		 0.6};	// Femur
	double tibia_pos[3]	= {	-0.2, 		 0.0, 		-0.4};  // Tibia

	for( int i = 0; i < 6; i++ )
	{
		angleVector[i][0] = coxa_pos[i%3];
		angleVector[i][1] = femur_pos[i%3];
		angleVector[i][2] = -tibia_pos[i%3];
	}

	// Backjoint

}

void DungBotEmptyController::rollstand( std::vector<std::vector<double>> &angleVector )
{
	// 						Front		 Middle		 Rear
	double coxa_pos[3] 	= { 0.4, 		-0.25, 		-0.2};  // Coxa
	double femur_pos[3]	= {	0.2, 		 0.2, 		 0.5};	// Femur
	double tibia_pos[3]	= { 0.7, 		 0.2, 		 0.2};  // Tibia

	for( int i = 0; i < 6; i++ )
	{
		angleVector[i][0] = coxa_pos[i%3];
		angleVector[i][1] = femur_pos[i%3];
		angleVector[i][2] = -tibia_pos[i%3];
	}
}

void DungBotEmptyController::headstand( std::vector<std::vector<double>> &angleVector )
{
	// 						Front		 Middle		 Rear
	double coxa_pos[3] 	= { 0.0, 		-0.2, 		-0.5};  // Coxa
	double femur_pos[3]	= {	0.2, 		 0.05, 		 0.5};	// Femur
	double tibia_pos[3]	= { 0.6, 		 0.9, 		 0.4};  // Tibia

	for( int i = 0; i < 6; i++ )
	{
		angleVector[i][0] = coxa_pos[i%3];
		angleVector[i][1] = femur_pos[i%3];
		angleVector[i][2] = -tibia_pos[i%3];
	}

	//angleVector[3][0] = 1;
	//angleVector[3][1] = 1;
	//angleVector[3][2] = 0;

}

void DungBotEmptyController::standsimple( std::vector<std::vector<double>> &angleVector )
{
	double coxa_pos[3] 	= {-0.4, -0.4, -0.4}; // Front, Middle, Rear
	double femur_pos[3]	= {0.6, 0.6, 0.6};
	double tibia_pos[3]	= {0.0, 0.0, 0.0};

	for( int i = 0; i < 6; i++ )
	{
		angleVector[i][0] = coxa_pos[i%3];
		angleVector[i][1] = femur_pos[i%3];
		angleVector[i][2] = tibia_pos[i%3];
	}
}

void DungBotEmptyController::start( motor* motor, double vel ) {

	for( int i = 0; i < DungBotMotorSensor::DUNGBOT_MOTOR_MAX; i++ )
	{
		if( i >= 0 && i < 6)		// COXA
		{
			motor[i] = -vel;//*sin(ticks_since_init*0.001);
		}
		if( i >= 6 && i < 12 ) 		// FEMUR
		{
			motor[i] = vel;//*sin(ticks_since_init*0.001);
		}
		if( i >= 12 && i < 18 ) 	// TIBIA
		{
			motor[i] = -vel;//*sin(ticks_since_init*0.001);
		}
	}
}

void DungBotEmptyController::moveRobot( motor* motor, std::vector<std::vector<double>> angleVector, std::vector<std::vector<double>> velocityVector ) {

	for( int i = 0; i < 3; i++ )
	{
		for(int j = 0; j < 6; j ++)
		{
			if ( angleVector[j][3] == 0 && velocityVector[j][3] == 1 ) {
				motor[i*6+j] = saturate(angleVector[j][i], -1.0, 1.0);
			}
			else if (  angleVector[j][3] == 1 && velocityVector[j][3] == 0 ){
				motor[i*6+j] = saturate(velocityVector[j][i] + 10, 9.0, 11.0)*-1; // Minus 1 because motor array reverses the sign
			}
			else {
				cout << "[FAILED] Can use the motor as both velocity and position controlled!" << endl;
				exit(1);
			}
		}
	}
}

void DungBotEmptyController::outputData( const sensor* sensor, motor* motor )
{
	// TODO: Make function that write (!) at the end if there is a miss match between motor[] and sensor[]

	std::vector<double> sensorOutput;
	std::vector<double> motorInput;
	std::vector<bool> legPhase;
	walknet->getPhase( legPhase );

    cout.setf(ios::fixed, ios::floatfield);
    cout.precision(2);

	if( writeOutput )
	{
		collectData( sensor, motor );
	}
}

void DungBotEmptyController::collectData( const sensor* sensor, motor* motor )
{
	if( outputFile.is_open() && writeOutput)
	{
		//	Print all motor and sensor values here.
	/*
		outputFile << ticks_since_init;
		for( unsigned int i = 0; i < motorInput.size(); i++ )
		{
			outputFile << "," << motor[i] << "," << sensor[i];
		}
		outputFile << std::endl;
	*/

		//	Print the contact sensors for the stump.
		/*
		outputFile << sensor[DungBotMotorSensor::L0_s0] << "," << sensor[DungBotMotorSensor::L1_s0] << "," << sensor[DungBotMotorSensor::L2_s0] << ","
		    		<< sensor[DungBotMotorSensor::R0_s0] << "," << sensor[DungBotMotorSensor::R1_s0] << "," << sensor[DungBotMotorSensor::R2_s0];
		outputFile << std::endl;
		*/

		/**
		 * Print position of Head, Thorax, and abdomen.
		 * Print position of Legs.
		 */

		for( int i = DungBotMotorSensor::RPS_Leg1Cx; i <= DungBotMotorSensor::RPS_Leg6Taz; i++ ){
			outputFile << sensor[i] << ",";
		}
		for( int i = DungBotMotorSensor::RPS_REARx; i <= DungBotMotorSensor::RPS_HEADz; i++ ){
			outputFile << sensor[i] << ",";
		}
		outputFile << std::endl;

	}
	else
	{
		std::cout << "DungBot controller: File not open" << std::endl;
	}
}

double DungBotEmptyController::saturate(double input, double thres_low, double thres_high) {
	if ( input > thres_high ) {
		return thres_high;
	}
	else if ( input < thres_low ) {
		return thres_low;
	}
	else {
		return input;
	}
}
