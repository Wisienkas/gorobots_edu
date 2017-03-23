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

	localSensorArray.assign( 4, 0 );
	angleVector.assign( 6 , vector<double>( 4 , 0 ) );
	velocityVector.assign( 6 , vector<double>( 4 , 0 ) );

	for(int i = 0; i < 6; i++) {
		angleVector[i][3] = 0;
		velocityVector[i][3] = 1;
	};

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

	int wait_time = 500;
	if( int(ticks_since_init) < wait_time ) // Start after 1000 ticks in init position. Please drop before times run out
	{
		stand( angleVector );
		moveRobot( motor, angleVector, velocityVector );

		if( int(ticks_since_init)%100==0 )
			std::cout << (wait_time-ticks_since_init)/100 << std::endl;
		return;
	}

	// ----------------------------------
	//start(motor, 1.0);
	//stand( angleVector );
	//walknet->stepWalknetTripod( sensor, angleVector, velocityVector );
	//walknet->frontLegWalk( sensor, angleVector, velocityVector );
	walknet->stepWalknet( sensor, angleVector, velocityVector );
	//invKin->stepKinematicsController( sensor, angleVector );
	//stand( angleVector );
	ballstand_head( angleVector );
	//stand( angleVector );
	//headstand( angleVector );
	moveRobot( motor, angleVector, velocityVector );
	// ----------------------------------

	if( int( ticks_since_init )%50 == 0 && outputFlag ){
		outputFlag = false;
		outputData( sensor, motor );
		outputFlag = true;
/*
		for( int i = DungBotMotorSensor::BX_ori; i <= DungBotMotorSensor::BZ_ori; i++ ){
			std::cout << sensor[i] << "\t";
		}
		std::cout << std::endl;
*/
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
		outputFile.open( "sensor_output_2.csv" );
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
	double coxa_pos[3] 	= {-1.0, -0.9,  -0.6}; // Front, Middle, Rear
	double femur_pos[3]	= {0.8,   0.1,   0.3};
	double tibia_pos[3]	= {0.4,   0.55, 0.55};

	for( int i = 0; i < 6; i++ )
	{
			angleVector[i][0] = coxa_pos[i%3];
			angleVector[i][1] = femur_pos[i%3];
			angleVector[i][2] = -tibia_pos[i%3];
	}
}


void DungBotEmptyController::ballstand_head( std::vector<std::vector<double>> &angleVector )
{
	/*
	 * Note to my self:
	 * Use head as third stand
	 * This supports my theoy:
	 * http://cache1.asset-cache.net/gc/550747325-dung-beetle-rolling-dung-on-sandy-field-gettyimages.jpg?v=1&c=IWSAsset&k=2&d=1AzDBmhi5w6Gi5HbUrlfGzYirwfrebvoolpCe4pFsKU%3D
	 *
	 * Modify Tarsus
	 * I do not think is will work, with a smaller ball. It will still slide off
	 *
	 * Add speed sensor
	*/



	// Coxa  	+ = to head	, - = to back
	// Femur 	+ = up 	, - = down
	// Tibia	+ = up 	, - = down
	int numof_frontleg = 4;
	float tibia_grep_hind = 0.1;
	float tibia_grep_mid = -0.87;


	if(ticks_since_init > 2000){
		numof_frontleg = -1;

		//angleVector[3][0] = -0.9;
		//angleVector[3][2] = -0.9; 	// -0.7;
		//tibia_grep_hind =  0.0;
		//tibia_grep_mid = -0.3;
	} else if(ticks_since_init > 750){
		//tibia_grep_hind =  0.0;
		//tibia_grep_mid = -0.3;
	}

	switch (ball_state) {
		case INIT:
			for (int i = 0; i < numof_frontleg; i+=3) {
				angleVector[i][0] = 0.4;
				angleVector[i][1] = 0.9; 	// 0.05;
				angleVector[i][2] = -0.55; 	// -0.7;
			}

			for (int i = 1; i < 5; i+=3) {
				angleVector[i][0] = -0.7;
				angleVector[i][1] = -0.74;
				angleVector[i][2] = tibia_grep_mid;
			}

			for (int i = 2; i < 6; i+=3) {
				angleVector[i][0] =  -0.4;
				angleVector[i][1] =  -0.0;
				angleVector[i][2] = tibia_grep_hind;
			}

			break;

		default:
			break;
	}

}

void DungBotEmptyController::ballstand_nohead( std::vector<std::vector<double>> &angleVector )
{
	/*
	 * Note to my self:
	 * Use head as third stand
	 * This supports my theoy:
	 * http://cache1.asset-cache.net/gc/550747325-dung-beetle-rolling-dung-on-sandy-field-gettyimages.jpg?v=1&c=IWSAsset&k=2&d=1AzDBmhi5w6Gi5HbUrlfGzYirwfrebvoolpCe4pFsKU%3D
	 *
	 * Modify Tarsus
	 * I do not think is will work, with a smaller ball. It will still slide off
	*/



	// Coxa  	+ = to head	, - = to back
	// Femur 	+ = up 	, - = down
	// Tibia	+ = up 	, - = down
	int numof_frontleg = 4;
	float tibia_grep_hind = -0.2;
	float tibia_grep_mid = -0.5;


	if(ticks_since_init > 2000){
		numof_frontleg = 3;

		//tibia_grep_hind =  0.0;
		//tibia_grep_mid = -0.3;
	} else if(ticks_since_init > 750){
		//tibia_grep_hind =  0.0;
		//tibia_grep_mid = -0.3;
	}

	switch (ball_state) {
		case INIT:
			for (int i = 0; i < numof_frontleg; i+=3) {
				angleVector[i][0] = 0.4;
				angleVector[i][1] = 0.05; 	// 0.05;
				angleVector[i][2] = -0.7; 	// -0.7;
			}

			for (int i = 1; i < 5; i+=3) {
				angleVector[i][0] = 0.2;
				angleVector[i][1] = -0.1;
				angleVector[i][2] = tibia_grep_mid;
			}

			for (int i = 2; i < 6; i+=3) {
				angleVector[i][0] =  0.0;
				angleVector[i][1] =  0.3;
				angleVector[i][2] = tibia_grep_hind;
			}

			break;

		default:
			break;
	}

}

void DungBotEmptyController::rollstand( std::vector<std::vector<double>> &angleVector )
{

	// 						Front		 Middle		 Rear
	double coxa_pos[3] 	= { 0.4, 		-0.4, 		-0.15}; // Coxa  	+ = to head	, - = to back
	double femur_pos[3]	= {	0.2, 		0.9, 		-0.3};	// Femur 	+ = down 	, - = up
	double tibia_pos[3]	= {-0.7, 		-0.3, 		-0.2};  // Tibia	+ = down	, - = up

	for( int i = 0; i < 6; i++ )
	{
		if( (i!=0 && i!=3) || int(ticks_since_init) < 700){
			angleVector[i][0] = coxa_pos[i%3];
			angleVector[i][1] = femur_pos[i%3];
			angleVector[i][2] = tibia_pos[i%3];
		}
	}

	/**
	 * 	Preliminary:
	 *
	 * 	Ballsize: 0.27
	 * 	s1->setPosition(osg::Vec3(0.4, 0.0, 0.0));
	 *	robot->place( Pos( 0.0, 0.0, 0.2 ) );
	 *
	 * 	Close to body  	- AEP: -0.4  0.8  0.5
	 * 	Away from body 	- PEP: -0.4 -0.2 -0.5
	 *	Back again		- MID: -0.4  0.9 -0.3
	 */
}

void DungBotEmptyController::standsimple( std::vector<std::vector<double>> &angleVector )
{
	double coxa_pos[3] 	= {-0.4, -0.4, -0.4}; // Front, Middle, Rear
	double femur_pos[3]	= {-0.6, -0.6, -0.6};
	double tibia_pos[3]	= {0.0, 0.0, 0.0};

	for( int i = 0; i < 6; i++ )
	{
		angleVector[i][0] = coxa_pos[i%3];
		angleVector[i][1] = femur_pos[i%3];
		angleVector[i][2] = tibia_pos[i%3];
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
		for( int i = DungBotMotorSensor::R0_s0; i <= DungBotMotorSensor::L2_s0; i=i+6 ){
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

void DungBotEmptyController::extractSensor( const sensor* sensor, int leg, std::vector<double> & extractedSensors )
{
        //      Set the three first places for the angles for that specific leg.
        for( int i = 0; i < 3; i++ )
        {
                extractedSensors[ i ] = sensor[ leg + i*6 ];
        }

        //      Set index 3, for the contact sensor
        extractedSensors[3] = 0.0;
        for( int i = 0; i < 6; i++ )
        {
                if( sensor[25 + 6*leg + i] == true ){
                        extractedSensors[3] = 1.0;
                }
        }
        // TODO EXTRACT FRA ALL 3 PARTS (Need help from theis)
        getGroundContact_tarsus();
        getGroundContact_tibia();
        getGroundContact_femur();
}

bool DungBotEmptyController::getGroundContact_tarsus( void )
{
	return localSensorArray[3];
}

bool DungBotEmptyController::getGroundContact_tibia( void )
{
	return localSensorArray[3];
}

bool DungBotEmptyController::getGroundContact_femur( void )
{
	return localSensorArray[3];
}
