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

#include "kinematicsController.h"

kinematicsController::kinematicsController(void) {
	std::cout << "CREATING KINEMATICS CONTROLLER" << std::endl;

	targetPositionPointer.assign( 6, 1 );

	std::cout << "LOADING POSITION VECTORS" << std::endl;

	loadPositionVectors();

	std::cout << "CREATE FRAME" << std::endl;

	frame();

	std::cout << "KinematicsController Constructor DONE" << std::endl;
}

kinematicsController::~kinematicsController(void) {
}

void kinematicsController::frame(){



}

void kinematicsController::homo( std::vector<std::vector<double>> &t, std::vector<std::vector<double>> r, std::vector<double> p ){
	t[0][0] = r[0][0];
	t[0][1] = r[0][1];
	t[0][2] = r[0][2];
	t[0][3] = p[0];

	t[1][0] = r[1][0];
	t[1][1] = r[1][1];
	t[1][2] = r[1][2];
	t[1][3] = p[1];

	t[2][0] = r[2][0];
	t[2][1] = r[2][1];
	t[2][2] = r[2][2];
	t[2][3] = p[2];

	t[3][0] = 0;
	t[3][1] = 0;
	t[3][2] = 0;
	t[3][3] = 1;
/*
	std::cout << "******homo******" << std::endl;
	for (int row = 0; row < 4; row++) {
		for (int col = 0; col < 4; col++) {
			std::cout << t[row][col] << "  ";
		}
		std::cout << "\n";
	}
*/
}

void kinematicsController::rotate( std::vector<std::vector<double>> &matrix, double x, double y, double z ){
	matrix[0][0] = cos(y)*cos(z);						//	R11
	matrix[0][1] = sin(x)*sin(y)*cos(z)-cos(x)*sin(z);	//	R12
	matrix[0][2] = cos(x)*sin(y)*cos(z)+sin(x)*sin(z);	//	R13

	matrix[1][0] = cos(y)*sin(z);						//	R21
	matrix[1][1] = sin(x)*sin(y)*sin(z)+cos(x)*cos(z);	//	R22
	matrix[1][2] = cos(x)*sin(y)*sin(z)-sin(x)*cos(z);	//	R23

	matrix[2][0] = -sin(y);								//	R31
	matrix[2][1] = sin(x)*cos(y);						//	R32
	matrix[2][2] = cos(x)*cos(y);						//	R33
/*
	std::cout << "******rotate******" << std::endl;
	for (int row = 0; row < 3; row++) {
		for (int col = 0; col < 3; col++) {
			std::cout << matrix[row][col] << "  ";
		}
		std::cout << "\n";
	}
*/
}

void kinematicsController::matrixMulti( std::vector<std::vector<double>> &newM, std::vector<std::vector<double>> m1, std::vector<std::vector<double>> m2 ){
	for (int row = 0; row < 4; row++) {
		for (int col = 0; col < 4; col++) {
			// Multiply the row of A by the column of B to get the row, column of product.
			for (int inner = 0; inner < 3; inner++) {
				newM[row][col] += m1[row][inner] * m2[inner][col];
			}
			//std::cout << newM[row][col] << "  ";
		}
		//std::cout << "\n";
	}
}

void kinematicsController::kinematic( std::vector<double> &newAngle, std::vector<double> theta, int legNum ){
	std::vector<double> tc;		//	TC offset.
	std::vector<double> coxa, rc;
	std::vector<double> femur, rf;
	std::vector<double> tibia, rt;

	double scale = 3.75+9.111+10.324;

	int lr;
	switch (legNum) {
		case 0: case 1: case 2:
			lr = 1;
			break;
		case 3: case 4: case 5:
			lr = -1;
			break;
		default:
			break;
	}

	tc = {0,0,0};
	switch (legNum) {
		case 0: case 3:
		//	TC offset.
			//tc = {1.65227/scale/4-2.8689/scale, lr * 2.5409/scale, -4.9/scale/2 + 0.4841/scale};
		//	Limb lengths
			coxa = {0, 0, 2.46037/scale};
			femur = {0, 0, 3.24472/scale};
			tibia = {0, 0, 4.68943/scale};
			break;
		case 1: case 4:
		//	TC offset.
			//tc = {1.65227/scale/4+0/scale, lr * 2.5883/scale, -4.9/scale/2 + 0.5672/scale};
		//	Limb lengths
			coxa = {0, 0, 2.11888/scale};
			femur = {0, 0, 4.23025/scale};
			tibia = {0, 0, 3.72093/scale};
		//	Limb rotations
			rc = { M_PI/180*(180+110.4237), M_PI/180*(90-65.3676) };
			rf = { M_PI/2, -M_PI/180*120 };
			rt = { M_PI/180*85 };
		//	Joint limits
			break;
		case 2: case 5:
			//tc = {1.65227/scale/4+3.1666/scale, lr * 4.7496/scale, -4.9/scale/2 + 0/scale};
			coxa = {0, 0, 4.05514/scale};
			femur = {0, 0, 4.63394/scale};
			tibia = {0, 0, 5.54793/scale};
			break;
		default:
			break;
	}

	//	Translation and rotation of TC
	std::vector<std::vector<double>> rab(3, std::vector<double>(3));
	std::vector<std::vector<double>> tab(4, std::vector<double>(4));
	rotate( rab, rc[0], 0, rc[1]+theta[0] );
	homo( tab, rab, tc );

	//	Translation and rotation of CF
	std::vector<std::vector<double>> rbc(3, std::vector<double>(3));
	std::vector<std::vector<double>> tbc(4, std::vector<double>(4));
	rotate( rbc, 0, rf[1]+theta[1], rf[0] );
	homo( tbc, rbc, coxa );

	//	Translation and rotation of FT
	std::vector<std::vector<double>> rcd(3, std::vector<double>(3));
	std::vector<std::vector<double>> tcd(4, std::vector<double>(4));
	rotate( rcd,  0, rt[0]+theta[2], 0 );
	homo( tcd, rcd, femur );

	//	Translation and rotation of End effector
	std::vector<std::vector<double>> rd(3, std::vector<double>(3));
	std::vector<std::vector<double>> td(4, std::vector<double>(4));
	rotate( rd, 0, 0, 0 );
	homo( td, rd, tibia );

	//	Now we need to return the values.
	std::vector<std::vector<double>> temp1(4, std::vector<double>(4));
	std::vector<std::vector<double>> temp2(4, std::vector<double>(4));
	std::vector<std::vector<double>> temp3(4, std::vector<double>(4));

	matrixMulti( temp1, tab, tbc );
	matrixMulti( temp2, temp1, tcd);
	matrixMulti( temp3, temp2, td );

	newAngle.push_back(temp3[0][3]);
	newAngle.push_back(temp3[1][3]);
	newAngle.push_back(-temp3[2][3]);
}

void kinematicsController::preOffset( std::vector<double> &angleVector, int legNum ){
	/**
	 *	Here we need to go from the joint values, to angles that makes sense.
	 *	"Small to large"
	 */

	double scale = 0.9;
	std::vector<double> tc;
	std::vector<double> cf;
	std::vector<double> ft;
	std::vector<double> temp;
	switch (legNum) {
		case 0: case 3:
			tc = { 95.7878*scale, 	-M_PI / 180.0 * 80, 	M_PI / 180.0 * (95.7878*scale - 80) };
			cf = { 90*scale, 		M_PI / 180.0 * 30, 		-M_PI / 180.0 * ( 30*scale - 30) };
			ft = { 170*scale, 		M_PI / 180.0 * 85, 		-M_PI / 180.0 * ( 170*scale - 85) };
			break;
		case 1: case 4:
			tc = { 116.1153*scale, 	-M_PI / 180.0 * 50, 	M_PI / 180.0 * (116.1153*scale - 50) };
			cf = { 90*scale, 		M_PI / 180.0 * 30, 		-M_PI / 180.0 * ( 30*scale - 30) };
			ft = { 170*scale, 		M_PI / 180.0 * 85, 		-M_PI / 180.0 * ( 170*scale - 85) };
			break;
		case 2: case 5:
			tc = { 160.8514*scale, 	-M_PI / 180.0 * 50, 	M_PI / 180.0 * (160.8514*scale - 50) };
			cf = { 90*scale, 		M_PI / 180.0 * 30, 		-M_PI / 180.0 * ( 30*scale - 30) };
			ft = { 170*scale, 		M_PI / 180.0 * 85, 		-M_PI / 180.0 * ( 170*scale - 85) };
			break;
		default:
			break;
	}

	if(true){
		std::cout << "***********************-PRE" << std::endl;
		std::cout << angleVector[0] << " " << angleVector[1] << " " << angleVector[2] << std::endl;
	}

	angleVector[0] = ( angleVector[0] - ( tc[1]+tc[2] )/2 ) * preScale( tc[0] );
	angleVector[1] = ( angleVector[1] - ( cf[1]+cf[2] )/2 ) * preScale( cf[0] );
	angleVector[2] = ( angleVector[2] - ( ft[1]+ft[2] )/2 ) * preScale( ft[0] );

	if(true){
		std::cout << angleVector[0] << " " << angleVector[1] << " " << angleVector[2] << std::endl;
	}
}

double kinematicsController::preScale( double x ){
	return ( 2 * M_PI / 360 * x ) / (2 * M_PI/360 * 360);
}

void kinematicsController::postOffset( std::vector<std::vector<double>> &angleVector, int legNum ){
	/**
	 * 	Here we need to go from the angles that makes sense, back to the angles that the joints "understand".
	 * 	"Large to small"
	 */

	double scale = 0.9;
	std::vector<double> tc;
	std::vector<double> cf;
	std::vector<double> ft;
	std::vector<double> temp;
	switch (legNum) {
		case 0: case 3:
			tc = { 95.7878*scale, 	-M_PI / 180.0 * 80, 	M_PI / 180.0 * (95.7878*scale - 80) };
			cf = { 90*scale, 		M_PI / 180.0 * 30, 		-M_PI / 180.0 * ( 30*scale - 30) };
			ft = { 170*scale, 		M_PI / 180.0 * 85, 		-M_PI / 180.0 * ( 170*scale - 85) };
			break;
		case 1: case 4:
			tc = { 116.1153*scale, 	-M_PI / 180.0 * 50, 	M_PI / 180.0 * (116.1153*scale - 50) };
			cf = { 90*scale, 		 M_PI / 180.0 * 30, 	-M_PI / 180.0 * ( 30*scale - 30) };
			ft = { 170*scale, 		 M_PI / 180.0 * 85, 	-M_PI / 180.0 * ( 170*scale - 85), };
			break;
		case 2: case 5:
			tc = { 160.8514*scale, 	-M_PI / 180.0 * 50, 	M_PI / 180.0 * (160.8514*scale - 50) };
			cf = { 90*scale, 		M_PI / 180.0 * 30, 		-M_PI / 180.0 * ( 30*scale - 30) };
			ft = { 170*scale, 		M_PI / 180.0 * 85, 		-M_PI / 180.0 * ( 170*scale - 85) };
			break;
		default:
			break;
	}

	if(true){
		std::cout << "***********************-POST" << std::endl;
		std::cout << angleVector[legNum][0] << " " << angleVector[legNum][1] << " " << angleVector[legNum][2] << std::endl;
	}
	angleVector[legNum][0] = ( angleVector[legNum][0] * postScale( tc[0] ) ) + ( tc[1]+tc[2] )/2;
	angleVector[legNum][1] = ( angleVector[legNum][1] * postScale( cf[0] ) ) + ( cf[1]+cf[2] )/2;
	angleVector[legNum][2] = ( angleVector[legNum][2] * postScale( ft[0] ) ) + ( ft[1]+ft[2] )/2;

	if(true){
		std::cout << angleVector[legNum][0] << " " << angleVector[legNum][1] << " " << angleVector[legNum][2] << std::endl;
	}
}

double kinematicsController::postScale( double x ){
	return ( 2 * M_PI / 360 * 360 ) / (2 * M_PI/360 * x);
}

void kinematicsController::stepKinematicsController( const sensor* sensor, std::vector<std::vector<double>> &angleVector ) {
	/**
	 * 	For each step, we need to take a look at the position each leg is in now.
	 * 	And if it is close enough, send it to the next "location".
	 *
	 *	The cycle is fixed.
	 *
	 * 	The "atPosition" method in the bottom should be used.
	 *
	 *	We look at the sensor values for each leg. Decide if it is
	 *	close enough to the position, that we can send the leg
	 *	to the next position.
	 */

	std::cout << "*********************" << std::endl;

	std::cout << "COUNTER IS: " << counter << std::endl;

	for( int i = 0; i < 6; i++ ){
		legPositionControl( sensor, angleVector, 1 );
		break; //todo: Remove when done
		legPositionControl( sensor, angleVector, i );
	}

	counter++;
	std::cout << "$$$$$$$$$$$$$$$$$$$$$" << std::endl;

}

void kinematicsController::legPositionControl( const sensor* sensor, std::vector<std::vector<double>> &angleVector, int legNum ) {
		std::vector<double> targetPos = {0, 0, 0};
		switch(legNum){
			case 0:
				for( int i = 0; i < 3; i++ ){
					targetPos[i] = positionListL0[ targetPositionPointer[legNum] % positionListL0.size() ][i];
				}
				break;
			case 1:
				for( int i = 0; i < 3; i++ ){
					targetPos[i] = positionListL1[ targetPositionPointer[legNum] % positionListL1.size() ][i];
				}
				break;
			case 2:
				for( int i = 0; i < 3; i++ ){
					targetPos[i] = positionListL2[ targetPositionPointer[legNum] % positionListL2.size() ][i];
				}
				break;
			case 3:
				for( int i = 0; i < 3; i++ ){
					targetPos[i] = positionListR0[ targetPositionPointer[legNum] % positionListR0.size() ][i];
				}
				break;
			case 4:
				for( int i = 0; i < 3; i++ ){
					targetPos[i] = positionListR1[ targetPositionPointer[legNum] % positionListR1.size() ][i];
				}
				break;
			case 5:
				for( int i = 0; i < 3; i++ ){
					targetPos[i] = positionListR2[ targetPositionPointer[legNum] % positionListR2.size() ][i];
				}
				break;
			default:
				std::cout << "error 2" << std::endl;
				break;
		}

	//	The current sensor values for the leg. (non-offset)
		std::vector<double> legSensor = { sensor[legNum], sensor[6+legNum], sensor[12+legNum] };

	//	Do the offset so that the angles fit correctly.
		preOffset( legSensor, legNum );

	//	Do forward kinematics to find the current position.
		std::vector<double> currentPos;
		kinematic( currentPos, legSensor, legNum );

	//	Find the error between the target- and current position.
		std::vector<double> errorPos = { targetPos[0]-currentPos[0], targetPos[1]-currentPos[1], targetPos[2]-currentPos[2] };

	//	Find the distance from the start.
		double dist_start = sqrt( pow(errorPos[0],2) + pow(errorPos[1],2) + pow(errorPos[2],2) );

		std::vector<double> deltaStep;
		std::vector<double> stepPosition;
		double dist_test;
		double nummerical_value = 0.001;
		double stepsize = 0.0005;
//used in the bottom of the function
double threshold = 0.01; //todo: put in .h

		while( true ){
			//	Step we want to take
			deltaStep = { errorPos[0]*stepsize, errorPos[1]*stepsize, errorPos[2]*stepsize };

			//	Position we want to hit
			stepPosition = { currentPos[0] + deltaStep[0], currentPos[1] + deltaStep[1], currentPos[2] + deltaStep[2] };

			dist_test = sqrt( pow(deltaStep[0],2) + pow(deltaStep[1],2) + pow(deltaStep[2],2) );

			if( dist_test < nummerical_value ){
				stepsize *= 2;
				num_test_counter++;
			}else{
				break;
			}
		}

std::cout << std::endl;
std::cout << "*******" << std::endl;
std::cout << "cx: " << currentPos[0] << " cy: " << currentPos[1]  << " cz: " << currentPos[2]  << std::endl;
std::cout << "ex: " << errorPos[0] << " ey: " << errorPos[1]  << " ez: " << errorPos[2]  << std::endl;
std::cout << "dx: " << stepPosition[0] << " dy: " << stepPosition[1] << " dz: " << stepPosition[2] << std::endl;
std::cout << "tx: " << targetPos[0] << " ty: " << targetPos[1] << " tz: " << targetPos[2] << std::endl;
std::cout << "d:  " << std::setprecision(5) <<dist_start << std::endl;
std::cout << "*******" << std::endl;
std::cout << std::endl;


		//	Now that we have the position we want to hit, we try to rotate the joints, to the positions.
		//	Do the six calculations.
		std::vector<double> temp = legSensor;

		//	joint 1-
		//std::cout << "******CAL1******" << std::endl;
		std::vector<double> cal1;
		legSensor[0] = legSensor[0] - stepsize;
		kinematic( cal1, legSensor, legNum );
		legSensor = temp;

		//	joint 1+
		std::vector<double> cal2;
		//std::cout << "******CAL2******" << std::endl;
		legSensor[0] = legSensor[0] + stepsize;
		kinematic( cal2, legSensor, legNum );
		legSensor = temp;

		//	joint 2-
		//std::cout << "******CAL3******" << std::endl;
		std::vector<double> cal3;
		legSensor[1] = legSensor[1] - stepsize;
		kinematic( cal3, legSensor, legNum );
		legSensor = temp;

		//	joint 2+
		//std::cout << "******CAL4******" << std::endl;
		std::vector<double> cal4;
		legSensor[1] = legSensor[1] + stepsize;
		kinematic( cal4, legSensor, legNum );
		legSensor = temp;

		//	joint 3-
		//std::cout << "******CAL5******" << std::endl;
		std::vector<double> cal5;
		legSensor[2] = legSensor[2] - stepsize;
		kinematic( cal5, legSensor, legNum );
		legSensor = temp;

		//	joint 3+
		//std::cout << "******CAL6******" << std::endl;
		std::vector<double> cal6;
		legSensor[2] = legSensor[2] + stepsize;
		kinematic( cal6, legSensor, legNum );
		legSensor = temp;

		//	Now we need to calculate the distance to the point, with the different joint moves.
		std::vector<double> lt;
		lt.push_back( sqrt( pow(stepPosition[0] - cal1[0],2) + pow(stepPosition[1] - cal1[1],2) + pow(stepPosition[2] - cal1[2],2)) );
		lt.push_back( sqrt( pow(stepPosition[0] - cal2[0],2) + pow(stepPosition[1] - cal2[1],2) + pow(stepPosition[2] - cal2[2],2)) );
		lt.push_back( sqrt( pow(stepPosition[0] - cal3[0],2) + pow(stepPosition[1] - cal3[1],2) + pow(stepPosition[2] - cal3[2],2)) );
		lt.push_back( sqrt( pow(stepPosition[0] - cal4[0],2) + pow(stepPosition[1] - cal4[1],2) + pow(stepPosition[2] - cal4[2],2)) );
		lt.push_back( sqrt( pow(stepPosition[0] - cal5[0],2) + pow(stepPosition[1] - cal5[1],2) + pow(stepPosition[2] - cal5[2],2)) );
		lt.push_back( sqrt( pow(stepPosition[0] - cal6[0],2) + pow(stepPosition[1] - cal6[1],2) + pow(stepPosition[2] - cal6[2],2)) );


		if( (lt[0] < lt[1]) && (lt[0] < lt[2]) && (lt[0] < lt[3]) && (lt[0] < lt[4]) && (lt[0] < lt[5]) ){
			std::cout << "CASE 0" << std::endl;
			angleVector[legNum][0] = legSensor[0]	- stepsize;
			angleVector[legNum][1] = legSensor[1]	;
			angleVector[legNum][2] = legSensor[2]	;
		}
		else if( (lt[1] < lt[0]) && (lt[1] < lt[2]) && (lt[1] < lt[3]) && (lt[1] < lt[4]) && (lt[1] < lt[5]) ){
			std::cout << "CASE 1" << std::endl;
			angleVector[legNum][0] = legSensor[0]	+ stepsize;
			angleVector[legNum][1] = legSensor[1]	;
			angleVector[legNum][2] = legSensor[2]	;
		}
		else if( (lt[2] < lt[0]) && (lt[2] < lt[1]) && (lt[2] < lt[3]) && (lt[2] < lt[4]) && (lt[2] < lt[5]) ){
			std::cout << "CASE 2" << std::endl;
			angleVector[legNum][0] = legSensor[0]	;
			angleVector[legNum][1] = legSensor[1]	- stepsize;
			angleVector[legNum][2] = legSensor[2]	;
		}
		else if( (lt[3] < lt[0]) && (lt[3] < lt[1]) && (lt[3] < lt[2]) && (lt[3] < lt[4]) && (lt[3] < lt[5]) ){
			std::cout << "CASE 3" << std::endl;
			angleVector[legNum][0] = legSensor[0]	;
			angleVector[legNum][1] = legSensor[1]	+ stepsize;
			angleVector[legNum][2] = legSensor[2]	;
		}
		else if( (lt[4] < lt[0]) && (lt[4] < lt[1]) && (lt[4] < lt[2]) && (lt[4] < lt[3]) && (lt[4] < lt[5]) ){
			std::cout << "CASE 4" << std::endl;
			angleVector[legNum][0] = legSensor[0]	;
			angleVector[legNum][1] = legSensor[1]	;
			angleVector[legNum][2] = legSensor[2]	- stepsize;
		}
		else if( (lt[5] < lt[0]) && (lt[5] < lt[1]) && (lt[5] < lt[2]) && (lt[5] < lt[3]) && (lt[5] < lt[4]) ){
			std::cout << "CASE 5" << std::endl;
			angleVector[legNum][0] = legSensor[0]	;
			angleVector[legNum][1] = legSensor[1]	;
			angleVector[legNum][2] = legSensor[2]	+ stepsize;
		}
		else{
			std::cout << "DEFAULT CASE" << std::endl;
			std::cout << lt[0] << " " << lt[1] << " " << lt[2] << " " << lt[3] << " " << lt[4] << " " << lt[5] << std::endl;
		}

		std::vector<double> nextPos;
		temp = { angleVector[legNum][0], angleVector[legNum][1], angleVector[legNum][2] };
		kinematic( nextPos, temp, legNum );

		double dist_fin = sqrt( pow(targetPos[0] - nextPos[0],2) + pow(targetPos[1] - nextPos[1],2) + pow(targetPos[2] - nextPos[2],2) );

		if( dist_fin < threshold ){
			targetPositionPointer[legNum] = targetPositionPointer[legNum]+1;
			std::cout << dist_fin << " - TRUE  TRUE  TRUE  - leg: " << legNum << " state: " << targetPositionPointer[legNum] % positionListL0.size() << " state changed: " << targetPositionPointer[legNum] << std::endl;
			std::cout << "test condition: " << num_test_counter << std::endl;
		}
		else{
			std::cout << dist_fin << " - FALSE FALSE FALSE - leg: " << legNum << " state: " << targetPositionPointer[legNum] % positionListL0.size() << " state changed: " << targetPositionPointer[legNum] << std::endl;
			std::cout << "test condition: " << num_test_counter << std::endl;
		}

		/**
		 * 	The last thing before leaving, is making sure that the joints behave as we want them too.
		 * 	All joints are limited, and then offset. Where a unlimited joint can move in a full circle
		 * 	the dung beetles joints are limited. LPZ takes this limit, and then sets 0 to the middle of
		 * 	this limit
		 */

		postOffset( angleVector, legNum );

		if( angleVector[legNum][0] >= 1 ){ angleVector[legNum][0] = 0.99; };
		if( angleVector[legNum][1] >= 1 ){ angleVector[legNum][1] = 0.99; };
		if( angleVector[legNum][2] >= 1 ){ angleVector[legNum][2] = 0.99; };


		if(false){
			angleVector[legNum][0] = 0;
			angleVector[legNum][1] = 0;
			angleVector[legNum][2] = -1;
		}
}

void kinematicsController::loadPositionVectors(void) {
	/**
	 * Input is row upon rows.
	 * Three data values, for each leg.
	 * These are all stored in a 2D vector.
	 * The 2D vector is 3 for each row,
	 * and the height depends on the resolution
	 * of the data given.
	 */

	std::ifstream myfile ("position.data", std::ios::in);

	/**
	 * 	Make a check on the input strings, that it is empty
	 * 	or that there is a '-' to end.
	 */

	int m = 0;
	while(myfile)
	{
		std::string s;
		if( !getline( myfile, s ) ){
			break;
		}

		std::istringstream ss( s );
		std::vector<double> record;

		while( ss ){
			std::string s;
			if (!getline( ss, s, ',' )){
				break;
			}
			record.push_back( std::stod( s ) );
		}

		for( int i=0; i<record.size(); i=i+3 )
		{
			//todo:	outcomment if input file is very long
			std::cout << record[i] << " " << record[i+1] << " " << record[i+2] << " I= " << i << " m= " << m << " " << record.size() << std::endl;

			//	INPUT TO THE VECTOR
			std::vector<double> temp_vec;

			switch (i) {
				case 0: case 1: case 2:			//	L0
					temp_vec.push_back(record[i]);
					temp_vec.push_back(record[i+1]);
					temp_vec.push_back(record[i+2]);

					positionListL0.push_back( temp_vec );
					break;

				case 3: case 4: case 5:			//	L1
					temp_vec.push_back(record[i]);
					temp_vec.push_back(record[i+1]);
					temp_vec.push_back(record[i+2]);

					positionListL1.push_back( temp_vec );
					break;

				case 6: case 7: case 8:			//	L2
					temp_vec.push_back(record[i]);
					temp_vec.push_back(record[i+1]);
					temp_vec.push_back(record[i+2]);

					positionListL2.push_back( temp_vec );
					break;
				case 9: case 10: case 11:		//	R0
					temp_vec.push_back(record[i]);
					temp_vec.push_back(record[i+1]);
					temp_vec.push_back(record[i+2]);

					positionListR0.push_back( temp_vec );
					break;
				case 12: case 13: case 14:		//	R1
					temp_vec.push_back(record[i]);
					temp_vec.push_back(record[i+1]);
					temp_vec.push_back(record[i+2]);

					positionListR1.push_back( temp_vec );
					break;
				case 15: case 16: case 17:		//	R2
					temp_vec.push_back(record[i]);
					temp_vec.push_back(record[i+1]);
					temp_vec.push_back(record[i+2]);

					positionListR2.push_back( temp_vec );
					break;

				default:
					std::cout << "default" << std::endl;
					break;
			}
		}
		std::cout << std::endl;

		m++;	//	Increment m for the rows for the vectors.
	}
}
