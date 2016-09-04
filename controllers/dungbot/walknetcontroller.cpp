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

#include "walknetcontroller.h"

walknetcontroller::walknetcontroller( void )
{
	walknetSeparateLeg legFL(0);
	walknetSeparateLeg legML(1);
	walknetSeparateLeg legRL(2);
	walknetSeparateLeg legFR(3);
	walknetSeparateLeg legMR(4);
	walknetSeparateLeg legRR(5);

	separateLegs.push_back(legFL);
	separateLegs.push_back(legML);
	separateLegs.push_back(legRL);
	separateLegs.push_back(legFR);
	separateLegs.push_back(legMR);
	separateLegs.push_back(legRR);

	nextLegPos.assign( 4 , 0 );
	legPos.assign( 4 , 0 );
	tmpAEP.assign( 3 , 0 );
}

walknetcontroller::~walknetcontroller( void )
{
	for( int i = 0; i <= 6; i++ )
	{
		separateLegs[i].~walknetSeparateLeg();
	}
}

void walknetcontroller::stepWalknetTripod( const sensor* sensor, std::vector<std::vector<double>> &angleVector, std::vector<std::vector<double>> &velocityVector  )
{
	bool flag = true;

	for( int i = 0; i < 6; i++ )
	{
		if( separateLegs[i].startStance == true || separateLegs[i].startSwing == true )
		{
			flag = false;
		}


		//std::cout << (separateLegs[i].startStance == true || separateLegs[i].startSwing == true) << " ";
	}
	if( switchFlag )
	{
		//std::cout << " F: " << switchFlag << " SW ST SW ST SW ST " << std::endl;
	}
	else
	{
		//std::cout << " F: " << switchFlag << " ST SW ST SW ST SW" << std::endl;
	}

	if( flag )
	{
		//std::cout << "SwitchFlag changes from: " << switchFlag << " to: " << !switchFlag << std::endl;
		switchFlag = !switchFlag;

		if( switchFlag )
		{
			separateLegs[0].startSwing = true;
			separateLegs[2].startSwing = true;
			separateLegs[4].startSwing = true;
			separateLegs[1].startStance = true;
			separateLegs[3].startStance = true;
			separateLegs[5].startStance = true;
		}
		else
		{
			separateLegs[0].startStance = true;
			separateLegs[2].startStance = true;
			separateLegs[4].startStance = true;
			separateLegs[1].startSwing = true;
			separateLegs[3].startSwing = true;
			separateLegs[5].startSwing = true;
		}

	}

	for( int i = 0; i < 6; i++ )
	{
		separateLegs[i].stepWalknetSeprateLeg( sensor, angleVector[i], velocityVector[i] );
	}

}


void walknetcontroller::stepWalknet( const sensor* sensor, std::vector<std::vector<double>> &angleVector, std::vector<std::vector<double>> &velocityVector  )
{
	coordinatingInfluences( sensor );
	for( int i = 0; i < 6; i++ )
	{
		separateLegs[i].stepWalknetSeprateLeg( sensor, angleVector[i], velocityVector[i] );
	}
}

void walknetcontroller::getPhase( std::vector<bool> &phaseVector )
{
	phaseVector.clear();
	for( int i = 0; i < 6; i++ )
	{
		phaseVector.push_back( separateLegs[i].getPhase() );
	}
}

void walknetcontroller::coordinatingInfluences( const sensor* sensor )
{
	/**
	 * 		Do the coordination influences here.
	 * 		Figure out what the legs need to do, and send the
	 * 		signals to the separate legs, including the correction of the AEP.
	 */

	coordinateRule1();
	coordinateRule2();
	coordinateRule3();
	//coordinateRule4( sensor );
}

void walknetcontroller::coordinateRule1( void ){
	for( int i = 5; i>0; i--)
	{
		switch (i) {
			case 1:
			case 4:
				if( separateLegs[i].startSwing == true ){
					separateLegs[i-1].rule1 = 1;
					//std::cout << "suppressed" << i-1 << std::endl;
				}else{
					separateLegs[i-1].rule1 = 0;
				}
				break;
			case 2:
				if( separateLegs[i].startSwing == true ){
					separateLegs[i-1].rule1 = 1;
					separateLegs[i+3].rule1 = 1;
					//std::cout << "suppressed" << i-1 << std::endl;
				}else{
					separateLegs[i-1].rule1 = 0;
					separateLegs[i+3].rule1 = 0;
				}
				break;
			case 5:
				if( separateLegs[i].startSwing == true ){
					separateLegs[i-1].rule1 = 1;
					separateLegs[i-3].rule1 = 1;
					//std::cout << "suppressed" << i-1 << std::endl;
				}else{
					separateLegs[i-1].rule1 = 0;
					separateLegs[i-3].rule1 = 0;
				}
				break;
			default:
				break;
		}
	}
}

void walknetcontroller::coordinateRule2( void ){
	for( int i = 5; i>=0; i--)
	{
		switch (i) {
			case 0:
				// Upon touch-down of a leg, it facilitates lift-off of the 3 leg
				if(separateLegs[i].touch_down == true){
					separateLegs[i].touch_down = false;
					separateLegs[i+3].rule2 = 1;
					//std::cout << "facilitate swing of "<< i+3 << std::endl;
				} else { separateLegs[i+3].rule2 = 0; };
				break;
			case 3:
				// Upon touch-down of a leg, it facilitates lift-off of the 0 leg
				if(separateLegs[i].touch_down == true){
					separateLegs[i].touch_down = false;
					separateLegs[i-3].rule2 = 1;
					//std::cout << "facilitate swing of "<< i-3 << std::endl;
				} else { separateLegs[i-3].rule2 = 0; };
				break;
			case 1:
				// Upon touch-down of a leg, it facilitates lift-off of the 0 and 4 leg
			case 2:
				// Upon touch-down of a leg, it facilitates lift-off of the 1 and 5 leg
				if(separateLegs[i].touch_down == true){
					separateLegs[i].touch_down = false;
					separateLegs[i-1].rule2 = 1;
					separateLegs[i+3].rule2 = 1;
					//std::cout << "facilitate swing of "<< i-1 << "&" << i+3 << std::endl;
				} else { separateLegs[i-1].rule2 = 0; separateLegs[i+3].rule2 = 0; };
				break;
			case 4:
				// Upon touch-down of a leg, it facilitates lift-off of the 1 and 3 leg
			case 5:
				// Upon touch-down of a leg, it facilitates lift-off of the 2 and 4 leg
				if(separateLegs[i].touch_down == true){
					separateLegs[i].touch_down = false;
					separateLegs[i-1].rule2 = 1;
					separateLegs[i-3].rule2 = 1;
					//std::cout << "facilitate swing of "<< i-3 << "&" << i-1 << std::endl;
				} else { separateLegs[i-1].rule2 = 0; separateLegs[i-3].rule2 = 0; };
				break;
			default:
				break;
		}
	}
}


void walknetcontroller::coordinateRule3( void ){
	for( int i = 5; i>=0; i--)
	{
		switch (i) {
			case 2:
				if( separateLegs[i].close_to_PEP == true ){
					separateLegs[i].close_to_PEP = false;
					separateLegs[i+3].rule3 = 1;
				} else { separateLegs[i+3].rule3 = 0; };
				break;
			case 5:
				if( separateLegs[i].close_to_PEP == true ){
					separateLegs[i].close_to_PEP = false;
						separateLegs[i-3].rule3 = 1;
				} else { separateLegs[i-3].rule3 = 0; };
				break;
			case 0:
			case 1:
				if( separateLegs[i].close_to_PEP == true ){
					separateLegs[i].close_to_PEP = false;
						separateLegs[i+1].rule3 = 1;
						separateLegs[i+3].rule3 = 1;
				} else { separateLegs[i+1].rule3 = 0; separateLegs[i+3].rule3 = 0; };
				break;
			case 3:
			case 4:
				if( separateLegs[i].close_to_PEP == true ){
					separateLegs[i].close_to_PEP = false;
						separateLegs[i+1].rule3 = 1;
						separateLegs[i-3].rule3 = 1;
				} else { separateLegs[i+1].rule3 = 0; separateLegs[i-3].rule3 = 0; };
				break;
			default:
				break;
		}
	}
}

void walknetcontroller::coordinateRule4( const sensor* sensor )
{
	for( int i = 5; i>=0; i--)
	{
		switch(i){
			case 0:
			case 1:
			case 3:
			case 4:
			case 2:
			case 5:
				break;
			default:
				std::cout << "Rule 4 error in leg: " << i << std::endl;
				break;
		}
	}
}

double walknetcontroller::calculateRule4Distance( std::vector<double> & currentPos, std::vector<double> & targetPos )
{
	double coxaError = targetPos[0] - currentPos[0];
	double femurError = targetPos[1] - currentPos[1];
	double tibiaError = targetPos[2] - currentPos[2];

	return sqrt( pow(coxaError,2) + pow(femurError,2) + pow(tibiaError,2) );
}
