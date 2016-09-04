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

#ifndef __ODE_ROBOTS_ROBOTS_DUNGBOTWALKNETCONTROLLER_H_
#define __ODE_ROBOTS_ROBOTS_DUNGBOTWALKNETCONTROLLER_H_

// #include <ode/ode.h>
#include <ode-dbl/ode.h>
#include <selforg/abstractcontroller.h>

// Extra includes
#include <vector>
#include <iostream>
#include <string>
#include "walknetSeparateLeg.h"


class walknetcontroller
{
	public:
	//	Public attributes

	public:
	//	Public methods
	walknetcontroller( void );
	virtual ~walknetcontroller( void );

	void stepWalknetTripod( const sensor* sensor, std::vector<std::vector<double>> &, std::vector<std::vector<double>> & );
	void stepWalknet( const sensor* sensor, std::vector<std::vector<double>> &, std::vector<std::vector<double>> & );
	void getPhase( std::vector<bool> & );

	protected:
	//	Protected attributes

	protected:
	//	Protected methods

	private:
	//	Private attributes
	std::vector<walknetSeparateLeg> separateLegs;
	std::vector<double> legPos;
	std::vector<double> nextLegPos;
	std::vector<double> tmpAEP;

	//	Used for testing the tripod gait
	bool switchFlag = false;

	private:
	//	Private methods
	void coordinatingInfluences( const sensor* );
	void coordinateRule1( void );
	void coordinateRule2( void );
	void coordinateRule3( void );
	void coordinateRule4( const sensor* sensor );

	double calculateRule4Distance( std::vector<double> &, std::vector<double> & );



};

#endif
