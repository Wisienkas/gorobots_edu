/*****************************************************************************
*   "THE BEER-WARE LICENSE" (Revision 43):
*   This software was written by Theis Strøm-Hansen <thstroemhansen@gmail.com>
*   and Mathias Thor <mathias.thor@gmail.com>
*   As long as you retain this notice you can do whatever you want with it.
*   If we meet some day, and you think this stuff is worth it, you can buy me
*   a beer in return.
*
*   Should this software ever become self-aware, remember: I am your master
*****************************************************************************/

#ifndef __ODE_ROBOTS_ROBOTS_DUNGBOTKINEMATICSCONTROLLER_H_
#define __ODE_ROBOTS_ROBOTS_DUNGBOTKINEMATICSCONTROLLER_H_

// #include <ode/ode.h>
#include <ode-dbl/ode.h>
#include <selforg/abstractcontroller.h>

// Extra includes
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cmath>
#include <iomanip>

#include "DungBotProperties.h"

class kinematicsController
{
	public:
	//	Public attributes

	public:
	//	Public methods
	kinematicsController( int );
	virtual ~kinematicsController( void );

	void stepKinematicsController( const sensor* sensor, std::vector<double> &, int );
	void resetSwingList();
	void resetStanceList();

	protected:
	//	Protected attributes

	protected:
	//	Protected methods

	private:
	//	Private attributes

	//	std::vector<std::vector<std::vector<double>>> positionList;
	//	First vector is the legs.
	//	Second vector is the positions.
	//	Third vector is the joint positions.

	std::vector<std::vector<double>> positionListL0SW;
	std::vector<std::vector<double>> positionListL1SW;
	std::vector<std::vector<double>> positionListL2SW;
	std::vector<std::vector<double>> positionListR0SW;
	std::vector<std::vector<double>> positionListR1SW;
	std::vector<std::vector<double>> positionListR2SW;
	std::vector<std::vector<double>> positionListL0ST;
	std::vector<std::vector<double>> positionListL1ST;
	std::vector<std::vector<double>> positionListL2ST;
	std::vector<std::vector<double>> positionListR0ST;
	std::vector<std::vector<double>> positionListR1ST;
	std::vector<std::vector<double>> positionListR2ST;

	std::vector<int> targetPositionPointer;

	int counter = 0;
	int num_test_counter = 0;
	int last_case;
	bool phase = false; //todo:control this
	int legNum;

	private:
	//	Private methods
	void rotate( std::vector<std::vector<double>> &, double, double, double );
	void matrixMulti( std::vector<std::vector<double>> &, std::vector<std::vector<double>>, std::vector<std::vector<double>> );
	void kinematic( std::vector<double> &, std::vector<double>, int );
	void homo( std::vector<std::vector<double>> &, std::vector<std::vector<double>>, std::vector<double> );
	void preOffset( std::vector<double> &, int );
	void postOffset( std::vector<double> &, int );
	double preScale( double );
	double postScale( double );

	void loadPositionVectors( void );
	void legPositionControl( const sensor*, std::vector<double> &, int );
	bool legAtPosition( std::vector<double>, double, int );
};

#endif
