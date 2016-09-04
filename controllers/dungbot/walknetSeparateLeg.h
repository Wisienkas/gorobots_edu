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

#ifndef __ODE_ROBOTS_ROBOTS_DUNGBOTWALKNETSEPARATELEG_H_
#define __ODE_ROBOTS_ROBOTS_DUNGBOTWALKNETSEPARATELEG_H_

// #include <ode/ode.h>
#include <ode-dbl/ode.h>
#include <selforg/abstractcontroller.h>

// Extra includes
#include <vector>
#include <iostream>
#include <string>
#include <math.h>
#include <time.h>


class walknetSeparateLeg
{
	public:
	//	Public attributes
	bool startSwing = false;
	bool startStance = true;
	bool supress_swing = false;
	bool pre_touch_down = false;
	bool touch_down = false;
	bool close_to_PEP = false;

	clock_t begin = 0;
	clock_t end = 0;

	int rule1 = 0, rule2 = 0, rule3 = 0, rule4 = 0;

	int swingState;
	int stanceState;
	enum swingState { LIFT, LOWER, TO_AEP_SWING, FINAL_SWING_POS, IDLE_SWING };
	enum stanceState { IDLE_STANCE, TO_PEP_STANCE, TO_MID_STANCE };

	public:
	//	Public methods
	walknetSeparateLeg( );
	walknetSeparateLeg( int legNum );
	virtual ~walknetSeparateLeg( void );
	void stepWalknetSeprateLeg( const sensor* sensor, std::vector<double> &, std::vector<double> &  );
	//	Used by the walknet to make the control laws for the legs.
	void extractSensor( const sensor* sensor, int leg, std::vector<double> & );
	void setAEP( std::vector<double> & );
	void setPEP( double );
	void setRule( int, bool );
	void getAEP( std::vector<double> & );
	std::vector<double> getPEP( void );
	bool atPosition( std::vector<double> , double);
	bool atAngle( double targetPos, int legPartNum, double deadband );
	bool getPhase();
	bool getGroundContact();

	protected:
	//	Protected attributes

	protected:
	//	Protected methods,.

	private:
	//	Private attributes
	int legNum;
	bool phase = false;

	std::vector<double> PEP;
	std::vector<double> MID;
	std::vector<double> STM;
	std::vector<double> AEP;
	std::vector<double> maxAEP;
	std::vector<double> localSensorArray;
	std::vector<bool> coordinationRules;

	bool STANCE_REACHED	= false;
	bool SWING_REACHED	= false;

	//bool RSunit 	= false;	//	Return Stroke unit (swing movement)
	//bool PSunit	= false;	//	Power Stroke unit (stance movement)
	//bool GCunit 	= false;	//	Ground Contact
	//bool PEPunit 	= false;	//	Boolean value if the leg is in the PEP position.

	int RSunit 	= 0;	//	Return Stroke unit (swing movement)
	int PSunit	= 0;	//	Power Stroke unit (stance movement)
	int GCunit 	= 0;	//	Ground Contact
	int PEPunit = 0;	//	Boolean value if the leg is in the PEP position.

	private:
	//	Private methods
	void selectorNet( const sensor* sensor, std::vector<double> &, std::vector<double> & );
	void stanceNetSimple( const sensor* sensor, std::vector<double> &, std::vector<double> & );
	void swingNetSimple(  const sensor* sensor, std::vector<double> &, std::vector<double> & );
	void stanceNet1( const sensor* sensor, std::vector<double> &, std::vector<double> & );
	void stanceNet2( const sensor* sensor, std::vector<double> &, std::vector<double> & );
	void stanceNet3( const sensor* sensor, std::vector<double> &, std::vector<double> & );
	void stanceNet_maxmin( const sensor* sensor, std::vector<double> & );
	void swingNet1( const sensor* sensor, std::vector<double> &, std::vector<double> & );
	void swingNet2( const sensor* sensor, std::vector<double> &, std::vector<double> & );
	void swingNet3( const sensor* sensor, std::vector<double> &, std::vector<double> & );
	double trajectory( double, int );


};

#endif
