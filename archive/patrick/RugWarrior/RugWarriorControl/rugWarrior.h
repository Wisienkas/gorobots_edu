#ifndef SB_RW_H_
#define SB_RW_H_

#include <comedilib.h>
#include <libconfig.h++>
#include <string>
#include <sstream>
#include <fstream>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>

#include "ximuAccess.h"
#include "laserScanner.h"

namespace RWControl{

//Controller class for the RugWarrior robot (http://www.robotbooks.com/rug_warrior.htm) with IMU and LRF attached to it.
class RugWarrior{
public:
	//Initialize and connect robot and IMU
	//cfgFileName: File, where parameters for robot are stored
	RugWarrior(const char* cfgFileName);

	//Destructor to properly disconnect device
	~RugWarrior();

	//Checks if robot is connected
	bool isConnected();
	//Check if robot is logging
	bool isLogging();

	//Manually connect robot by specifying the device name
	bool connect(const char* deviceName);
	//Disconnect robot
	void disconnect();

	//Let robot move forward with a given velocity until other command is given
	//velo: Velocity with which robot should move. Values should be in range [-2047, 2047]. Other
	//		values will be rejected as they could damage the motors
	void moveStraight(int velo);
	//Move forward with default velocity (given in parameter file) until other command is given
	void moveForward();
	//Move backward with default velocity (given in parameter file) until other command is given
	void moveBackward();
	//Robot stops
	void stop();
	//Let robot turn with a given angular velocity until other command is given
	//omega: Velocity with which robot turns, should be in range [-2047, 2047].
	//	  	 Negative values turn clockwise
	void turn(int omega);
	//Turn right with default velocity (given in parameter file) until other command is given
	void turnRight();
	//Turn left with default velocity (given in parameter file) until other command is given
	void turnLeft();

	//Start logging in background thread. Robot will run this thread until
	//it is explicitely stopped
	void startLogging();
	//Stop background thread
	void stopLogging();

	//Set motors to given values. Both values should be in range [1000, 3995]. Values outside this
	//range will be rejected as they might damage the motors.
	void setMotors(int valueLeft, int valueRight);

	//Write velocities, positions and imu data to file
	void writeDataToFile(std::string dataFolder);

	//Reset velocities of IMU to prevent integration drift
	void resetVelocityIMU();

	//Returns how many data points are taken (in fact size of velocities vector)
	int getSizeData();

	//Return (average) time in sec between two velocity (and also position) samples.
	//Returned value is only valid after logging has been started and stopped again
	double getSampleTime();

private:
	//Inertial Measurement Unit
	XIMU::Ximu imuDevice;

	//LRF
	URG::URGLaser urg;

	//Used to calculate sample time of velocity and position
	boost::chrono::high_resolution_clock::time_point startPoint;
	boost::chrono::duration<double> totalTime;

	//Velocities and position
	std::vector<std::vector<double> > velocities;
	std::vector<std::vector<double> > positions;
	std::vector<int> indicesVelo;
	//LRF data
	std::vector<std::vector<long> > urgData;
	std::vector<int> indicesLRF;

	//Check if proposed velocity is withing limits of motors
	lsampl_t checkMotorLimit(double velo);
	//Read from touch sensors and calculate if and where touch was received
	int calcTouchValue();
	//Init robot
	void init(const char* cfgFileName);
	//Robot moves forward and avoids obstacles based on touch sensors (again until other command is given)
	void doLogging();

	//Check if robot is connected
	bool connected;
	//Check if robot is doing OA
	bool logging;
	//Check if robot is currently turning
	bool isTurning;

	//Instance of device
	comedi_t *comediDev;

	//Thread in which to do logging
	boost::thread threadLogging;

	//Sample rate of lrf
	int sampleRateLRF;
	//Sample rate velocity
	int sampleRateVelo;

	//Variables needed for obstacle avoidance
	int lastTouch;
	int refrPeriod;

	int obstacleThreshold; //Threshold of touch sensors for starting obstacle avoidance
	int refrPeriodBase; //Refractory period of obstacle avoidance
	//Define avoidance movement
	int steerGainOA;
	int brake;
	int biasLeft;
	int biasRight;

	//Default translational and rotational velocity
	int veloStraight;
	int veloRot;

};
}

#endif
