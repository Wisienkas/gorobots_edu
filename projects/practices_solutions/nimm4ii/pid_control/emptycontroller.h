#ifndef __EMPTYCONTROLLER_H
#define __EMPTYCONTROLLER_H


#include <selforg/abstractcontroller.h>
#include <iostream>
#include <cmath>

/**************************************************************************
 * Solution Task 1 robot controller.
 * The controller gets a number of input sensor values each timestep
 * and has to generate a number of output motor values.
 *
 * Go to the step() function and enter the control commands with respect to your task!
 *
 * Solution adapted by combining the work of Dennis Meyer and Philipp Hannibal
 *
 *********************************************************************************/


class EmptyController : public AbstractController {
public:


  int numTargets;
  int curTarget;
  double sensorThreshold;
  double turnThreshold ;
  double left;
  double right;
  double reverse ;
  double turnDecay ;
  double stopDis;
  bool stopped;
  int stoppedSteps ;

  double KI;//integral part
  double KP;//proportional
  double KD;//differential

  	double remindSensors[23][10];//to save the sensors' values for the last 10 time steps

    /// contructor (hint: use $ID$ for revision)
    EmptyController(const std::string& name, const std::string& revision)
    : AbstractController(name, revision){

    	numTargets = 3;
    	curTarget = 0;
    	sensorThreshold = 0.1;
    	turnThreshold = 0.2;
    	left = 0;
    	right = 0;
    	reverse = 0;
    	turnDecay = 0.75;
    	stopDis = 5.0;
    	stopped = false;
    	stoppedSteps = 0;

        KI=0;
        KP=1;
    	KD=0;

    	 for(int i =0; i < 23; i++)
    	  for (int j=0;j < 10; j++)
    	    remindSensors[i][j]= 0.0;
    }

  /** initialization of the controller with the given sensor/ motornumber
      Must be called before use. The random generator is optional.
  */
  virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0){
    number_sensors = sensornumber;
    number_motors = motornumber;
  };

  /** @return Number of sensors the controller
      was initialised with or 0 if not initialised */
  virtual int getSensorNumber() const {
    return number_sensors;
  };

  /** @return Number of motors the controller
      was initialised with or 0 if not initialised */
  virtual int getMotorNumber() const {
    return number_motors;
  };

  private:
 void turnLeft(motor* motors) {
        motors[0]= -1;
        motors[1]=  1;
        motors[2]= -1;
        motors[3]=  1;
  }
  void turnRight(motor* motors) {
        motors[0]= 1;
        motors[1]= -1;
        motors[2]= 1;
        motors[3]= -1;
  }
  void moveForward(motor* motors) {
        motors[0]= 0.5;
        motors[1]= 0.5;
        motors[2]= 0.5;
        motors[3]= 0.5;
  }
  void moveBackward(motor* motors) {
        motors[0]= -1;
        motors[1]= -1;
        motors[2]= -1;
        motors[3]= -1;
  }
  void fullStop(motor* motors){
        motors[0]= 0;
        motors[1]= 0;
        motors[2]= 0;
        motors[3]= 0;
  }
  double Distance(double tarX, double tarY) {
        return sqrt(std::pow((tarX),2)+std::pow((tarY),2));
  }


  virtual void step(const sensor* sensors, int sensornumber,
                    motor* motors, int motornumber){
    assert(number_sensors == sensornumber);
    assert(number_motors == motornumber);

/*****************************************************************************************/
// sensors 0-3: wheel velocitiy of the corresponding wheel
// sensors 4-11: IR Sensors
    // front left and front right infrared sensor
    // right infrared sensors
    // rear right and rear left infrared sensor
    // left infrared sensors
// sensors 12-23: distance two objects in local coordinates (x,y,z)
/*****************************************************************************************/

// Example open loop controller:

    int tarX = curTarget*3+12;
    int tarY = curTarget*3+13;
    int tarZ = curTarget*3+14;
    double sensorFrontLeft = sensors[5];
    double sensorFrontRight = sensors[4];
    double sensorLeft = sensors[6];
    double sensorRight = sensors[11];
    double sensorBackLeft = sensors[7];
    double sensorBackRight = sensors[10];

    if(!stopped)
        {
        double sensorsLeft = 0.8*sensorFrontLeft+1*sensorLeft+1*sensorBackLeft;
        double sensorsRight = 0.8*sensorFrontRight+1*sensorRight+1*sensorBackRight;

        if(sensorsLeft>sensorThreshold)
            right = right + sensorsLeft;

        if(sensorsRight>sensorThreshold)
            left = left + sensorsRight;

        if(reverse > 0.0) {
            moveBackward(motors);
            reverse = reverse - 0.5;
        } else {
            if(sensors[tarY]>1) {
                right = right + 0.5;
            }
            if(sensors[tarY]<-1) {
                left = left + 0.5;
            }
            if(right>(turnThreshold+left)) {
                turnRight(motors);
            } else {
                if(left>(turnThreshold+right)) {
                    turnLeft(motors);
                } else {
                    if((left>2) && (right>2) && left<(turnThreshold+right) && right<(turnThreshold+left)) {//&& sensorBackLeft<1 && sensorBackRight<1) {
                        reverse = reverse + 1.0;
                    } else {
                        moveForward(motors);
                    }
                }
            }
        }
        left = left - turnDecay;
        right = right - turnDecay;
        if(left<0)
            left =0;
        if(right<0)
            right =0;

    if(Distance(sensors[tarX],sensors[tarY])<=stopDis)
    {
        stopped = true;

    	for(int j=0;j<number_sensors;j++){
    		  for(int t=0;t<10;t++) {
    		      if(t==10) remindSensors[j][10-t]=sensors[j];//remindSensors[j][0] ist der aktuelle Wert, usw...
    		      else {
    			remindSensors[j][9-t]=remindSensors[j][8-t];
    		      }
    		  }
    	       }
    	       //refresh integral factor for sensor 12:
    	       KI=0;
    	       for(int t=0;t<10;t++){
    		  KI+=0.6*remindSensors[tarX][t];
    	       }
    	       //refresh differential factor for sensor 12:
    	       KD=remindSensors[tarX][0]-remindSensors[tarX][1];//(present value minus previous value) ~ slope

    		//refresh proportional factor for sensor 12:
    		KP=0.8*(sensors[tarX]-stopDis);//distance to stop in front of target: 2 //5
    		//stopMotors(number_motors, motors);

    		driveStraightForward(number_motors, motors, (KP+KD+KI));

    }
    } else {
    	fullStop(motors);
        stoppedSteps++;

       if(sensors[tarY]>0.2) {
             turnRight(motors);
        }
        if(sensors[tarY]<-0.2) {
             turnLeft(motors);
        }
         //if(stoppedSteps==300)
         //   curTarget++;
        if(stoppedSteps>1500)
         {
             stopped = false;
             stoppedSteps = 0;
             curTarget++;
             if(curTarget>=numTargets)
                curTarget = 0;




         }
    }
  };

  /** performs one step without learning.
      @see step
  */
  virtual void stepNoLearning(const sensor* , int number_sensors,
                              motor* , int number_motors){

  };

  virtual void driveStraightForward(int motornumber, motor* motors, double speed) {
       assert(number_motors == motornumber);
       if(speed>1) speed=1;
       if(speed<-1) speed=-1;
       for(int i=0;i<number_motors;i++){
 	  motors[i]=speed;
       }
    }

  /********* STORABLE INTERFACE ******/
  /// @see Storable
  virtual bool store(FILE* f) const {
    Configurable::print(f,"");
    return true;
  }

  /// @see Storable
  virtual bool restore(FILE* f) {
    Configurable::parse(f);
    return true;
  }


protected:

  int number_sensors;
  int number_motors;

};

#endif
