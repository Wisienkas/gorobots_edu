
/*
 * This file was created by Tobias Jahn on Tuesday May 01, 2012
 *
 * This example shows the usage of the real epuck interface
 *
 *To run the program at terminal type:
 *
 *./start /dev/rfcomm0 for Epuck 2358
 *./start /dev/rfcomm1 for Epuck 0990
 *./start /dev/rfcomm2 for Epuck 3202
 *./start /dev/rfcomm3 for Epuck 2899
 *./start /dev/rfcomm4 for Epuck 3068
 */

#include <iostream>
#include <signal.h>
#include <cmath>
#include <string>

#include <selforg/agent.h>
#include <selforg/abstractrobot.h>
#include <selforg/one2onewiring.h>

#include "epuckbluetooth.h"

#include "examplecontroller.h"


using namespace std;
using namespace lpzrobots;

volatile bool abortLoop;
void endLoop(int){abortLoop=1;}


int main(int argc, char** argv)
{
  cout << endl << endl;
  cout << "Start program!" << endl;

  cout << "Create config..." << endl;
  EPuckConf conf = EPuckBluetooth::getDefaultConfig();

  conf.SENSOR_STATE = true; //true = use all sensors except micro & camera, false = not using all sensors except micro & camera
  conf.MIC_STATE = true; //true = use microphone, false = not using microphone
  conf.CAM_STATE = false;
  conf.CAM_HEIGHT = 40; // 2 small res
  conf.CAM_WIDTH = 40; // 10  small res
  conf.CAM_TYPE = 0; //0 = black and white, 1= color
  conf.CAM_ZOOM = 8;//1 2 4 8 or 16

  //40x40 = 1600 pixel or 2x10 = 20 pixel * 3 colors
  //maximum of Pixels is 3200

  if (argc !=1)  conf.port = argv[1];



  cout << "Create robot (EPuckBluetooth)..." << endl;
  EPuckBluetooth *robot = new EPuckBluetooth(conf);
  AbstractWiring *wiring = new One2OneWiring(new WhiteUniformNoise(),true);
  ExampleController *controller = new ExampleController();
  Agent* agent = new Agent();
  agent->init(controller, robot, wiring);

  cout << "Created. Embedded softwareversion:\n\t" << (char*)robot->version;
  usleep(1e6);

  cout << "optimal connection speed:\t" << robot->getRecommendConnectionSpeed(conf) << "Hz" << endl;


  int sensorCount, motorCount;
  SensorNumbers numOfSensor;
  MotorNumbers numOfMotor;
  robot->getNumOfMotSens(numOfSensor, sensorCount, numOfMotor, motorCount);
  double *sensors = new double[sensorCount];     for(int i=0; i<sensorCount;i++)    sensors[i]=0;

  controller->setSensorMotorNumbers(numOfSensor, sensorCount, numOfMotor, motorCount);
  controller->setConf(conf);
  cout << "Sensors: " << sensorCount << "\t\tMotors: " << motorCount << endl;

  if(!robot->isRunning()){
    cout << "Enter eventloop. Press Strg+C to exit." << endl;
    signal(SIGINT, endLoop); //catch strg+c to exit eventloop

    double Hz=robot->getRecommendConnectionSpeed(conf), t=0; abortLoop=0;

    while(!abortLoop){

      //use a controller
      controller->t=t;
      agent->step(0,t);
      //end using a controller

      const int n=Hz;//n = outputs per second
      if( (int)(n*t)!=(int)(n*(t+1./Hz))){
        robot->getSensors(sensors,sensorCount);
        cout << "\r\033[K" << flush; // clear line
        //cout << "\r";  for(int i=0; i<sensorCount&&i<22; i++) cout << sensors[i] << "\t" << flush;   cout << "\tHz =" << robot->connectionSpeedHz << "\tTime = " << (int)t << flush ;
      }
      usleep((int)1.e6/Hz);

      t+=1./Hz;
    }
  } // end if(isRunning())
  else{
    cout << "Connection is not running" << endl;
  }

  cout << endl << "Exit program!" << endl;
  cout << endl << endl;

  delete controller;
  delete agent;
  delete wiring;
  delete robot;
  delete sensors;
  return 0;

}//*/
