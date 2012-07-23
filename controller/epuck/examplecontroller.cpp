/*
 * This file was created by Tobias Jahn on Tuesday May 01, 2012
 * 
 * This examplecontroller shows the usage of the real epuck interface
 *
 */

#include "examplecontroller.h"

#include <vector>
#include <iostream>
#include <cmath>
#include <fstream>



double getPeakPos(double* in, int middle, int maxrange){
    int peakPos = middle;
    for(int pos= middle-maxrange; pos<middle+maxrange; pos++){
        if(in[pos]>in[peakPos])peakPos=pos;
    }
    return peakPos-middle;
}



ExampleController::ExampleController() : AbstractController("EPuck example controller", "1.0") {
}

ExampleController::~ExampleController() {
}

void ExampleController::init(int sensornumber, int motornumber, RandGen* randGen) {
}


int ExampleController::getSensorNumber() const {
  return motorCount;
}

int ExampleController::getMotorNumber() const {
  return sensorCount;
}

void ExampleController::step(const sensor* sensors, int _sensorCount, motor* motors, int _motorCount) {

      for(int i=0; i<300;i++)buffer[i]=sensors[numOfSensor.MIC0+i];

      //filter noise
      for(int i=0; i<299;i++)  if(abs(buffer[i]-buffer[i+1])>200)   buffer[i+1]=0;
      for(int i=0; i<299;i++)  buffer[i]=   (buffer[i]+buffer[i+1])/2;


      //interpolate the mics linear
      int N = 200;
      for(int i=0; i<3*N; i++){
          i%2==0?  (micData[0][i] = buffer[i/2])  :  (micData[0][i] = (buffer[(i-1)/2]+buffer[(i+1)/2])/2.);
      }

       //take the absolut of the mic-data
       //for(int i=0; i<3*N; i++) micData[0][i]=abs(micData[0][i]);

       //calculate meanSqr (for event-detection)
       double meanSqr=0;    for(int i=0; i<3*N; i++)meanSqr += micData[0][i]*micData[0][i]/3/N;

       if(meanSqr>30){
           //create array for the correlations
           int corrsize=100;
           for(int i=0; i<300;i++)corr[0][i]=0;

           //calculate the correlation. tau=0 is at corrsize/2
           for(int tau=-corrsize/2; tau<corrsize/2; tau++)for(int j=0; j<N; j++){
               int i = tau+corrsize/2;
               corr[0][i] += micData[0][j]*micData[1][(j+tau +N)%N];
               corr[1][i] += micData[1][j]*micData[2][(j+tau +N)%N];
               corr[2][i] += micData[0][j]*micData[2][(j+tau +N)%N];
           }

           double kx, ky;
           kx =  -3.* (getPeakPos(corr[0], corrsize/2, 10)) -6.*(getPeakPos(corr[1], corrsize/2, 10));
           ky =  5.* (getPeakPos(corr[0],  corrsize/2, 10)) -0.*(getPeakPos(corr[1], corrsize/2, 10));
           double phi = atan2(-ky, kx)/M_PI*180;

           phi = ((int)(phi+360))%360;

           for(int i=0; i<8; i++)motors[numOfMotor.LED0+i]=0;
           if(phi < 30) motors[numOfMotor.LED4] = 1;
           else if(phi < 30+45) motors[numOfMotor.LED5] = 1;
           else if(phi < 30+90) motors[numOfMotor.LED6] = 1;
           else if(phi < 30+135) motors[numOfMotor.LED7] = 1;
           else if(phi < 30+180) motors[numOfMotor.LED0] = 1;
           else if(phi < 30+225) motors[numOfMotor.LED1] = 1;
           else if(phi < 30+270) motors[numOfMotor.LED2] = 1;
           else if(phi < 30+315) motors[numOfMotor.LED3] = 1;
           else if(phi < 30+45) motors[numOfMotor.LED4] = 1;

/*
           ofstream data("korr.dat");
           for(int tau=-corrsize/2; tau<corrsize/2; tau++){
               int i = tau+corrsize/2;
               data << tau << "\t" << corr[0][i]  << "\t" << corr[1][i]<< "\t" << corr[2][i] << endl;
           }
           data.close(); data.open("mic.dat");
           for(int i=0; i<N; i++){
               data << micData[0][i] << "\t" << micData[1][i] << "\t" << micData[2][i] << endl;
           }
           data.close(); data.open("peak.dat", ios::app);
               data << getPeakPos(corr[0], corrsize/2, 10) << "\t" << getPeakPos(corr[1], corrsize/2, 10) << "\t" << getPeakPos(corr[2], corrsize/2, 10)<< endl;
           data.close();//
     //*/
       }
}

void ExampleController::stepNoLearning(const sensor*, int _sensorCount, motor*, int _motorCount) {

}

bool ExampleController::store(FILE* f) const {
  return false;
}

bool ExampleController::restore(FILE* f) {
  return false;
}
