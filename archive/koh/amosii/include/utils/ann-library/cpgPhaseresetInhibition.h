/*
 * SO2CPGPhaseresetPhasereset.h
 *
 *  Created on: Jan 22, 2013
 *      Author: subhi
 */

#ifndef SO2CPGPhasereset_H_
#define SO2CPGPhasereset_H_
#include "so2cpg.h"
#include <selforg/abstractcontroller.h>
#include <ode_robots/amosiisensormotordefinition.h>
//#include "utils/ann-framework/ann.h"
//#include "utils/interpolator2d.h"

/**class SO2CPGPhaseresetPhasereset {
  public:
    SO2CPGPhaseresetPhasereset();

};*/





class CPGPhaseresetInhibition : public SO2CPG
{
public:
    /**
        * The constructor
        */
    typedef double sensor;
    CPGPhaseresetInhibition();
    /**
         * The destructor
         */
    virtual ~CPGPhaseresetInhibition();
   void setFootbias(int footbias_id,double footbias_value);
   void setControlInput(double Control_input);
  double* getFootbias();
    double getInhibiCoeff();
   void setInhibiCoeff(double inhibiCoeffvalue);
   virtual void calculate(const std::vector<double> x,AmosIISensorNames sensorname,bool footSensorWorking,std::vector<double> predictActivity,std::vector<double> predictOutput,std::vector<double> currentOutput);
    virtual void step();
private:
   // std::vector<double> footbias;
    double footbias[2];
    double inhibiCoeff;
    bool prvTouchF;// the legs did not touch the ground previously
    bool touchF;// the legs touch the ground currently
    int count;
    double ContM;
    //double footbias1;

};


#endif /* SO2CPGPhaseresetP_H_ */
