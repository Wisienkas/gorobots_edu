/***************************************************************************
 *   Copyright (C) 2010 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *                                                                         *
 *   ANY COMMERCIAL USE FORBIDDEN!                                         *
 *   LICENSE:                                                              *
 *   This work is licensed under the Creative Commons                      *
 *   Attribution-NonCommercial-ShareAlike 2.5 License. To view a copy of   *
 *   this license, visit http://creativecommons.org/licenses/by-nc-sa/2.5/ *
 *   or send a letter to Creative Commons, 543 Howard Street, 5th Floor,   *
 *   San Francisco, California, 94105, USA.                                *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                  *
 *                                            * 
 *                                                                         *
 *   $Log: randomcontroller.h,v $
 *                                                                         *
 ***************************************************************************/
#ifndef __RANDOMCONTROLLER_H
#define __RANDOMCONTROLLER_H

#include <selforg/abstractcontroller.h>
#include <selforg/controller_misc.h>
#include <selforg/noisegenerator.h>
#include <selforg/configurable.h>

#include <assert.h>
#include <cmath>
#include <stdlib.h>
#include <string.h>


#include <selforg/matrix.h>





typedef struct RandomControllerConf {
  double maxdeviation;
//  double WeightH1_H1;
//  double WeightH2_H2;
//  double WeightH1_H2;
//  double WeightH2_H1;
//  double fact;
//  double direction;
//  double bias;
} RandomControllerConf;




/**
 *
 * class for hexapod tripodgait using 12 DOF
 * 
 */
class RandomController : public AbstractController {

public:
  RandomController(const RandomControllerConf conf = getDefaultConf());
  virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0);

  virtual ~RandomController();

  /// returns the name of the object (with version number)
  virtual paramkey getName() const {return name; } 
  /// returns the number of sensors the controller was initialised with or 0 if not initialised
  virtual int getSensorNumber() const { return number_sen; }
  /// returns the mumber of motors the controller was initialised with or 0 if not initialised
  virtual int getMotorNumber() const  { return number_mot; }

  /// performs one step (includes learning). 
  /// Calulates motor commands from sensor inputs.
  virtual void step(const sensor* , int number_sensors, motor* , int number_motors);


  /// performs one step without learning. Calulates motor commands from sensor inputs.
  virtual void stepNoLearning(const sensor* , int number_sensors, 
			      motor* , int number_motors);
			      

  /***** STOREABLE ****/
  /** stores the controller values to a given file. */
  virtual bool store(FILE* f) const;
  /** loads the controller values from a given file. */
  virtual bool restore(FILE* f);  


  static RandomControllerConf getDefaultConf(){
    RandomControllerConf c;

    c.maxdeviation = 0.412; //max deviation of tripodgate12dof controller
//    c.WeightH1_H1  =  1.5;
//    c.WeightH2_H2  =  1.5;
//    c.WeightH1_H2  =  0.4;
//    c.WeightH2_H1  = -0.4;
//    c.fact         =  1.0;//0.7;
//    c.direction    = -1;
//    c.bias         =  0.0; //negative is legs up
   
    return c;
  }
 



protected:
  unsigned short number_sen;
  unsigned short number_mot;

  int t;
  paramkey name;
//  double  outputH1;
//  double  outputH2;
  
  RandomControllerConf conf;
  
  NoiseGenerator* randomgen;
private:
  double* temp_y_;

public:
   
 virtual paramval getParam(const paramkey& key) const{
    if(key == "maxdeviation") return conf.maxdeviation;
//    else if(key == "WeightH2_H2") return conf.WeightH2_H2;
//    else if(key == "WeightH1_H2") return conf.WeightH1_H2;
//    else if(key == "WeightH2_H1") return conf.WeightH2_H1;
//    else if(key == "fact") return conf.fact;
//    else if(key == "direction") return conf.direction;
//    else if(key == "bias") return conf.bias;
//	 else
		 return AbstractController::getParam(key) ;
 }

  virtual bool setParam(const paramkey& key, paramval val){
    if(key == "maxdeviation") conf.maxdeviation=val;
//    else if(key == "WeightH2_H2") conf.WeightH2_H2=val;
//    else if(key == "WeightH1_H2") conf.WeightH1_H2=val;
//    else if(key == "WeightH2_H1") conf.WeightH2_H1=val;
//    else if(key == "fact") conf.fact=val;
//    else if(key == "direction") conf.direction=val;
//    else if(key == "bias") conf.bias=val;
//    else return false; //AbstractController::setParam(key, val);  //nutzt ja keine Parameter davon
    return true;
  }

  virtual paramlist getParamList() const{
    paramlist list;
    list.push_back(std::pair<paramkey, paramval> ("maxdeviation", conf.maxdeviation));
//    list.push_back(std::pair<paramkey, paramval> ("WeightH2_H2", conf.WeightH2_H2));
//    list.push_back(std::pair<paramkey, paramval> ("WeightH1_H2", conf.WeightH1_H2));
//    list.push_back(std::pair<paramkey, paramval> ("WeightH2_H1", conf.WeightH2_H1));
//    list.push_back(std::pair<paramkey, paramval> ("fact", conf.fact));
//    list.push_back(std::pair<paramkey, paramval> ("direction", conf.direction));
//    list.push_back(std::pair<paramkey, paramval> ("bias", conf.bias));
    return list;
  }


};

#endif


