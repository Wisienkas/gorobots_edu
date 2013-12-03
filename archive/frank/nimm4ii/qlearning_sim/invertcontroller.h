/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
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
 *                                                                         *
 *   $Log: invertcontroller.h,v $
 *   Revision 1.10  2006/07/20 17:14:34  martius
 *   removed std namespace from matrix.h
 *   storable interface
 *   abstract model and invertablemodel as superclasses for networks
 *
 *   Revision 1.9  2006/07/14 12:23:58  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.7.6.2  2006/07/10 13:05:16  martius
 *   NON-COMMERICAL LICENSE added to controllers
 *
 *   Revision 1.12.6.4  2006/06/25 21:56:06  martius
 *   configureable has name and revision
 *
 *   Revision 1.12.6.3  2006/03/30 12:35:12  martius
 *   documentation updated
 *
 *   Revision 1.12.6.2  2006/01/18 16:48:10  martius
 *   configurables can be stored and reloaded
 *
 *   Revision 1.12.6.1  2006/01/17 16:58:39  martius
 *   loading and storing
 *
 *   Revision 1.12  2005/08/06 20:47:54  martius
 *   Commented
 *
 *   Revision 1.11  2005/08/03 20:28:57  martius
 *   inspectable interface
 *
 *   Revision 1.10  2005/06/21 15:31:11  martius
 *   getSensorNumber and getMotorMumber added controller interface
 *
 *   Revision 1.9  2005/06/20 09:04:16  martius
 *   init function added to controller-interface
 *
 *   Revision 1.8  2005/06/17 10:45:22  martius
 *   GPL added
 *                                                                 *
 ***************************************************************************/
#ifndef __INVERTCONTROLLER_H
#define __INVERTCONTROLLER_H

#include <selforg/abstractcontroller.h>
#include <stdlib.h>
#include <string.h>

/**
 * Abstract class (interface) for robot controller that use direct matrix inversion and
 * simple one layer networks.
 * 
 * Implements standart parameters: eps, rho, mu, stepnumber4avg, stepnumber4delay
 */
class InvertController : public AbstractController {
public:
  InvertController(const std::string& name, const std::string& revision)
    : AbstractController(name, revision){
     eps=0.3;
     rho=0.0; 
     s4delay=1;
     s4avg=1;
     delta=0.01;
     factor_a=1.0;
     desens=0.0; // name form desensitization
     number_it=0;
     epsilon_it= 0; 
     damping_c=0.0;
  }

  virtual paramval getParam(const paramkey& key) const{
    if(key == "eps") return eps; 
    else if(key == "rho") return rho; 
    else if(key == "desens") return desens;  	
    else if(key == "s4delay") return s4delay; 
    else if(key == "s4avg") return s4avg; 
    else if(key == "factor_a") return factor_a;     
    else if(key == "number_it") return number_it; 
    else if(key == "epsilon_it") return epsilon_it; 
    else if(key == "delta") return delta;             
    else if(key == "damping_c") return damping_c;             
    else  return AbstractController::getParam(key) ;
  }

  virtual bool setParam(const paramkey& key, paramval val){
    if(key == "eps") eps=val;
    else if(key == "rho") rho=val; 
    else if(key == "desens") desens=val;
    else if(key == "s4delay") s4delay=val; 
    else if(key == "s4avg") s4avg=val;
    else if(key == "factor_a") factor_a=val;
    else if(key == "number_it") number_it=val;
    else if(key == "epsilon_it") epsilon_it=val;
    else if(key == "delta") delta=val;                
    else if(key == "damping_c") damping_c=val;                
    else return AbstractController::setParam(key, val);
    //    else fprintf(stderr, "parameter %s unknown\n", key);
    return true;
  }

  virtual paramlist getParamList() const{
    paramlist list;
    list.push_back(std::pair<paramkey, paramval> ("eps", eps));
    list.push_back(std::pair<paramkey, paramval> ("rho", rho));
    list.push_back(std::pair<paramkey, paramval> ("desens", desens));
    list.push_back(std::pair<paramkey, paramval> ("s4delay", s4delay));
    list.push_back(std::pair<paramkey, paramval> ("s4avg", s4avg));
    list.push_back(std::pair<paramkey, paramval> ("factor_a", factor_a));
    list.push_back(std::pair<paramkey, paramval> ("number_it", number_it));
    list.push_back(std::pair<paramkey, paramval> ("epsilon_it", epsilon_it));
    list.push_back(std::pair<paramkey, paramval> ("delta", delta));
    list.push_back(std::pair<paramkey, paramval> ("damping_c", damping_c));

    return list;
  }

  paramval eps;
   paramval rho;
   paramval desens;
   paramval s4delay;
   paramval s4avg;
   paramval factor_a;
   paramval number_it;
   paramval epsilon_it;
   paramval delta;
   paramval damping_c;
protected:
 

};

#endif
