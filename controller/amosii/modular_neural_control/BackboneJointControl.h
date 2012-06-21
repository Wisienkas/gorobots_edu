/*
 * BackboneJoinControl.h
 *  Created on: May 9, 2012
 *      Author: degoldschmidt
 */

#ifndef BACKBONEJOINTCONTROL_H_
#define BACKBONEJOINTCONTROL_H_

#include <vector>
#include <iostream>
#include <cstdlib>
#include "../../utils/ann-framework/ann.h"
using namespace std;


class BackboneJointControl : public ANN {
  public:
    BackboneJointControl();

  private:
    double lowpass;

};

#endif /* FORWARDMODEL_H_ */
