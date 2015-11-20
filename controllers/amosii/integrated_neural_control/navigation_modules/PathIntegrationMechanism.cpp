/*
 * PathIntegrationMechanism.cpp
 *
 *  Created on: 22.10.2015
 *      Author: Dennis Goldschmidt
 */

#include "PathIntegrationMechanism.h"

PathIntegration::PathIntegration(int numneurons, double noise, NoiseType type, double leak) : CircANN(numneurons) {
	head_direction = new CircANN(numneurons);
	addSubnet(head_direction);
	gater = new CircANN(numneurons);
	addSubnet(gater);
	memory = new CircANN(numneurons);
	addSubnet(memory);


	for(unsigned int index = 0; index < numneurons; index++){
		w(gater->getNeuron(index), head_direction->getNeuron(index), 1);                            //  AVN paper Eq. 5
		b(gater->getNeuron(index), 1);                                                              //  AVN paper Eq. 5
		w(memory->getNeuron(index), gater->getNeuron(index), 1);                                    //  AVN paper Eq. 7
		w(memory->getNeuron(index), memory->getNeuron(index), 1-leak);                              //  AVN paper Eq. 5
		for(unsigned int jindex = 0; jindex < numneurons; jindex++)
			w(getNeuron(jindex), memory->getNeuron(index), cos(2*M_PI*(index-jindex)/numneurons));  //  AVN paper Eq. 9
	}
}

PathIntegration::~PathIntegration(){

}

double PathIntegration::getHVAngle(){
	return getVecAvgAngle();
}

double PathIntegration::getHVLength(){
	return getVector().length();
}

osg::Vec3f PathIntegration::getHV(){
	return getVector();
}

void PathIntegration::step(double angle, double speed){
	for(unsigned int index = 0; index < head_direction->N(); index++)
		setInput(head_direction->getNeuron(index), cos(angle - (2*M_PI*index/head_direction->N())));
	head_direction->step();
	gater->step();
	memory->step();
	CircANN::step();
}
