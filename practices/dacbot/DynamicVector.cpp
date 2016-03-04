/*
 * DynamicVector.cpp
 *
 *  Created on: Apr 14, 2015
 *      Author: giuliano
 */

#include <DynamicVector.h>

DynamicVector::DynamicVector(int size) {
	// TODO Auto-generated constructor stub
	for(int i=0;i<size;i++)
		vec.push_back(0);
}

int DynamicVector::getSize()
{
	return vec.size();
}

void DynamicVector::changeSize(unsigned long int newSize)
{
	double temp=vec.at(vec.size()-1);
	vec.resize(newSize,temp);
}
DynamicVector::~DynamicVector() {
	// TODO Auto-generated destructor stub
}

double DynamicVector::update(double value)
{
    if(vec.size() == 0)
    	return value;
    else if(vec.size() == 1)
	{
    	double temp=vec.at(0);
    	vec[0]=value;
    	return temp;

	}

    else
    {
	double temp=vec.at(0);
    for(int i=0;i<=vec.size()-2;i++)
		vec[i]=vec.at(i+1);
	vec[vec.size()-1] = value;

    return temp;
    }
}
