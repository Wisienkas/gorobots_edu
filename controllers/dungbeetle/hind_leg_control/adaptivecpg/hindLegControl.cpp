/*
 * hindLegControl.cpp
 *
 *  Created on: Nov 3, 2014
 *      Author: Giuliano Di Canio 
 *      Comments: please note that the TC feedback range is here set to -45 to 45.
 *      If you change the feedback range in the serial communication class, you have to change it in here as well.
 *       double per=((feedbackFiltered_1+45)/(90))*0.4-0.2;
 *       Shift register is only to have a bigger control of the final frequency, mostly for testing purposes.
 */
#include <controllers/dungbeetle/hind_leg_control/adaptivecpg/hindLegControl.h>
#include <selforg/controller_misc.h>
#include <math.h>


using namespace std;

hindLegControl::hindLegControl():
	AbstractController("hindLegControl", "$Id: hindLegControl.cpp,v 0.1 $"){
	initialize();
	osc=new plastic(0.2,0.2,0.2,0.04*2*3.14,1.01,0.01);//CPG
	plot.open("/home/hp/Documents/dungBeetle.dat");//writing data to file.. change this to your directory
	// TODO Auto-generated constructor stub
	filterJoint1= new lowPass_filter(0.5);//low pass filters
	filterJoint2= new lowPass_filter(0.2);
	filterJoint3= new lowPass_filter(0.2);

	inputDerivative.push_back(0);//vector to compute derivative
	inputDerivative.push_back(0);

	reg=new shift_register(0);// STATIC shift register, bigger is its dimension, lower will be the final reached frequency

	//reg2=new shift_register(20);


	//Add Delay line /////////////////////////////////////////////////////
	tau = 4;
	tr_delayline = new Delayline(2 * tau + 1/*size of delayline buffer*/);
	///////////////////////////////////////////////////////////////////////
}

hindLegControl::~hindLegControl() {
	// TODO Auto-generated destructor stub
}


void hindLegControl::init(int sensornumber, int motornumber, RandGen* randGen) {

  numbersensors = sensornumber;
  numbermotors = motornumber;
  x.resize(sensornumber);
  y.resize(DUNGBEETLE_MOTOR_MAX);
  //y.at(0)=0;
  //y.at(1)=0;
  //y.at(2)=0;
  count=0;
 std::cout << "hi";

}

std::vector<double> hindLegControl::getMaxMinFeedback(double feedback, double max, double min)
{
	std::vector<double> fin;
	if(feedback > max)
		max=feedback;
	if(feedback < min)
		min=feedback;

   fin.push_back(max);
   fin.push_back(min);
   return fin;
}


void hindLegControl::initialize()//this function will be more complex when adding function to the robot
{
t=0;

}

void hindLegControl::step(const sensor* x_, int number_sensors,motor* y_, int number_motors)
{

	assert(number_sensors == numbersensors);
	assert(number_motors == numbermotors);

	for(unsigned int i=0; i<(numbersensors);i++)
		{
			x.at(i) = x_[i];//READ SENSOR S VALUE FROM HERE

		}

	std::cout << t << std::endl;

	//IMPORTANT!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	//Increase w2p in plastic.cpp constructor for faster adaptation

	feedbackFiltered_1=filterJoint1->update(x.at(0));//cleaning the feedbacks
	feedbackFiltered_2=filterJoint2->update(x.at(1));
	feedbackFiltered_3=filterJoint3->update(x.at(2));

	rangeLimit = true;

	
	
	
	// generating singal SO, where deltaPhi=0.2*pi
	 double input = 6*cos(0.02*3.14)*osc->getOut0()+6*sin(0.02*3.14)*osc->getOut1();
	


	 //from +-45 (feedback range) to +- 0.2 (CPG) range
	 double per=((feedbackFiltered_1+45)/(90))*0.4-0.2;


	 double input_secJ;// CT joint
	inputDerivative[0]=inputDerivative.at(1);
	inputDerivative[1]=input;

	derivative=inputDerivative.at(1)-inputDerivative.at(0);//S0 derivative
	std:cout << t << std::endl;

	input=reg->update(input);//shift register
	

	//Generating CT signals

	if(derivative > 0 && input > 0)
	{	
	//if(derivative > 0)
		input_secJ=input;
		if (rangeLimit)
		{
		//((value-oldMin)/(oldMax-oldMin)) * (newMax-newMin) + newMin;
		input_secJ=((input_secJ+1.1)/(2.4))*(1.8)-1.1;
		}
	}
	else
	{
		input_secJ=-1.1;
	}
	//
	
	//input_secJ=reg2->update(input_secJ);

	osc->update(0);//updating CPG  with external perturbation USE PER


	//writing to file
	plot << t << " " << osc->getFrequency()*0.7/0.02<<  " "<<x.at(0)<< " "<<input
			<<" "<<input_secJ<<" " <<osc->getOut2()<< " " <<osc->getOut0() << " " <<osc->getOut1()
			<<" " << osc->getW02() << " "<<osc->getW20() <<" "<< osc->getW2p() <<" "<<std::endl;



	//Add Delay line //////////////////////
	//writing values into delayline buffer
	tr_delayline->Write(input);
	input_delay = tr_delayline->Read(tau);
	tr_delayline->Step();
	////////////////////////////////////////

	// writing commands to the motors
	y_[0]= input_delay;//input;
	y_[1]= input_secJ;
	y_[2]= 0;//-input_secJ;



	t++;
}

bool hindLegControl::store(FILE* f) const {
  return true;
}

/** loads the controller values from a given file. */
bool hindLegControl::restore(FILE* f) {
  //	Configurable::parse(f);
  return true;
}

