/*
 * DynamicCpg.cpp
 *
 *  Created on: Apr 2, 2015
 *      Author: Giuliano Di Canio
 */

#include <controllers/runbotii_dacbot/DynamicCpg.h>



DynamicCpg::DynamicCpg(double cpgInitialFrequency) {


	enableCpgController=false;//reflexive controller starts
	dynRegister=new DynamicVector(4);
	//build derivative vectors with size 2//
	motor0Derivative.push_back(0);
	motor0Derivative.push_back(0);

	cpgOut1Derivative.push_back(0);
	cpgOut1Derivative.push_back(0);

	signal0Derivative.push_back(0);
	signal0Derivative.push_back(0);

	maxDer.push_back(0);
	maxDer.push_back(0);

	maxKneeDer.push_back(0);
	maxKneeDer.push_back(0);
	shiftVector.push_back(0);
	shiftVector.push_back(0);

	signal2Derivative.push_back(0);
	signal2Derivative.push_back(0);

	hipAngleDerivative.push_back(0);
	hipAngleDerivative.push_back(0);

	systemVectorFreq.push_back(0);
	systemVectorFreq.push_back(0);

	delay.push_back(0);
	delay.push_back(0);

	der.push_back(0);
    der.push_back(0);

    derOut1.push_back(0);
    derOut1.push_back(0);

    derOut2.push_back(0);
    derOut2.push_back(0);
	//
    delayFeedback=false;
    delayFiltered=false;
	cpg=new plastic(0.2,0.2,0.2,cpgInitialFrequency*2*3.14,1.01);//1.01//0.01
	checkSignal=new derivativeTransitionRegister(4,1); //the 4 has actually no effect, check instead the 1
	filterFeedback=new lowPass_filter(0.2);//0.2
	feedbackDelay= new shift_register(4);//3//56
	minFeedback =1000; //needs to be a high number, just to allow the min function to work
	maxFeedback=0;
	step=0;
	cpgPlot.open("/home/giuliano/Documents/thesis/plots/twoleg.dat");
	}


double DynamicCpg::changeRange(double oldMin, double oldMax, double newMin, double newMax, double value)
{

	return ((value-oldMin)/(oldMax-oldMin)) * (newMax-newMin) + newMin;

}


double DynamicCpg::generateHip(double signal, double derivative,double oscillation, double value)
{
	std::vector<double> result;
			double right, left;
			if (signal> oscillation)
			  {

				if (derivative >= 0)
				{
					left=value;
				}

				else
				{
					left=0;
				}

			  }

			  if (signal <= oscillation)
			  {

				  if (derivative < 0)
				  {
					left=-value;
				  }

				  else
				  {
					left=0;

				  }

			  }


			  return left;


}

double DynamicCpg::generateKnee(double signal, double derivative,double oscillation, double valueHip, double valueKnee)
{
	double temp=changeRange(-valueHip, valueHip, -valueKnee, valueKnee,signal);
	double result;
	if (signal > oscillation)
	{
		if (derivative >= 0)
			result=-valueKnee;//temp
		else result=valueKnee;
	}

	if (signal <= oscillation)
		result=valueKnee;

	return result;

}

double DynamicCpg::getAmplitudeHips(double signal, double max)
{
	if (signal > 0 && signal > max)
		max=signal;

	return max;

}

double  DynamicCpg::getAmplitudeKnee(double signal_knee, double max)
{
	if (signal_knee > 0 && signal_knee > max)
			max=signal_knee;

		return max;

}


std::vector<double> DynamicCpg::generateOutputOneLeg(double feedback, double motor0, double motor1, double stepValue)
{
	if(feedback!=0)
			countPerturbation++;//counting how many steps the perturbation is present
		else countPerturbation=0;

		//std::cout << "Perturbation time " << countPerturbation << std::endl;

		std::vector<double> output;
		double temp;
		step = stepValue;
		//computing derivative motor 0
		motor0Derivative[0]=motor0Derivative.at(1);
		motor0Derivative[1]=motor0;
		double motor0DerivativeValue=motor0Derivative[1]-motor0Derivative[0];//Derivative of the motor signal, left hip
		//
		double per,tem,deriv;
		if(feedback!= 0)//if there is a perturbation then is processed, filtered and change range
		{
		per=filterFeedback->update(feedback);
		tem=changeRange(minFeedback,maxFeedback,-0.2,0.2,per);

		der[0]=der.at(1);
		der[1]=tem;
	    deriv=der.at(1)-der.at(0);
		}
		else tem=0;//otherwise set perturbation to zero
	    int oscillationFeedback=(maxFeedback+minFeedback)/2;//get the oscillation point of the perturbation
	    bool look=true;


	    cpgPlot << step << " " << feedback <<" " <<  tem<<" "<<getShiftDelay(cpg->getOut1(), cpg->getOut2(), step, cpg->getFrequency() )<< std::endl;
	    //compute delay between perturbation and filtered one
	    if(feedback > oscillationFeedback && counterDelayFeedback==0 && delayFeedback==false )
	    {
	    	delay[0]=step;
	    	counterDelayFeedback++;
	    	delayFeedback=true;
	    }
	    if(per > oscillationFeedback && counterDelayFiltered==0 && delayFiltered==false )
	    {
	       	delay[1]=step;
	       	counterDelayFiltered++;
	       	delayFiltered=true;
	     }
	    if(feedback< oscillationFeedback)
	    	counterDelayFeedback=0;

	    if(per<oscillationFeedback)
	    	counterDelayFiltered=0;



	    if(delayFeedback==true && delayFiltered==true )
	    {
	    	delayValue=delay.at(1)-delay.at(0);
	    	delayFeedback=false;
	    	delayFiltered=false;



	    }



		//compute delay... the delay is in delayValue


		//cpgPlot << step << " " << feedback << " " << per<< " " << deriv<<std::endl;

		double perturbation=0;



		if(step > 1000 )// for 1000 steps perturbation =0
			perturbation=tem;


		cpg->update(perturbation);//apply perturbation to cpg

        std::vector<double> vec;
	    if (feedback!=0)//check max of feedback, only if present
	    {
	    	vec=getMaxMinFeedback(feedback, maxFeedback,minFeedback);

	    	maxFeedback=vec.at(0);
	    	minFeedback=vec.at(1);
	    }



	    // Derivative CPG 1

	    cpgOut1Derivative[0]=cpgOut1Derivative.at(1);
	    cpgOut1Derivative[1]=cpg->getOut1();
	    double out1DerivativeCpg =cpgOut1Derivative[1]-cpgOut1Derivative[0];

	    //Dynamic register. IF for test purposed want to show that the CPG increases its frequency after switiching
	    //from reflexive-based to CPG-based controller, comment next 2 lines and uncomment double test= cpg.....
	    //this way there is not dynamic register

	    if(dynRegister->getSize() != delayValue && step > 400 && feedback!=0 && delayValue > 0 && delayValue < dynRegister->getSize()+2)//change size of register according to delay in feedback
	      dynRegister->changeSize(delayValue+2);//+2

	     double test= dynRegister->update(cpg->getOut1());//generate cpg1 output


	    //double test=cpg->getOut1();



	    //std::cout << "step1 " << delay.at(1) << "step2 " << delay.at(0) << "Delay  " << delayValue << "   ShiftRegisterDimention  " << dynRegister->getSize() << std::endl;

        temp=generateHip(test, out1DerivativeCpg, 0, amplitudeMotor0);

        output.push_back(temp);


	    signal0Derivative[0]=signal0Derivative.at(1);
	    signal0Derivative[1]=output.at(0);//leftHip
	    double derivativeLeftHip= signal0Derivative[1] - signal0Derivative[0];


	    amplitudeMotor0=getAmplitudeHips(motor0,amplitudeMotor0);
	    amplitudeMotor2=getAmplitudeHips(motor1,amplitudeMotor2);

	   // std::cout << "ampl" << amplitudeMotor0 << " " << amplitudeMotor2 << std::endl;

	    temp=generateKnee(test, out1DerivativeCpg,0, amplitudeMotor0, amplitudeMotor2);

	    output.push_back(temp);//leftknee  2


	    cpgSignal0=output.at(0);

	    return output;
}
plastic* DynamicCpg::getCpg()
{
	return cpg;
}

std::vector<double> DynamicCpg::getGeneratedMotorsSingle()
{

	std::vector<double> temp;
	temp.push_back(cpgSignal0);
	temp.push_back(cpgSignal1);
	return temp;
}


std::vector<double> DynamicCpg::getGeneratedMotorsDouble()
{

	std::vector<double> temp;
	temp.push_back(cpgSignal0);
	temp.push_back(cpgSignal1);
	temp.push_back(cpgSignal2);
	temp.push_back(cpgSignal3);


	return temp;
}



double DynamicCpg::getAbsol(double a, double b)
{
	double result=a-b;
	if (result>0)
		return result;
	else return -result;

}


double DynamicCpg::getShiftDelay(double out1, double out2, int step, double frequency)
{
	derOut1[0]=derOut1.at(1);
	derOut1[1]=tanh(20*out1);
	double derOut1Value=derOut1.at(1)-derOut1.at(0);

	derOut2[0]=derOut2.at(1);
	derOut2[1]=tanh(15000*out2);
	double derOut2Value=derOut2.at(1)-derOut2.at(0);

	if(tanh(20*out1)>0 && derOut1Value>0 && counterDelay1Value==0)
	{
		shiftVector[0]=step;
		countDelay1=true;
		counterDelay1Value++;
	}

	if(tanh(15000*out2)>0 && derOut2Value>0 && counterDelay2Value==0)
	{
		shiftVector[1]=step;
		countDelay2=true;
		counterDelay2Value++;
	}

	if(countDelay1==true && countDelay2==true)
	{
		phaseDelay=shiftVector.at(1)-shiftVector.at(0);
		countDelay1=false;

		countDelay2=false;

	}

	if(tanh(20*out1) == 0 && tanh(15000*out2)== 0)
	{
		counterDelay1Value=0;
		counterDelay2Value=0;

	}


	//std::cout << "CountDelay" << countDelay<<std::endl;;
	return phaseDelay;

}


std::vector<double> DynamicCpg::generateOutputTwoLegThereshold(double feedback, double motor0, double motor1, double stepValue)
{
	if(feedback!=0)
		countPerturbation++;
	else countPerturbation=0;

	//std::cout << "Perturbation time " << countPerturbation << std::endl;

	std::vector<double> output;
	std::vector<double> temp;
	step = stepValue;
	//computing derivative motor 0
	motor0Derivative[0]=motor0Derivative.at(1);
	motor0Derivative[1]=motor0;
	//double motor0DerivativeValue=motor0Derivative[1]-motor0Derivative[0];//Derivative of the motor signal, left hip
	//
	double per,tem,deriv;
	if(feedback!= 0)//if there is a perturbation then is processed, filtered and change range
	{
	per=filterFeedback->update(feedback);
	tem=changeRange(minFeedback,maxFeedback,-0.2,0.2,per);

	der[0]=der.at(1);
	der[1]=tem;
    deriv=der.at(1)-der.at(0);
	}
	else tem=0;//otherwise set perturbation to zero
    int oscillationFeedback=(maxFeedback+minFeedback)/2;//get the oscillation point of the perturbation
    bool look=true;



    //compute delay between perturbation and filtered one
    if(feedback > oscillationFeedback && counterDelayFeedback==0 && delayFeedback==false )
    {
    	delay[0]=step;
    	counterDelayFeedback++;
    	delayFeedback=true;
    }
    if(per > oscillationFeedback && counterDelayFiltered==0 && delayFiltered==false )
    {
       	delay[1]=step;
       	counterDelayFiltered++;
       	delayFiltered=true;
     }
    if(feedback< oscillationFeedback)
    	counterDelayFeedback=0;

    if(per<oscillationFeedback)
    	counterDelayFiltered=0;



    if(delayFeedback==true && delayFiltered==true )
    {
    	delayValue=delay.at(1)-delay.at(0);
    	delayFeedback=false;
    	delayFiltered=false;



    }


   //compute delay.... delay in delayValue

    double perturbation=0;



	if(step > 500 )//if(step > 1000 )
		perturbation=tem;

	cpg->update(perturbation);//apply perturbation to cpg




    std::vector<double> vec;
    if (feedback!=0)//check max of feedback, only if present
    {
    	vec=getMaxMinFeedback(feedback, maxFeedback,minFeedback);

    	maxFeedback=vec.at(0);
    	minFeedback=vec.at(1);
    }




    cpgOut1Derivative[0]=cpgOut1Derivative.at(1);
    cpgOut1Derivative[1]=cpg->getOut1();
    double out1DerivativeCpg =cpgOut1Derivative[1]-cpgOut1Derivative[0];



   //Dynamic register. IF for test purposed want to show that the CPG increases its frequency after switiching
   //from reflexive-based to CPG-based controller, comment next 2 lines and uncomment double test= cpg.....
   //this way there is not dynamic register

     if(dynRegister->getSize() != delayValue && step > 400 && feedback!=0 && delayValue > 0 && delayValue < dynRegister->getSize()+2)//change size of register according to delay in feedback
    	 dynRegister->changeSize(delayValue+2);//+2

     double test= dynRegister->update(cpg->getOut1());//generate cpg1 output

    //double test=cpg->getOut1();
   // std::cout << "step1 " << delay.at(1) << "step2 " << delay.at(0) << "Delay  " << delayValue << "   ShiftRegisterDimention  " << dynRegister->getSize() << std::endl;


    temp=generateCpgHipsMultiple(test, out1DerivativeCpg, 0, amplitudeMotor0);//CHANGE HERE TEST


    output.push_back(temp.at(0));//leftHip  0
    output.push_back(temp.at(1));//rightHip 1

    signal0Derivative[0]=signal0Derivative.at(1);
    signal0Derivative[1]=temp.at(0);//leftHip
    double derivativeLeftHip= signal0Derivative[1] - signal0Derivative[0];

    signal2Derivative[0]=signal2Derivative.at(1);
    signal2Derivative[1]=temp.at(1);//leftHip
    double derivativeRightHip= signal2Derivative[1] - signal2Derivative[0];





    if (motor0>-0.01 && motor0<0.01)
		{
		 maxHip=0;

		}
    if(motor1>-0.1 && motor1<0.1)
    	maxKnee=0;

    maxHip=getAmplitudeHips(motor0,maxHip);
    maxKnee=getAmplitudeHips(motor1,maxKnee);

    maxDer[0]=maxDer.at(1);
    maxDer[1]=maxHip;

    double maxDerValue=maxDer.at(1)-maxDer.at(0);

    maxKneeDer[0]=maxKneeDer.at(1);
    maxKneeDer[1]=maxKnee;


    double maxKneeDerValue=maxKneeDer.at(1)-maxKneeDer.at(0);

    if(maxDerValue > -0.1 && maxDerValue<0.1 && maxHip>1)
    {
    	amplitudeMotor0=maxHip;

    }

    if(maxKneeDerValue > -0.1 && maxKneeDerValue<0.1 && maxKnee>1.5 && maxKnee < 2)
    {
       	amplitudeMotor2=maxKnee;
    }





    cpgPlot << step << " " << perturbation <<" " <<  maxKnee<<" "<<motor1<< " "<< amplitudeMotor0<< std::endl;


    //std::cout << "ampl Hip" << amplitudeMotor0 << " " <<"ampl knee" << " "<< amplitudeMotor2 << std::endl;
    temp=generateCpgKneeMultiple(test, out1DerivativeCpg, 0, amplitudeMotor0); //CHANGE HERE TEST

    output.push_back(temp.at(0));//leftknee  2
    output.push_back(temp.at(1));//rightKnee 3


    cpgSignal0=output.at(0);
    cpgSignal1=output.at(1);
    cpgSignal2=output.at(2);
    cpgSignal3=output.at(3);


    return output;
}



int DynamicCpg::getEnable()
{
	return enable;
}


void DynamicCpg::setEnable(bool externalCondition,double motor0, double leftFoot, double rightFoot)
{
	if(externalCondition == false)
	{
		enable=0;
		counter=0;
	}

	if (externalCondition == true && counter==0 && getAbsol(cpgSignal0,motor0) < 0.07 && leftFoot*rightFoot==0 /*&& motor0 > 0.3*/)// && checkSignal->goodPhase()==true )
	{
		enable=1;
		counter++;
	}
}


std::vector<double> DynamicCpg::getMaxMinFeedback(double feedback, double max, double min)
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

std::vector<double> DynamicCpg::generateCpgKneeMultiple(double signal, double derivative,double oscillation, double value)
{

double left,right;
		std::vector<double> result;

		if(signal > oscillation)
		{
			right=value;
			if(derivative >=0)
				left=-value;
			else //else left=1.77;
				left = value;

		}

		if(signal <= oscillation)
		{
			left=value;
			if(derivative >=0)
				right=value;
			else right=-value;
		}

		result.push_back(left);
		result.push_back(right);

		return result;

}

std::vector<double> DynamicCpg::generateCpgHipsMultiple(double signal, double derivative,double oscillation, double value)
{
	std::vector<double> result;
		double right, left;
		if (signal> oscillation)
		  {

			if (derivative >= 0)
			{
				left=value;
				right=-value;


			}

			else
			{
				left=0;
				right=0;
			}

		  }

		  if (signal <= oscillation)
		  {

			  if (derivative < 0)
			  {
				left=-value;
				right=value;
			  }

			  else
			  {
				left=0;
				right=0;

			  }

		  }

		  result.push_back(left);
		  result.push_back(right);

		  return result;


}




DynamicCpg::~DynamicCpg() {
	// TODO Auto-generated destructor stub
}

