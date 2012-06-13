/*
 * NeuralPreprocessingReflex.cpp
 *
 *  Created on: May 2, 2011
 *      Author: poramate
 */

#include "NeuralPreprocessingReflex.h"


///-------------------------------------------------------------------------------------------------------------------------

//1) Class for Neural preprocessing------------


NeuralPreprocessingReflex::NeuralPreprocessingReflex(){

		 //Save files
		 //outFilenpp1.open("Neuralpreprocessing.txt");

		 //---Set vector size----//
		 mappingsensor.resize(AMOSII_SENSOR_MAX);// defined in sensormotordefinition
		 sensor_activity.resize(AMOSII_SENSOR_MAX);
		 sensor_output.resize(AMOSII_SENSOR_MAX);
		 preprosensor.resize(AMOSII_SENSOR_MAX);

  		 //---Initialize your values

		 sensor_w_pfs_rfs = -5.0;
		 sensor_w_pfs_pfs = 0.0;//0.5;//1.0;//1.5;//2.0;

  	};

NeuralPreprocessingReflex::~NeuralPreprocessingReflex(){

		  //Save files
		  //outFilenpp1.close();

	 };

//1)  Step function of Neural preprocessing------------
std::vector<double> NeuralPreprocessingReflex::step_npp(const std::vector<double> in0){

	//----------KOH--------//

	//Define local parameters

	//1)****Prepro Foot sensors*********//
	double Amplifiy_factor_Foot = 2.5;
	int signal_inversion = -1;


	//1)****Prepro Foot sensors for searching reflexes********

	for(unsigned int i=R0_fs; i<(L2_fs+1);i++)
	{
		//Mapping foot sensor to +-1
		//preprosensor.at(i)=(((x.at(i)-0)/(1-0))*2.0-1.0)*Amplifiy_factor_Foot*signal_inversion; //amplify (e.g., 2.5) and invert signals (-1)
		mappingsensor.at(i)=(((in0.at(i)-0)/(1-0))*2.0-1.0);//*Amplifiy_factor_Foot*signal_inversion; //amplify (e.g., 2.5) and invert signals (-1)

//			   if (mappingsensor.at(i)>1)
//			  {
//				   mappingsensor.at(i)=1;
//			  }
//			  if (mappingsensor.at(i)<-1)
//			  {
//				  mappingsensor.at(i)=-1;
//			  }

			//Preprocessing
			sensor_activity.at(i) = mappingsensor.at(i)*sensor_w_pfs_rfs+sensor_output.at(i)*sensor_w_pfs_pfs;//*presyFL3+biasFL3+ac_OutPostprocessFL3*recurrentFL3;
	  		sensor_output.at(i) = tanh(sensor_activity.at(i));
	  		preprosensor.at(i) = sensor_output.at(i);

//			preprosensor.at(R0_fs);
//			preprosensor.at(R1_fs);
//			preprosensor.at(R2_fs);
//			preprosensor.at(L0_fs);
//			preprosensor.at(L1_fs);
//			preprosensor.at(L2_fs);

	}



	  // >> i/o operations here <<
	  //outFilenpp1<<in0.at(L0_fs)<<' '<<preprosensor.at(L0_fs)<<' '<<in0.at(L1_fs)<<' '<<preprosensor.at(L1_fs)
	  //		  <<' '<<in0.at(L2_fs)<<' '<<preprosensor.at(L2_fs)<<endl;

	preprosensor.at(G0x_s) = in0.at(G0x_s);
	preprosensor.at(G0y_s) = in0.at(G0y_s);
	preprosensor.at(G0z_s) = in0.at(G0z_s);


	//
	//---------------------------//



	return preprosensor;

};

///-------------------------------------------------------------------------------------------------------------------------




