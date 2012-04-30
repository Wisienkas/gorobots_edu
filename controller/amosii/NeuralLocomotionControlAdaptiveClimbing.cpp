/*
 * NeuralLocomotionControlAdaptiveClimbing.cpp
 *
 *  Created on: May 2, 2011
 *      Author: poramate
 */

#include "NeuralLocomotionControlAdaptiveClimbing.h"


NeuralLocomotionControlAdaptiveClimbing::NeuralLocomotionControlAdaptiveClimbing(){

 		  //Save files
 		  outFilenlc1.open("Neurallocomotion.dat");

/*******************************************************************************
* Set vector size
*******************************************************************************/
 		  //---Set vector size----//

 		  //Input vector size
 		  input.resize(5);

 		  m.resize(19);

 		  //Using saved text

 		  m_r0_t.resize(8000);
 		  m_r1_t.resize(8000);
 		  m_r2_t.resize(8000);
 		  m_l0_t.resize(8000);
 		  m_l1_t.resize(8000);
 		  m_l2_t.resize(8000);


/*******************************************************************************
*  Initial parameters
*******************************************************************************/

	      //Test walking behavior with saved motor text
 	     initialized = false;
 	     ii = 0;

 	     m_r0_t_old = 0.0;
 	     m_r1_t_old = 0.0;
 	     m_r2_t_old = 0.0;
 	     m_l0_t_old = 0.0;
 	     m_l1_t_old = 0.0;
 	     m_l2_t_old = 0.0;

/*******************************************************************************
*  CONTROL OPTION!!!!
*******************************************************************************/


 	      //Testing controller from text (e.g. SOINN control as motor memory network)
 	     reading_text_testing = false;


 	  };


NeuralLocomotionControlAdaptiveClimbing::~NeuralLocomotionControlAdaptiveClimbing(){


		  //Save files
		  outFilenlc1.close();

  	};

std::vector<double> NeuralLocomotionControlAdaptiveClimbing::step_nlc(const std::vector<double> in0, const std::vector<double> in1){//, bool Footinhibition){

	//Input to control locomotion
		input.at(0) = 0; //walking = 0, joint inhibition = 1
		input.at(1) = 1; //lateral = 0, no lateral motion = 1
		input.at(2) = 1; //lateral right = 0, lateral left = 1
		input.at(3) = -1; //turn left = 1,
		input.at(4) = -1; //turn right = 1,



/**/
/*******************************************************************************
*  MODULE please add your controller as a MODULE with Class Walking
*  put your contribution as a class and call the respective method here,
*  also provide informative comment
*******************************************************************************/


/*******************************************************************************
*  FINAL MOTOR OUTPUTS TO MOTOR NEURONS
*******************************************************************************/
		//------------------Reading signals from Text file
		if(reading_text_testing)
		{
			//Test the reading function
			char str[10];
			//std::string str;

			//Opens for reading the file
			//ifstream b_file ("ManyLegs_Trans_0.11_by_AMMC_gain=0.5_trainon0.02.txt");//( "Test1.txt" );
			//ifstream b_file ("ManyLegs_Tripod_0.18_by_AMMC_gain=0.5_trainon0.02.txt");//( "Test1.txt" );
			ifstream b_file ("ManyLegs_0.05_by_AMMC_gain=0.5_trainon0.02.txt");//( "Test1.txt" );

			//Reads one string from the file

			m_r0_t_old = m.at(TR0_m);
			m_r1_t_old = m.at(TR1_m);
			m_r2_t_old = m.at(TR2_m);
			m_l0_t_old = m.at(TL0_m);
			m_l1_t_old = m.at(TL1_m);
			m_l2_t_old = m.at(TL2_m);

			if(!initialized)
			{	i_text_loop = 0;
			while(b_file>> str) //time first column
			{

				// b_file>> str;
				m_r0_text = atof(str);//input1
				m_r0_t.at(i_text_loop) = m_r0_text;

				b_file>> str;
				m_r1_text = atof(str);//input2
				m_r1_t.at(i_text_loop) = m_r1_text;

				b_file>> str;
				m_r2_text = atof(str);//input3
				m_r2_t.at(i_text_loop) = m_r2_text;

				b_file>> str;
				m_l0_text = atof(str);//input4
				m_l0_t.at(i_text_loop) = m_l0_text;

				b_file>> str;
				m_l1_text = atof(str);//input5
				m_l1_t.at(i_text_loop) = m_l1_text;

				b_file>> str;
				m_l2_text = atof(str);//input6
				m_l2_t.at(i_text_loop) = m_l2_text;


				i_text_loop++;

//				std::cout<<"mR0 "<<m_r0_text<<"mR1 "<<m_r1_text<<"mR2 "<<m_r2_text<<"mL0 "<<m_l0_text<<"mL1 "<<m_l1_text<<"mL2 "<<m_l2_text <<" "<<i_text_loop<<"\n"<<endl;

			}
			initialized = true;
			}

			///Use motor signal from text

			if(ii<i_text_loop)
			{
				ii++;
			}
			else
			{
				ii=0;
			}
			m.at(TR0_m) = m_r0_t.at(ii);// m_reflex.at(TR0_m);
			m.at(TR1_m) = m_r1_t.at(ii);//m_reflex.at(TR1_m);
			m.at(TR2_m) = m_r2_t.at(ii);//m_reflex.at(TR2_m);

			m.at(TL0_m) = m_l0_t.at(ii);//m_reflex.at(TL0_m);
			m.at(TL1_m) = m_l1_t.at(ii);//m_reflex.at(TL1_m);
			m.at(TL2_m) = m_l2_t.at(ii);//m_reflex.at(TL2_m);

			double up = 0.6;

			if(m.at(TR0_m)>m_r0_t_old)
				m.at(CR0_m) = m.at(TR0_m)+0.5;
			else
				m.at(CR0_m) = up;

			if(m.at(TR1_m)>m_r1_t_old)
				m.at(CR1_m) = m.at(TR1_m)+1.5;
			else
				m.at(CR1_m) = up;

			if(m.at(TR2_m)>m_r2_t_old)
				m.at(CR2_m) = m.at(TR2_m)+1.5;
			else
				m.at(CR2_m) = up;

			if(m.at(TL0_m)>m_l0_t_old)
				m.at(CL0_m) = m.at(TL0_m)+1.5;
			else
				m.at(CL0_m) = up;

			if(m.at(TL1_m)>m_l1_t_old)
				m.at(CL1_m) = m.at(TL1_m)+1.5;
			else
				m.at(CL1_m) = up;

			if(m.at(TL2_m)>m_l2_t_old)
				m.at(CL2_m) = m.at(TL2_m)+1.5;
			else
				m.at(CL2_m) = up;

			m.at(FR0_m) = -1;//m_reflex.at(FR0_m);
			m.at(FR1_m) = -1;//m_reflex.at(FR1_m);
			m.at(FR2_m) = -1;//m_reflex.at(FR2_m);

			m.at(FL0_m) = -1;//m_reflex.at(FL0_m);
			m.at(FL1_m) = -1;//m_reflex.at(FL1_m);
			m.at(FL2_m) = -1;//m_reflex.at(FL2_m);

			m.at(BJ_m) =  0;//(m_reflex.at(TR0_m)-0.35)*5;//

			//------------------Reading signals from Text file
		}
		else // using normal control
		{
			tripod.Kine();

			m.at(TR0_m) = tripod.out.at(TR0_m);
			m.at(TR1_m) = tripod.out.at(TR1_m);
			m.at(TR2_m) = tripod.out.at(TR2_m);

			m.at(TL0_m) = tripod.out.at(TL0_m);
			m.at(TL1_m) = tripod.out.at(TL1_m);
			m.at(TL2_m) = tripod.out.at(TL2_m);

			m.at(CL0_m) = tripod.out.at(CL0_m);
			m.at(CL1_m) = tripod.out.at(CL1_m);
			m.at(CL2_m) = tripod.out.at(CL2_m);

			m.at(CR0_m) = tripod.out.at(CR0_m);
			m.at(CR1_m) = tripod.out.at(CR1_m);
			m.at(CR2_m) = tripod.out.at(CR2_m);

			m.at(FR0_m) = tripod.out.at(FR0_m);
			m.at(FR1_m) = tripod.out.at(FR1_m);
			m.at(FR2_m) = tripod.out.at(FR2_m);

			m.at(FL0_m) = tripod.out.at(FL0_m);
			m.at(FL1_m) = tripod.out.at(FL1_m);
			m.at(FL2_m) = tripod.out.at(FL2_m);

			m.at(BJ_m) =  0;
			outFilenlc1<<m.at(TR0_m)<<' '<<m.at(TR1_m)<<' '<<m.at(TR2_m)<<' '<<m.at(TL0_m)<<' '<<m.at(TL1_m)<<' '<<m.at(TL2_m)<<endl;
}
		global_count++;

		return m;

};



