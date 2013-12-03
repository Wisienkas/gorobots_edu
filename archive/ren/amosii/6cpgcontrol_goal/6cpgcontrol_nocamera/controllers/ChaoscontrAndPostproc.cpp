/*
 * ChaoscontrAndPostProc.cpp
 *
 *  Created on: Aug 15, 2011
 *      Author: Ren Guanjiao
 */
 
 #include "ChaoscontrAndPostproc.h"

//This function is to initialize parameters
ChaoscontrAndPostproc::ChaoscontrAndPostproc(int p1 = 4,int p2 = 4,int p3 = 4,int p4 = 4,int p5 = 4,int p6 = 4)
{
	//difference values
	diff_n1_=0;
	diff_n2_=0;
	diffi_ = 0;
	diff_n1_cli=0;
	diff_n2_cli=0;
	diffi_cli = 0;
	diff_n1_cli1=0;
	diff_n2_cli1=0;
	diffi_cli1 = 0;
	diff_n1_cli2=0;
	diff_n2_cli2=0;
	diffi_cli2 = 0;
	diff_n1_cli3=0;
	diff_n2_cli3=0;
	diffi_cli3 = 0;
	diff_n1_cli4=0;
	diff_n2_cli4=0;
	diffi_cli4 = 0;
	
	//control inputs
	input1_ = 0;
	input2_ = 0;
	input1_cli = 0;
	input2_cli = 0;
	input1_cli1= 0;
	input2_cli1= 0;
	input1_cli2= 0;
	input2_cli2= 0;
	input1_cli3= 0;
	input2_cli3= 0;
	input1_cli4= 0;
	input2_cli4= 0;

	//----------------Global parameters-----------------------------
	//two neuron connection weights*****
	w11_ = -22.;	
	w12_ = 5.9;
	w21_ = -6.6 ;
	//two neuron bias terms*****
	theta1_ = -3.4;
	theta2_ = 3.8;

	//arrays for two neuron network output
	data_ = new data_t[PERIODMAX+1]; //(data_t*) calloc (periodmax_+1,sizeof(data_t));
	data_[0].o1 = 0;
	data_[0].o2 = 0;
	data_cli = new data_t[PERIODMAX+1];
	data_cli[0].o1 = 0;
	data_cli[0].o2 = 0;
	data_cli1 = new data_t[PERIODMAX+1];
	data_cli1[0].o1 = 0;
	data_cli1[0].o2 = 0;
	data_cli2 = new data_t[PERIODMAX+1];
	data_cli2[0].o1 = 0;
	data_cli2[0].o2 = 0;
	data_cli3 = new data_t[PERIODMAX+1];
	data_cli3[0].o1 = 0;
	data_cli3[0].o2 = 0;
	data_cli4 = new data_t[PERIODMAX+1];
	data_cli4[0].o1 = 0;
	data_cli4[0].o2 = 0;

	//learning rate
	lr_ = 0.05;
	lr_cli = 0.05;
	lr_cli1 = 0.05;
	lr_cli2 = 0.05;
	lr_cli3 = 0.05;
	lr_cli4 = 0.05;
		
	//weight of the hystesis
	thetahys_[0] = -0.5;
	thetahys_[1] = -0.6;
	
	//set frequency
	Freq = 2;
	
	//hysteresis inputs
	input_hys1 = input_hys2 = 0.0;
	input_hys1_cli = input_hys2_cli = 0.0;
	input_hys1_cli1 = input_hys2_cli1 = 0.0;
	input_hys1_cli2 = input_hys2_cli2 = 0.0;
	input_hys1_cli3 = input_hys2_cli3 = 0.0;
	input_hys1_cli4 = input_hys2_cli4 = 0.0;

	//hysteresis weight
	whys = 1.1;
	
	//initialize hysteresis
	ahys1 = ahys_old1 = 0;
	ahys2 = ahys_old2 = 0; 
	ahys1_cli = ahys_old1_cli = 0;
	ahys2_cli = ahys_old2_cli = 0; 
	ahys1_cli1 = ahys_old1_cli1 = 0;
	ahys2_cli1 = ahys_old2_cli1 = 0;
	ahys1_cli2 = ahys_old1_cli2 = 0;
	ahys2_cli2 = ahys_old2_cli2 = 0; 
	ahys1_cli3 = ahys_old1_cli3 = 0;
	ahys2_cli3 = ahys_old2_cli3 = 0; 
	ahys1_cli4 = ahys_old1_cli4 = 0;
	ahys2_cli4 = ahys_old2_cli4 = 0;

	//initialize hysteresis output
	output1 = 0;
	output2 = 0;
	output1_cli = 0;
	output2_cli = 0;
	output1_cli1 = 0;
	output2_cli1 = 0;
	output1_cli2 = 0;
	output2_cli2 = 0;
	output1_cli3 = 0;
	output2_cli3 = 0;
	output1_cli4 = 0;
	output2_cli4 = 0;
	
	//set inverse
	inverse_ = 1;

	//initialize period
	setPeriod(p1);
	setPeriod_cli(p2);
	setPeriod_cli1(p3);
	setPeriod_cli2(p4);
	setPeriod_cli3(p5);
	setPeriod_cli4(p6);
	
	//interconnection between differet CPGs
	alpha_syn = 0;
	memory = 0;
	synflag = false;
		
	alpha_syn1 = 0;
	memory1 = 0;
	synflag1 = false;
	
	alpha_syn2 = 0;
	memory2 = 0;
	synflag2 = false;
	
	alpha_syn3 = 0;
	memory3 = 0;
	synflag3 = false;
	
	alpha_syn4 = 0;
	memory4 = 0;
	synflag4 = false;
	
	fout.open("o1.txt");
	
}


ChaoscontrAndPostproc::~ChaoscontrAndPostproc()
{
	fout.close();
}


//This function is to generate the ChaosControl CPG Signal
void ChaoscontrAndPostproc::ChaosControl()
{
	step();
	step_cli();
	step_cli1();
	step_cli2();
	step_cli3();
	step_cli4();
	fout<<o1_<<"    "<<o1_cli<<endl;
	PostProcessing();
	PostProcessing_cli();
	PostProcessing_cli1();
	PostProcessing_cli2();
	PostProcessing_cli3();
	PostProcessing_cli4();
	//std::cout<<"period_cli="<<period_cli<<"  "<<"period_cli1="<<period_cli1<<"  "<<alpha_syn<<"  "<<alpha_syn1<<endl;
	//std::cout<<"normalize_cli="<<normalize_cli<<"  "<<"normalize_cli1="<<normalize_cli1<<endl;
}

//------------------------------------------------------------------------------------
//--------------------------------     Master    -------------------------------------
//------------------------------------------------------------------------------------
void ChaoscontrAndPostproc::step()
{
	double d1,d2;
	d1 = d2 = 0;
	if( n_ == period_ )
	{
		n_ = 0;
		diff_n1_ = data_[period_].o1-data_[0].o1; //Output neuron1
		diff_n2_ = data_[period_].o2-data_[0].o2; //Output neuron2
		diffi_ = diff_n1_ * diff_n1_ + diff_n2_ * diff_n2_;

		if(learning)
		{
			d1 = cl_ * diff_n1_ - diff_n1_;
			d2 = cl_ * diff_n2_ - diff_n2_;
            input1_ = w11_ * d1 + w12_ * d2;
            input2_ = w21_ * d1;
		}

		if(learning)	
			cl_ += ( (diffi_ * lr_) / period_ );
        else			
			cl_ -= (0.2 * cl_);

        if(!learning)
        {
			if ( cl_ < 1e-6 )	learning = true;
		}
		
		oscistep();//calculate the output
		
		data_[0].o1 = o1_;//a buffer for computering
	  	data_[0].o2 = o2_;	
	}
	else
	{
		n_++;
		
		oscistep();//calculate the output

		data_[n_].o1 = o1_;
	    data_[n_].o2 = o2_;
	}
}

// calculate output values
void ChaoscontrAndPostproc::oscistep()
{
	double activityO1 = w11_*o1_ + w12_*o2_ + theta1_ + input1_;
	double activityO2 = w21_*o1_ +            theta2_ + input2_;

	o1_ = sigmoid_(activityO1);
	o2_ = sigmoid_(activityO2);

	//After calculating the output, inputs must be reseted to zero
	input1_ = input2_ = 0;
}

//This function is to post-process the signals of the CPG generated by Chaos Control part 
void ChaoscontrAndPostproc::PostProcessing()
{
	if( 0 == ( ++percount % (normalize_) ) )  //time window funciton
	{
		input_hys1 = FAC * ( data_[n_].o1+ thetahys_[0] );
		input_hys2 = FAC * ( data_[n_].o2+ thetahys_[1] );
	}
	else
	{
		input_hys1 = 0;
        input_hys2 = 0;
	}
	
	//----------------------------------------------------------------
	//-----------------This is for the first neuron-------------------
	//----------------------------------------------------------------
	ahys_old1 = ahys1;
    ahys1 = tanh_(whys * ahys1 + input_hys1);    //hysteresis unit
	
	//signal integrator
	if( fabs(ahys1 - ahys_old1 ) > EPSILON )
	{
		if( ahys1 > 0 )	output1 = -inverse_;
		else		output1 = inverse_;
	}

	if( (inverse_ * ahys1) > 0 )
	{
		output1 += slopeUP_;
	}
	else
	{
		output1 += slopeDOWN_;
	}

	//fix it -1 to 1
	if( output1 > 1. )		
	{
		output1 = 1.;
	}
	else if( output1 < -1. )
	{
		output1 = -1.;
	}

	//----------------------------------------------------------------
	//---------------This is for the second neuron--------------------
	//----------------------------------------------------------------
	ahys_old2 = ahys2;
    ahys2 = tanh_(whys * ahys2 + input_hys2);    //hysteresis unit
	
	//signal integrator
	if( fabs(ahys2 - ahys_old2 ) > EPSILON )
	{
		if( ahys2 > 0 )	output2 = inverse_;
		else		output2 = -inverse_;
	}

	if( (-inverse_) * ahys2 > 0 )
	{
		output2 += slopeUP_;
	}
	else
	{
		output2 += slopeDOWN_;
	}

	//fix it -1 to 1
	if( output2 > 1. )		output2 = 1.;
	else if( output2 < -1. )	output2 = -1.;
	
}

//This function is to reset the parameters of ChaosControl CPG signal
void ChaoscontrAndPostproc::reset()
{
	percount = 0;
	cl_ =0;
	n_=0;
	data_[0].o1 = o1_ = 0;
	data_[0].o2 = o2_ = 0;
	learning = true;
	normalize_ =  Freq * period_ +1;
	slopeUP_ = 2./normalize_;
	slopeDOWN_ = 2./((1-period_)*normalize_);

}
//------------------------------------------------------------------------------------
//--------------------------------   Master  end -------------------------------------
//------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------
//--------------------------------   Clinet0     -------------------------------------
//------------------------------------------------------------------------------------
void ChaoscontrAndPostproc::step_cli()
{
	double d3,d4;
	d3 = d4 = 0;
	
	//1------------Reset to Period 1 when memory reach threshold!!!!
	//intial memory = 10000
	if( (deri_sigmoid_steep(memory)>0) && synflag)
	{
		setPeriod_cli(1);
		memory++;
		synflag = false;
	}
			
	if( n_cli == period_cli )
	{
		n_cli = 0;
		diff_n1_cli = data_cli[period_cli].o1-data_cli[0].o1; //Output neuron1
		diff_n2_cli = data_cli[period_cli].o2-data_cli[0].o2; //Output neuron2
		diffi_cli = diff_n1_cli * diff_n1_cli + diff_n2_cli * diff_n2_cli;
		
		//--------------8Period detection mechanism--------------------
		if ( rand()/((double)RAND_MAX) < diffi_cli )
			period_cli +=  (int)((rand()%3)-1)*(1-sigmoid_steep(memory)) ;
			//period_cli += (1-sigmoid_steep(memory));
		if(period_cli < 1)
			period_cli = 1;

		if(learning_cli)
		{
			d3 = cl_cli * diff_n1_cli - diff_n1_cli;
			d4 = cl_cli * diff_n2_cli - diff_n2_cli;
            input1_cli = w11_ * d3 + w12_ * d4;
            input2_cli = w21_ * d3;
		}

		if(learning_cli)	
			cl_cli += ( (diffi_cli * lr_cli) / period_cli );
        else			
			cl_cli -= (0.2 * cl_cli);

        if(!learning_cli)
        {
			if ( cl_cli < 1e-6 )	learning_cli = true;
		}
		
		//when we want to make the client synchronize to master, 
		//we have to firstly put the input to 0, to make it chaos 
		input1_cli *= sigmoid_steep(memory);
		input2_cli *= sigmoid_steep(memory);
		
		oscistep_cli();//calculate the output
		
		data_cli[0].o1 = o1_cli;//a buffer for computering
	  	data_cli[0].o2 = o2_cli;	
	}
	else
	{
		n_cli++;
		
		oscistep_cli();//calculate the output

		data_cli[n_cli].o1 = o1_cli;
	    data_cli[n_cli].o2 = o2_cli;
	}
}

// calculate output values
void ChaoscontrAndPostproc::oscistep_cli()
{
	double activityO1 = w11_*o1_cli + w12_*o2_cli + theta1_ + input1_cli;
	double activityO2 = w21_*o1_cli +               theta2_ + input2_cli;

	//	memory = 0; 		// alpha_syn = 0 = NOT SYN
	//	memory = 10000;   	// alpha_syn = 0.5
	//	memory > 10000;   	// alpha_syn = 1 = SYN
	alpha_syn = (1-sigmoid_steep(memory));
	o1_cli = sigmoid_(activityO1) + alpha_syn*(o1_- sigmoid_(activityO1));
	o2_cli = sigmoid_(activityO2);
		
	//After calculating the output, inputs must be reseted to zero
	input1_cli = input2_cli = 0;
}

//This function is to post-process the signals of the CPG generated by Chaos Control part 
void ChaoscontrAndPostproc::PostProcessing_cli()
{
	//recalculate the normalize and slope when the CPG synchronize to the master
	if(alpha_syn)
	{
		normalize_cli =  Freq * period_ +1;
		slopeUP_cli = 2./normalize_cli;
		slopeDOWN_cli = 2./((1-period_)*normalize_cli);
	}
	else
	{
		normalize_cli =  Freq * period_cli +1;
		slopeUP_cli = 2./normalize_cli;
		slopeDOWN_cli = 2./((1-period_cli)*normalize_cli);
	}
	
	if( 0 == ( ++percount_cli % (normalize_cli) ) )  //time window funciton
	{
		input_hys1_cli = FAC * ( data_cli[n_cli].o1+ thetahys_[0] );
		input_hys2_cli = FAC * ( data_cli[n_cli].o2+ thetahys_[1] );
	}
	else
	{
		input_hys1_cli = 0;
        input_hys2_cli = 0;
	}
	
	//----------------------------------------------------------------
	//-----------------This is for the first neuron-------------------
	//----------------------------------------------------------------
	ahys_old1_cli = ahys1_cli;
    ahys1_cli = tanh_(whys * ahys1_cli + input_hys1_cli);    //hysteresis unit
	
	//signal integrator
	if( fabs(ahys1_cli - ahys_old1_cli ) > EPSILON )
	{
		if( ahys1_cli > 0 )	output1_cli = -inverse_;
		else		output1_cli = inverse_;
	}

	if( (inverse_ * ahys1_cli) > 0 )
	{
		output1_cli += slopeUP_cli;
	}
	else
	{
		output1_cli += slopeDOWN_cli;
	}

	//fix it -1 to 1
	if( output1_cli > 1. )		
	{
		output1_cli = 1.;
	}
	else if( output1_cli < -1. )
	{
		output1_cli = -1.;
	}

	//----------------------------------------------------------------
	//---------------This is for the second neuron--------------------
	//----------------------------------------------------------------
	ahys_old2_cli = ahys2_cli;
    ahys2_cli = tanh_(whys * ahys2_cli + input_hys2_cli);    //hysteresis unit
	
	//signal integrator
	if( fabs(ahys2_cli - ahys_old2_cli ) > EPSILON )
	{
		if( ahys2_cli > 0 )	output2_cli = inverse_;
		else		output2_cli = -inverse_;
	}

	if( (-inverse_) * ahys2_cli > 0 )
	{
		output2_cli += slopeUP_cli;
	}
	else
	{
		output2_cli += slopeDOWN_cli;
	}

	//fix it -1 to 1
	if( output2_cli > 1. )		output2_cli = 1.;
	else if( output2_cli < -1. )	output2_cli = -1.;
}

//This function is to reset the parameters of ChaosControl CPG signal
void ChaoscontrAndPostproc::reset_cli()
{
	percount_cli = 0;
	cl_cli =0;
	n_cli=0;
	data_cli[0].o1 = o1_cli = 0;
	data_cli[0].o2 = o2_cli = 0;
	learning_cli = true;
	normalize_cli =  Freq * period_cli +1;
	slopeUP_cli = 2./normalize_cli;
	slopeDOWN_cli = 2./((1-period_cli)*normalize_cli);
}
//------------------------------------------------------------------------------------
//--------------------------------   Clinet0 end -------------------------------------
//------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------
//--------------------------------   Clinet1     -------------------------------------
//------------------------------------------------------------------------------------
void ChaoscontrAndPostproc::step_cli1()
{
	double d5,d6;
	d5 = d6 = 0;
	
	//1------------Reset to Period 1 when memory reach threshold!!!!
	if( (deri_sigmoid_steep(memory1)>0) && synflag1)
 	{
		setPeriod_cli1(1);
		memory1++;
		synflag1 = false;
	}
	
	if( n_cli1 == period_cli1 )
	{
		n_cli1 = 0;
		diff_n1_cli1 = data_cli1[period_cli1].o1-data_cli1[0].o1; //Output neuron1
		diff_n2_cli1 = data_cli1[period_cli1].o2-data_cli1[0].o2; //Output neuron2
		diffi_cli1 = diff_n1_cli1 * diff_n1_cli1 + diff_n2_cli1 * diff_n2_cli1;
		
		//--------------8Period detection mechanism--------------------
		if ( rand()/((double)RAND_MAX) < diffi_cli1 )
			period_cli1 += (int)((rand()%3)-1)*(1-sigmoid_steep(memory1));
			//period_cli1 += (1-sigmoid_steep(memory1));
		if(period_cli1 < 1)
			period_cli1 = 1;
			
		if(learning_cli1)
		{
			d5 = cl_cli1 * diff_n1_cli1 - diff_n1_cli1;
			d6 = cl_cli1 * diff_n2_cli1 - diff_n2_cli1;
            input1_cli1 = w11_ * d5 + w12_ * d6;
            input2_cli1 = w21_ * d5;
		}

		if(learning_cli1)	
			cl_cli1 += ( (diffi_cli1 * lr_cli1) / period_cli1 );
        else			
			cl_cli1 -= (0.2 * cl_cli1);

        if(!learning_cli1)
        {
			if ( cl_cli1 < 1e-6 )	learning_cli1 = true;
		}
		
		//when we want to make the client synchronize to master, 
		//we have to firstly put the input to 0, to make it chaos 
		input1_cli1 *= sigmoid_steep(memory1);
		input2_cli1 *= sigmoid_steep(memory1);
		
		oscistep_cli1();//calculate the output
		
		data_cli1[0].o1 = o1_cli1;//a buffer for computering
	  	data_cli1[0].o2 = o2_cli1;	
	}
	else
	{
		n_cli1++;
		
		oscistep_cli1();//calculate the output

		data_cli1[n_cli1].o1 = o1_cli1;
	    data_cli1[n_cli1].o2 = o2_cli1;
	}
}

// calculate output values
void ChaoscontrAndPostproc::oscistep_cli1()
{
	double activityO1 = w11_*o1_cli1 + w12_*o2_cli1 + theta1_ + input1_cli1;
	double activityO2 = w21_*o1_cli1 +                theta2_ + input2_cli1;

	//	memory = 0; 		// alpha_syn = 0 = NOT SYN
	//	memory = 10000;   	// alpha_syn = 0.5
	//	memory > 10000;   	// alpha_syn = 1 = SYN
	alpha_syn1 = (1-sigmoid_steep(memory1));
	o1_cli1 = sigmoid_(activityO1) + alpha_syn1*(o1_- sigmoid_(activityO1));
	o2_cli1 = sigmoid_(activityO2);
		
	//After calculating the output, inputs must be reseted to zero
	input1_cli1 = input2_cli1 = 0;
}

//This function is to post-process the signals of the CPG generated by Chaos Control part 
void ChaoscontrAndPostproc::PostProcessing_cli1()
{
	//recalculate the normalize and slope when the CPG synchronize to the master
	if(alpha_syn1)
	{
		normalize_cli1 =  Freq * period_ +1;
		slopeUP_cli1 = 2./normalize_cli1;
		slopeDOWN_cli1 = 2./((1-period_)*normalize_cli1);
	}
	else
	{
		normalize_cli1 =  Freq * period_cli1 +1;
		slopeUP_cli1 = 2./normalize_cli1;
		slopeDOWN_cli1 = 2./((1-period_cli1)*normalize_cli1);
	}
	
	if( 0 == ( ++percount_cli1 % (normalize_cli1) ) )  //time window funciton
	{
		input_hys1_cli1 = FAC * ( data_cli1[n_cli1].o1+ thetahys_[0] );
		input_hys2_cli1 = FAC * ( data_cli1[n_cli1].o2+ thetahys_[1] );
	}
	else
	{
		input_hys1_cli1 = 0;
        input_hys2_cli1 = 0;
	}
	
	//----------------------------------------------------------------
	//-----------------This is for the first neuron-------------------
	//----------------------------------------------------------------
	ahys_old1_cli1 = ahys1_cli1;
    ahys1_cli1 = tanh_(whys * ahys1_cli1 + input_hys1_cli1);    //hysteresis unit
	
	//signal integrator
	if( fabs(ahys1_cli1 - ahys_old1_cli1 ) > EPSILON )
	{
		if( ahys1_cli1 > 0 )	output1_cli1 = -inverse_;
		else					output1_cli1 = inverse_;
	}

	if( (inverse_ * ahys1_cli1) > 0 )
	{
		output1_cli1 += slopeUP_cli1;
	}
	else
	{
		output1_cli1 += slopeDOWN_cli1;
	}

	//fix it -1 to 1
	if( output1_cli1 > 1. )		
	{
		output1_cli1 = 1.;
	}
	else if( output1_cli1 < -1. )
	{
		output1_cli1 = -1.;
	}

	//----------------------------------------------------------------
	//---------------This is for the second neuron--------------------
	//----------------------------------------------------------------
	ahys_old2_cli1 = ahys2_cli1;
    ahys2_cli1 = tanh_(whys * ahys2_cli1 + input_hys2_cli1);    //hysteresis unit
	
	//signal integrator
	if( fabs(ahys2_cli1 - ahys_old2_cli1 ) > EPSILON )
	{
		if( ahys2_cli1 > 0 )	output2_cli1 = inverse_;
		else					output2_cli1 = -inverse_;
	}

	if( (-inverse_) * ahys2_cli1 > 0 )
	{
		output2_cli1 += slopeUP_cli1;
	}
	else
	{
		output2_cli1 += slopeDOWN_cli1;
	}

	//fix it -1 to 1
	if( output2_cli1 > 1. )		output2_cli1 = 1.;
	else if( output2_cli1 < -1. )	output2_cli1 = -1.;	
}

//This function is to reset the parameters of ChaosControl CPG signal
void ChaoscontrAndPostproc::reset_cli1()
{
	percount_cli1 = 0;
	cl_cli1 =0;
	n_cli1=0;
	data_cli1[0].o1 = o1_cli1 = 0;
	data_cli1[0].o2 = o2_cli1 = 0;
	learning_cli1 = true;
	normalize_cli1 =  Freq * period_cli1 +1;
	slopeUP_cli1 = 2./normalize_cli1;
	slopeDOWN_cli1 = 2./((1-period_cli1)*normalize_cli1);
}
//------------------------------------------------------------------------------------
//--------------------------------   Clinet1 end -------------------------------------
//------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------
//--------------------------------   Clinet2     -------------------------------------
//------------------------------------------------------------------------------------
void ChaoscontrAndPostproc::step_cli2()
{
	double d5,d6;
	d5 = d6 = 0;
	
	//1------------Reset to Period 1 when memory reach threshold!!!!
	if( (deri_sigmoid_steep(memory2)>0) && synflag2)
 	{
		setPeriod_cli2(1);
		memory2++;
		synflag2 = false;
	}
	
	if( n_cli2 == period_cli2 )
	{
		n_cli2 = 0;
		diff_n1_cli2 = data_cli2[period_cli2].o1-data_cli2[0].o1; //Output neuron1
		diff_n2_cli2 = data_cli2[period_cli2].o2-data_cli2[0].o2; //Output neuron2
		diffi_cli2 = diff_n1_cli2 * diff_n1_cli2 + diff_n2_cli2 * diff_n2_cli2;
		
		//--------------8Period detection mechanism--------------------
		if ( rand()/((double)RAND_MAX) < diffi_cli2 )
			period_cli2 += (int)((rand()%3)-1)*(1-sigmoid_steep(memory2));
			//period_cli2 += (1-sigmoid_steep(memory2));
		if(period_cli2 < 1)
			period_cli2 = 1;
			
		if(learning_cli2)
		{
			d5 = cl_cli2 * diff_n1_cli2 - diff_n1_cli2;
			d6 = cl_cli2 * diff_n2_cli2 - diff_n2_cli2;
            input1_cli2 = w11_ * d5 + w12_ * d6;
            input2_cli2 = w21_ * d5;
		}

		if(learning_cli2)	
			cl_cli2 += ( (diffi_cli2 * lr_cli2) / period_cli2 );
        else			
			cl_cli2 -= (0.2 * cl_cli2);

        if(!learning_cli2)
        {
			if ( cl_cli2 < 1e-6 )	learning_cli2 = true;
		}
		
		//when we want to make the client synchronize to master, 
		//we have to firstly put the input to 0, to make it chaos 
		input1_cli2 *= sigmoid_steep(memory2);
		input2_cli2 *= sigmoid_steep(memory2);
		
		oscistep_cli2();//calculate the output
		
		data_cli2[0].o1 = o1_cli2;//a buffer for computering
	  	data_cli2[0].o2 = o2_cli2;	
	}
	else
	{
		n_cli2++;
		
		oscistep_cli2();//calculate the output

		data_cli2[n_cli2].o1 = o1_cli2;
	    data_cli2[n_cli2].o2 = o2_cli2;
	}
}

// calculate output values
void ChaoscontrAndPostproc::oscistep_cli2()
{
	double activityO1 = w11_*o1_cli2 + w12_*o2_cli2 + theta1_ + input1_cli2;
	double activityO2 = w21_*o1_cli2 +                theta2_ + input2_cli2;

	//	memory = 0; 		// alpha_syn = 0 = NOT SYN
	//	memory = 10000;   	// alpha_syn = 0.5
	//	memory > 10000;   	// alpha_syn = 1 = SYN
	alpha_syn2 = (1-sigmoid_steep(memory2));
	o1_cli2 = sigmoid_(activityO1) + alpha_syn2*(o1_- sigmoid_(activityO1));
	o2_cli2 = sigmoid_(activityO2);
		
	//After calculating the output, inputs must be reseted to zero
	input1_cli2 = input2_cli2 = 0;
}

//This function is to post-process the signals of the CPG generated by Chaos Control part 
void ChaoscontrAndPostproc::PostProcessing_cli2()
{
	//recalculate the normalize and slope when the CPG synchronize to the master
	if(alpha_syn2)
	{
		normalize_cli2 =  Freq * period_ +1;
		slopeUP_cli2 = 2./normalize_cli2;
		slopeDOWN_cli2 = 2./((1-period_)*normalize_cli2);
	}
	else
	{
		normalize_cli2 =  Freq * period_cli2 +1;
		slopeUP_cli2 = 2./normalize_cli2;
		slopeDOWN_cli2 = 2./((1-period_cli2)*normalize_cli2);
	}
	
	if( 0 == ( ++percount_cli2 % (normalize_cli2) ) )  //time window funciton
	{
		input_hys1_cli2 = FAC * ( data_cli2[n_cli2].o1+ thetahys_[0] );
		input_hys2_cli2 = FAC * ( data_cli2[n_cli2].o2+ thetahys_[1] );
	}
	else
	{
		input_hys1_cli2 = 0;
        input_hys2_cli2 = 0;
	}
	
	//----------------------------------------------------------------
	//-----------------This is for the first neuron-------------------
	//----------------------------------------------------------------
	ahys_old1_cli2 = ahys1_cli2;
    ahys1_cli2 = tanh_(whys * ahys1_cli2 + input_hys1_cli2);    //hysteresis unit
	
	//signal integrator
	if( fabs(ahys1_cli2 - ahys_old1_cli2 ) > EPSILON )
	{
		if( ahys1_cli2 > 0 )	output1_cli2 = -inverse_;
		else					output1_cli2 = inverse_;
	}

	if( (inverse_ * ahys1_cli2) > 0 )
	{
		output1_cli2 += slopeUP_cli2;
	}
	else
	{
		output1_cli2 += slopeDOWN_cli2;
	}

	//fix it -1 to 1
	if( output1_cli2 > 1. )		
	{
		output1_cli2 = 1.;
	}
	else if( output1_cli2 < -1. )
	{
		output1_cli2 = -1.;
	}

	//----------------------------------------------------------------
	//---------------This is for the second neuron--------------------
	//----------------------------------------------------------------
	ahys_old2_cli2 = ahys2_cli2;
    ahys2_cli2 = tanh_(whys * ahys2_cli2 + input_hys2_cli2);    //hysteresis unit
	
	//signal integrator
	if( fabs(ahys2_cli2 - ahys_old2_cli2 ) > EPSILON )
	{
		if( ahys2_cli2 > 0 )	output2_cli2 = inverse_;
		else					output2_cli2 = -inverse_;
	}

	if( (-inverse_) * ahys2_cli2 > 0 )
	{
		output2_cli2 += slopeUP_cli2;
	}
	else
	{
		output2_cli2 += slopeDOWN_cli2;
	}

	//fix it -1 to 1
	if( output2_cli2 > 1. )		output2_cli2 = 1.;
	else if( output2_cli2 < -1. )	output2_cli2 = -1.;	
}

//This function is to reset the parameters of ChaosControl CPG signal
void ChaoscontrAndPostproc::reset_cli2()
{
	percount_cli2 = 0;
	cl_cli2 =0;
	n_cli2=0;
	data_cli2[0].o1 = o1_cli2 = 0;
	data_cli2[0].o2 = o2_cli2 = 0;
	learning_cli2 = true;
	normalize_cli2 =  Freq * period_cli2 +1;
	slopeUP_cli2 = 2./normalize_cli2;
	slopeDOWN_cli2 = 2./((1-period_cli2)*normalize_cli2);
}
//------------------------------------------------------------------------------------
//--------------------------------   Clinet2 end -------------------------------------
//------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------
//--------------------------------   Clinet3     -------------------------------------
//------------------------------------------------------------------------------------
void ChaoscontrAndPostproc::step_cli3()
{
	double d5,d6;
	d5 = d6 = 0;
	
	//1------------Reset to Period 1 when memory reach threshold!!!!
	if( (deri_sigmoid_steep(memory3)>0) && synflag3)
 	{
		setPeriod_cli3(1);
		memory3++;
		synflag3 = false;
	}
	
	if( n_cli3 == period_cli3 )
	{
		n_cli3 = 0;
		diff_n1_cli3 = data_cli3[period_cli3].o1-data_cli3[0].o1; //Output neuron1
		diff_n2_cli3 = data_cli3[period_cli3].o2-data_cli3[0].o2; //Output neuron2
		diffi_cli3 = diff_n1_cli3 * diff_n1_cli3 + diff_n2_cli3 * diff_n2_cli3;
		
		//--------------8Period detection mechanism--------------------
		if ( rand()/((double)RAND_MAX) < diffi_cli3 )
			period_cli3 += (int)((rand()%3)-1)*(1-sigmoid_steep(memory3));
			//period_cli2 += (1-sigmoid_steep(memory2));
		if(period_cli3 < 1)
			period_cli3 = 1;
			
		if(learning_cli3)
		{
			d5 = cl_cli3 * diff_n1_cli3 - diff_n1_cli3;
			d6 = cl_cli3 * diff_n2_cli3 - diff_n2_cli3;
            input1_cli3 = w11_ * d5 + w12_ * d6;
            input2_cli3 = w21_ * d5;
		}

		if(learning_cli3)	
			cl_cli3 += ( (diffi_cli3 * lr_cli3) / period_cli3 );
        else			
			cl_cli3 -= (0.2 * cl_cli3);

        if(!learning_cli3)
        {
			if ( cl_cli3 < 1e-6 )	learning_cli3 = true;
		}
		
		//when we want to make the client synchronize to master, 
		//we have to firstly put the input to 0, to make it chaos 
		input1_cli3 *= sigmoid_steep(memory3);
		input2_cli3 *= sigmoid_steep(memory3);
		
		oscistep_cli3();//calculate the output
		
		data_cli3[0].o1 = o1_cli3;//a buffer for computering
	  	data_cli3[0].o2 = o2_cli3;	
	}
	else
	{
		n_cli3++;
		
		oscistep_cli3();//calculate the output

		data_cli3[n_cli3].o1 = o1_cli3;
	    data_cli3[n_cli3].o2 = o2_cli3;
	}
}

// calculate output values
void ChaoscontrAndPostproc::oscistep_cli3()
{
	double activityO1 = w11_*o1_cli3 + w12_*o2_cli3 + theta1_ + input1_cli3;
	double activityO2 = w21_*o1_cli3 +                theta2_ + input2_cli3;

	//	memory = 0; 		// alpha_syn = 0 = NOT SYN
	//	memory = 10000;   	// alpha_syn = 0.5
	//	memory > 10000;   	// alpha_syn = 1 = SYN
	alpha_syn3 = (1-sigmoid_steep(memory3));
	o1_cli3 = sigmoid_(activityO1) + alpha_syn3*(o1_- sigmoid_(activityO1));
	o2_cli3 = sigmoid_(activityO2);
		
	//After calculating the output, inputs must be reseted to zero
	input1_cli3 = input2_cli3 = 0;
}

//This function is to post-process the signals of the CPG generated by Chaos Control part 
void ChaoscontrAndPostproc::PostProcessing_cli3()
{
	//recalculate the normalize and slope when the CPG synchronize to the master
	if(alpha_syn3)
	{
		normalize_cli3 =  Freq * period_ +1;
		slopeUP_cli3 = 2./normalize_cli3;
		slopeDOWN_cli3 = 2./((1-period_)*normalize_cli3);
	}
	else
	{
		normalize_cli3 =  Freq * period_cli3 +1;
		slopeUP_cli3 = 2./normalize_cli3;
		slopeDOWN_cli3 = 2./((1-period_cli3)*normalize_cli3);
	}
	
	if( 0 == ( ++percount_cli3 % (normalize_cli3) ) )  //time window funciton
	{
		input_hys1_cli3 = FAC * ( data_cli3[n_cli3].o1+ thetahys_[0] );
		input_hys2_cli3 = FAC * ( data_cli3[n_cli3].o2+ thetahys_[1] );
	}
	else
	{
		input_hys1_cli3 = 0;
        input_hys2_cli3 = 0;
	}
	
	//----------------------------------------------------------------
	//-----------------This is for the first neuron-------------------
	//----------------------------------------------------------------
	ahys_old1_cli3 = ahys1_cli3;
    ahys1_cli3 = tanh_(whys * ahys1_cli3 + input_hys1_cli3);    //hysteresis unit
	
	//signal integrator
	if( fabs(ahys1_cli3 - ahys_old1_cli3 ) > EPSILON )
	{
		if( ahys1_cli3 > 0 )	output1_cli3 = -inverse_;
		else					output1_cli3 = inverse_;
	}

	if( (inverse_ * ahys1_cli3) > 0 )
	{
		output1_cli3 += slopeUP_cli3;
	}
	else
	{
		output1_cli3 += slopeDOWN_cli3;
	}

	//fix it -1 to 1
	if( output1_cli3 > 1. )		
	{
		output1_cli3 = 1.;
	}
	else if( output1_cli3 < -1. )
	{
		output1_cli3 = -1.;
	}

	//----------------------------------------------------------------
	//---------------This is for the second neuron--------------------
	//----------------------------------------------------------------
	ahys_old2_cli3 = ahys2_cli3;
    ahys2_cli3 = tanh_(whys * ahys2_cli3 + input_hys2_cli3);    //hysteresis unit
	
	//signal integrator
	if( fabs(ahys2_cli3 - ahys_old2_cli3 ) > EPSILON )
	{
		if( ahys2_cli3 > 0 )	output2_cli3 = inverse_;
		else					output2_cli3 = -inverse_;
	}

	if( (-inverse_) * ahys2_cli3 > 0 )
	{
		output2_cli3 += slopeUP_cli3;
	}
	else
	{
		output2_cli3 += slopeDOWN_cli3;
	}

	//fix it -1 to 1
	if( output2_cli3 > 1. )		output2_cli3 = 1.;
	else if( output2_cli3 < -1. )	output2_cli3 = -1.;	
}

//This function is to reset the parameters of ChaosControl CPG signal
void ChaoscontrAndPostproc::reset_cli3()
{
	percount_cli3 = 0;
	cl_cli3 =0;
	n_cli3=0;
	data_cli3[0].o1 = o1_cli3 = 0;
	data_cli3[0].o2 = o2_cli3 = 0;
	learning_cli3 = true;
	normalize_cli3 =  Freq * period_cli3 +1;
	slopeUP_cli3 = 2./normalize_cli3;
	slopeDOWN_cli3 = 2./((1-period_cli3)*normalize_cli3);
}
//------------------------------------------------------------------------------------
//--------------------------------   Clinet3 end -------------------------------------
//------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------
//--------------------------------   Clinet4     -------------------------------------
//------------------------------------------------------------------------------------
void ChaoscontrAndPostproc::step_cli4()
{
	double d5,d6;
	d5 = d6 = 0;
	
	//1------------Reset to Period 1 when memory reach threshold!!!!
	if( (deri_sigmoid_steep(memory4)>0) && synflag4)
 	{
		setPeriod_cli4(1);
		memory4++;
		synflag4 = false;
	}
	
	if( n_cli4 == period_cli4 )
	{
		n_cli4 = 0;
		diff_n1_cli4 = data_cli4[period_cli4].o1-data_cli4[0].o1; //Output neuron1
		diff_n2_cli4 = data_cli4[period_cli4].o2-data_cli4[0].o2; //Output neuron2
		diffi_cli4 = diff_n1_cli4 * diff_n1_cli4 + diff_n2_cli4 * diff_n2_cli4;
		
		//--------------8Period detection mechanism--------------------
		if ( rand()/((double)RAND_MAX) < diffi_cli4 )
			period_cli4 += (int)((rand()%3)-1)*(1-sigmoid_steep(memory4));
			//period_cli4 += (1-sigmoid_steep(memory4));
		if(period_cli4 < 1)
			period_cli4 = 1;
			
		if(learning_cli4)
		{
			d5 = cl_cli4 * diff_n1_cli4 - diff_n1_cli4;
			d6 = cl_cli4 * diff_n2_cli4 - diff_n2_cli4;
            input1_cli4 = w11_ * d5 + w12_ * d6;
            input2_cli4 = w21_ * d5;
		}

		if(learning_cli4)	
			cl_cli4 += ( (diffi_cli4 * lr_cli4) / period_cli4 );
        else			
			cl_cli4 -= (0.2 * cl_cli4);

        if(!learning_cli4)
        {
			if ( cl_cli4 < 1e-6 )	learning_cli4 = true;
		}
		
		//when we want to make the client synchronize to master, 
		//we have to firstly put the input to 0, to make it chaos 
		input1_cli4 *= sigmoid_steep(memory4);
		input2_cli4 *= sigmoid_steep(memory4);
		
		oscistep_cli4();//calculate the output
		
		data_cli4[0].o1 = o1_cli4;//a buffer for computering
	  	data_cli4[0].o2 = o2_cli4;	
	}
	else
	{
		n_cli4++;
		
		oscistep_cli4();//calculate the output

		data_cli4[n_cli4].o1 = o1_cli4;
	    data_cli4[n_cli4].o2 = o2_cli4;
	}
}

// calculate output values
void ChaoscontrAndPostproc::oscistep_cli4()
{
	double activityO1 = w11_*o1_cli4 + w12_*o2_cli4 + theta1_ + input1_cli4;
	double activityO2 = w21_*o1_cli4 +                theta2_ + input2_cli4;

	//	memory = 0; 		// alpha_syn = 0 = NOT SYN
	//	memory = 10000;   	// alpha_syn = 0.5
	//	memory > 10000;   	// alpha_syn = 1 = SYN
	alpha_syn4 = (1-sigmoid_steep(memory4));
	o1_cli4 = sigmoid_(activityO1) + alpha_syn4*(o1_- sigmoid_(activityO1));
	o2_cli4 = sigmoid_(activityO2);
		
	//After calculating the output, inputs must be reseted to zero
	input1_cli4 = input2_cli4 = 0;
}

//This function is to post-process the signals of the CPG generated by Chaos Control part 
void ChaoscontrAndPostproc::PostProcessing_cli4()
{
	//recalculate the normalize and slope when the CPG synchronize to the master
	if(alpha_syn4)
	{
		normalize_cli4 =  Freq * period_ +1;
		slopeUP_cli4 = 2./normalize_cli4;
		slopeDOWN_cli4 = 2./((1-period_)*normalize_cli4);
	}
	else
	{
		normalize_cli4 =  Freq * period_cli4 +1;
		slopeUP_cli4 = 2./normalize_cli4;
		slopeDOWN_cli4 = 2./((1-period_cli4)*normalize_cli4);
	}
	
	if( 0 == ( ++percount_cli4 % (normalize_cli4) ) )  //time window funciton
	{
		input_hys1_cli4 = FAC * ( data_cli4[n_cli4].o1+ thetahys_[0] );
		input_hys2_cli4 = FAC * ( data_cli4[n_cli4].o2+ thetahys_[1] );
	}
	else
	{
		input_hys1_cli4 = 0;
        input_hys2_cli4 = 0;
	}
	
	//----------------------------------------------------------------
	//-----------------This is for the first neuron-------------------
	//----------------------------------------------------------------
	ahys_old1_cli4 = ahys1_cli4;
    ahys1_cli4 = tanh_(whys * ahys1_cli4 + input_hys1_cli4);    //hysteresis unit
	
	//signal integrator
	if( fabs(ahys1_cli4 - ahys_old1_cli4 ) > EPSILON )
	{
		if( ahys1_cli4 > 0 )	output1_cli4 = -inverse_;
		else					output1_cli4 = inverse_;
	}

	if( (inverse_ * ahys1_cli4) > 0 )
	{
		output1_cli4 += slopeUP_cli4;
	}
	else
	{
		output1_cli4 += slopeDOWN_cli4;
	}

	//fix it -1 to 1
	if( output1_cli4 > 1. )		
	{
		output1_cli4 = 1.;
	}
	else if( output1_cli4 < -1. )
	{
		output1_cli4 = -1.;
	}

	//----------------------------------------------------------------
	//---------------This is for the second neuron--------------------
	//----------------------------------------------------------------
	ahys_old2_cli4 = ahys2_cli4;
    ahys2_cli4 = tanh_(whys * ahys2_cli4 + input_hys2_cli4);    //hysteresis unit
	
	//signal integrator
	if( fabs(ahys2_cli4 - ahys_old2_cli4 ) > EPSILON )
	{
		if( ahys2_cli4 > 0 )	output2_cli4 = inverse_;
		else					output2_cli4 = -inverse_;
	}

	if( (-inverse_) * ahys2_cli4 > 0 )
	{
		output2_cli4 += slopeUP_cli4;
	}
	else
	{
		output2_cli4 += slopeDOWN_cli4;
	}

	//fix it -1 to 1
	if( output2_cli4 > 1. )		output2_cli4 = 1.;
	else if( output2_cli4 < -1. )	output2_cli4 = -1.;	
}

//This function is to reset the parameters of ChaosControl CPG signal
void ChaoscontrAndPostproc::reset_cli4()
{
	percount_cli4 = 0;
	cl_cli4 =0;
	n_cli4=0;
	data_cli4[0].o1 = o1_cli4 = 0;
	data_cli4[0].o2 = o2_cli4 = 0;
	learning_cli4 = true;
	normalize_cli4 =  Freq * period_cli4 +1;
	slopeUP_cli4 = 2./normalize_cli4;
	slopeDOWN_cli4 = 2./((1-period_cli4)*normalize_cli4);
}
//------------------------------------------------------------------------------------
//--------------------------------   Clinet4 end -------------------------------------
//------------------------------------------------------------------------------------

std::vector<double> ChaoscontrAndPostproc::step_nlm(const std::vector<double> in0)
{
	// just a silly example, do your neural learning stuff here and generate an output vector
	std::vector<double> output;
	return output;
};

