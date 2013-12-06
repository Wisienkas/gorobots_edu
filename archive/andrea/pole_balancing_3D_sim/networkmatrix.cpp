/*  networkmatrix.cpp
 *
 *  Contains the ESNetwork class member function definitions
 *
 *  Created on: Mar 22, 2012
 *  modified on: May 23, 2012
 *  Author: Andrej Fillipow and Sakyasingha Dasgupta
 */
#include <fstream>
#include "networkmatrix.h"



    ESNetwork::ESNetwork(int a, int b, int c , bool param1, bool param2, double param3, bool param4)
    {
      enable_IP = param4;

      //parameters:
      throughputConnections = param1; //if this boolean is true, the input matrix is added to
      feedbackConnections = param2; //param2;//true;
      stepsRun = 0;
      double leak_rate = param3; // rate of leakage of neurons
      if (leak_rate ==0)
        leak = false;
      else
        leak = true;
      // enables or disables leaky integrated neurons

      //numbers of neurons used in the net
      // networkNeurons should be about 20-100 times bigger than inputs or outputs

       inputNeurons = a;
       outputNeurons = b;
       networkNeurons = c;

       //the actual matrices that hold the output of the neurons
        inputs = new matrix::Matrix(inputNeurons,1);
        outputs = new matrix::Matrix(outputNeurons,1);
        intermediates = new matrix::Matrix(networkNeurons,1);
        leak_mat = new matrix::Matrix(networkNeurons,networkNeurons);

        //Initializing leak matrix
        for (int i =0;i<networkNeurons;i++)
            	for (int j=0;j<networkNeurons;j++)
            			if (i==j) leak_mat->val(i,j) = 1-leak_rate;
            			else leak_mat->val(i,j)=0.0;

       //**********Weight matrices for the ESN framework****************//

        //input weight matrix
        startweights = new matrix::Matrix(networkNeurons, inputNeurons);

        //Reservoir weight matrix. Matrix is bigger if the output feeds back
        int temp =  networkNeurons + (feedbackConnections ?  outputNeurons : 0);
        innerweights = new matrix::Matrix(temp, temp);

        //Output weight matrix. Matrix is bigger, if the output feeds back
        //temp = (throughputConnections?  inputNeurons : 0)+ networkNeurons;
        endweights   = new matrix::Matrix(outputNeurons, networkNeurons/*temp*/);

        noise = new matrix::Matrix(networkNeurons,1);

       //Matrix for online recursive learning
        onlineLearningAutocorrelation = new  matrix::Matrix(networkNeurons, networkNeurons);

        unsigned int i,j = 0; // initialise the function to a positive number

        /*for(i = 0; i < onlineLearningAutocorrelation->getM(); j++)
        {
          if (j == onlineLearningAutocorrelation->getN())
            {
              j = 0;
              i++;
              if (i == onlineLearningAutocorrelation->getM()) {break;}
            }
          onlineLearningAutocorrelation->val(i,j)=(i==j)?0.1:0; //watch out here
        }*/

        for(i = 0; i < networkNeurons; i++)
        	for(j=0;j< networkNeurons; j++)
        		{
        		    if(i==j)
        		    	onlineLearningAutocorrelation->val(i,j)= pow(10,5);
        		    else
        		    	onlineLearningAutocorrelation->val(i,j)= 0.0;
        		}


       errors = new float[outputNeurons];
       oldErrors = new float[outputNeurons];
       oldOutputs = new float[outputNeurons];
       oldIntermediates = new float[networkNeurons];
       history = new matrix::Matrix();
       *history = *endweights ;

       // for evaluation purposes:
       EvaluationCollectionStep = 0;
  //     outputsCollection = NULL;

       for (int i = 0; i< networkNeurons;i++)
       {
         oldIntermediates[i] =  0;
       }
       for (int i = 0; i< outputNeurons;i++)
       {
         errors[i] = oldErrors[i] = 0;
         oldOutputs[i] = 0;
       }

       for (int i = 0; i< 100000;i++)
              {
                store_para_a[i] =  0.0;
                store_para_b[i] = 0.0;
              }

       for (int i = 0; i< 10000;i++)
       outputsCollection[i]=0.0;

    // All array initializations complete
       std::cout<<"Arrays initialised" << "\n";
       out.open("output_matrix");


    }

    ESNetwork::~ESNetwork()
    {
      delete inputs;
      delete outputs;
      delete intermediates;
      delete startweights;
      delete innerweights;
      delete endweights;
      delete onlineLearningAutocorrelation;
      delete errors;
      delete oldErrors;
      delete oldOutputs;
      delete oldIntermediates;
      delete history;
      delete outputsCollection;
      out.close();
    }


    //Generate connection weights
    void ESNetwork::generate_random_weights(int sparsity, float spectral_radius) //sparsity from 1-100, density 0-1, all connections random
    {
      std::cout << "generating full-random weighting. \n \n";
      srand(time(NULL));

      unsigned int i,j = 0;

      for(i = 0; i < startweights->getM(); i++)
       for (j=0; j< startweights->getN();j++)
       {
    	   //if ((rand()%100) >= sparsity)
    	   {startweights->val(i,j)= ESNetwork::uniform(-0.5,0.5);} //((double)(rand()%100)/100);}}
    	//   else
    		//   startweights->val(i,j)= 0.000;
       }

      //initialise the inner weights
      j = 0;
      for(i = 0; i < innerweights->getM(); i++)
      for(j = 0; j < innerweights->getM(); j++)
      {
        if ((rand()%100) >= sparsity) { innerweights->val(i,j)= ESNetwork::uniform(-1,1);}//((double)(rand()%100)/100);}
        else
        	innerweights->val(i,j)= 0.000;
      }



      //finally initialise the output weights
      j = 0;
      for(i = 0; i < endweights->getM(); i++)
    	  for(j = 0; j < endweights->getM(); j++)
     //   if ((rand()%100) >= sparsity)
        { endweights->val(i,j)=0.000;/*((double)(rand()%100)/300);*/}


      //initialize noise matrix
      for(i = 0; i < networkNeurons; i++)
          	  for(j = 0; j < 1; j++)
            noise->val(i,j) = ESNetwork::uniform(-0.01,0.01);

      ESNetwork::normalizeInnerWeights(spectral_radius);

      std::cout <<" finished \n";
    }

    void ESNetwork::generate_neighbour_weights(int sparsity, float spectral_radius, int nNextNeighbours) //like above, only connections to the nNextNeighbours neurons are made
    {/*
      std::cout << "generating neighbored weighting. \n \n";
      srand(time(NULL));
      unsigned int i= 0;
      int j = 0;
      for(i = 0; i < startweights->getM(); j++)
      {
        if (j == startweights->getN())
                  {
                    j = 0;
                    i++;
                    if (i == startweights->getM()) {break;}
                  }
        if ((rand()%100) >= sparsity) { startweights->val(i,j)=((double)(rand()%100)/100);}
      }

      //initialise the inner weights
      j = 0;
      for(i = 0; i < innerweights->getM(); j++)
      {
        if (j == i+nNextNeighbours || j>= innerweights->getN())
          {
            j = i-nNextNeighbours;
            if (j < 0) j = 0;
            i++;
            if (i == innerweights->getM()) {break;}
          }
        if ((rand()%100) >= sparsity) { innerweights->val(i,j)=((double)(rand()%100)/100);}

      }
      std::cout<<".";

      //finally initialise the output weights
      j = 0;
      for(i = 0; i < endweights->getM(); j++)
      {
        if (j == endweights->getN())
        {
          j = 0;
          i++;
          if (i == endweights->getM()) {break;}
        }
        if ((rand()%100) >= sparsity) { endweights->val(i,j)=((double)(rand()%100)/100);}
      }

      ESNetwork::normalizeInnerWeights(spectral_radius);
      std::cout<<" finished \n"; */
    }

    void ESNetwork::setInput(float *Input, int size)// sets the input neurons, called by trainOutputs() or manually from main.cpp
    {
      for (int i = 0; i < size; i++)
      {
        inputs->val(i,0) = Input[i];
      //  printMatrix(inputs);
      }
    }

    void ESNetwork::trainOutputs(float * Inputs, float * Outputs, int time, int discardedTimesteps)
    {
        matrix::Matrix * stateMatrix = new
    matrix::Matrix(time-discardedTimesteps, networkNeurons
    /*+(feedbackConnections?outputNeurons:0)*/); //stateMatrix is the matrix ofstates
          //std::cout << "initiated Training \n \n";

          for(int i = 0; i < time; i++) //record states into state matrix
          {
            ESNetwork::setInput(Inputs+(i*inputNeurons), inputNeurons);
            for (unsigned int j = 0; j < outputNeurons;j++)
            {
              outputs->val(j,0) = Outputs[i*outputNeurons+j];
            }


            ESNetwork::takeStep();
            if (i >= discardedTimesteps)
            {
              int j = 0;

              /*for(; j <  inputNeurons; j++)
              {
                stateMatrix->val(i-discardedTimesteps,j)= inputs->val(j,0);
              }
              */
              for(; j <  networkNeurons; j++)
              {
                stateMatrix->val(i-discardedTimesteps,j)=
    intermediates->val(j,0)+ESNetwork::uniform(-0.05,0.05);
              }
              /*if (feedbackConnections)  //this part is tricky: if the outputs
    feed back, they must be part of the state matrix
              {
                for  (; j <  networkNeurons+outputNeurons; j++)
                {
                  stateMatrix->val(i-discardedTimesteps,j)=
    outputs->val(j-networkNeurons,0);
                }
              }*/
            }
          }
          std::cout << "states found \n";
          std::cout << "states matrix is: " << stateMatrix->getM() <<" times " <<
    stateMatrix->getN() << std::endl;
          /*
           *Here, the teacher outputs are collected
           *
           */
          matrix::Matrix * desiredOutput = new
    matrix::Matrix(time-discardedTimesteps, outputNeurons);
          for(int i = discardedTimesteps; i < time; i++)
          {
            for(int j = 0; j < outputNeurons; j++)
            {
              desiredOutput->val(i-discardedTimesteps,j) =
    Outputs[i*outputNeurons+j];
            }
          }
          std::cout<<"desired outputs found \n";

          /* generating: (s?¹ s)?¹ s^t D
          *
          */

          matrix::Matrix * temp2 = new matrix::Matrix(inputNeurons +
    networkNeurons, time-discardedTimesteps);
          *temp2 = *stateMatrix;
          // here was the mistake:
          //*temp2 = temp2->pseudoInverse();
          *temp2 = temp2->toExp(0xFF);
          matrix::Matrix * temp3 = new matrix::Matrix(time-discardedTimesteps,
    time-discardedTimesteps);
          temp3->mult(*temp2, *stateMatrix);               // is (s' s) now



          /*
          matrix::Matrix * Eigens = new matrix::Matrix;
          *Eigens =  eigenValuesRealSym(*temp3);
          temp3->mult(*temp3, (1/Eigens->val(0,0))*20);
          delete Eigens;
          */


          temp3->pseudoInverse();                           // is (s' s)?¹ now WARNING takes long time to compute
          std::cout << "pseudo-Inverse is: " << temp3->getM() <<" times " <<
    temp3->getN() << std::endl;

          matrix::Matrix * temp4 = new matrix::Matrix(time-discardedTimesteps,
    inputNeurons + networkNeurons);
          *temp4 = *stateMatrix;
          temp4->toExp(0xFF);                               //is s' now
          temp4->mult(*temp4, *desiredOutput);              //is s' * D now
          //temp4->mult(*temp4, *temp3);                      //is (s' s)?¹ s' now


          std::cout << " s^T * D is "<<temp4->getM() <<" times "<<temp4->getN()<< std::endl;

          temp4->mult(*temp3, *temp4);
          std::cout << "(s?¹ s)?¹ s^t D is: " << temp4->getM() <<" times " <<
    temp4->getN() << std::endl;
          *endweights = *temp4;

          /*
          //alternate method:
          matrix::Matrix * temp2 = new matrix::Matrix(inputNeurons +
    networkNeurons, time-discardedTimesteps);
          *temp2 = *stateMatrix;
          *temp2 = temp2->pseudoInverse();


          endweights->mult(*temp2, *desiredOutput);*/

          endweights->toExp(0xFF);
          std::cout << "endweights is: " << endweights->getM() <<" times " <<
    endweights->getN() << std::endl;





          delete stateMatrix;
          delete temp2;
          //delete temp3;
          //delete temp4;
          delete desiredOutput;
        }





    void ESNetwork::trainOnlineRecursive(float * Outputs, float forgettingFactor, float td_error)
    {
      //std::cout << "Trained to  "  << Outputs[0] << std::endl;
      //-----------------The matrices we need-----------------------------//
      matrix::Matrix *temp = new matrix::Matrix(1, networkNeurons);
      matrix::Matrix *temp2 = new matrix::Matrix(1, networkNeurons);
   //   matrix::Mtrix *scale = new matrix::Matrix(1,networkNeurons);

      onlineError = new matrix::Matrix(1, outputNeurons); //the Error vector between input and output
      transposedIntermediates = new matrix::Matrix(1, networkNeurons);
      toChangeOutputWeights = new matrix::Matrix(endweights->getM(), endweights->getN());
      /*
       * Step 1: calculate the transpose of the matrix of inner neuron states and the error between Outputs and training outputs
       *
       */


      *transposedIntermediates = *intermediates;
      transposedIntermediates->toTranspose();
      *temp = *endweights;
      temp->toTranspose();

      temp2->mult(*transposedIntermediates, *temp);

      for (int i = 0; i < outputNeurons; i++)
      {
        onlineError->val(0,i) = (atan(Outputs[i])-temp2->val(0,i));/*td_error;*/ //compute the Error between output and desired output -- end STEP 1
      }


      /*
       *  Step 2:Output weights change delta W_out is set to   onlineLearningAutocorrelation * x(t) / (forgetting factor +  X(t)^t * onlineLearningAutocorrelation* x(t) )
       */
      toChangeOutputWeights->mult(*onlineLearningAutocorrelation, *intermediates);
      temp ->mult( *transposedIntermediates, *onlineLearningAutocorrelation);
      temp2->mult(*temp,*intermediates);

      double scale = 1/(forgettingFactor + temp2->val(0,0));
      toChangeOutputWeights->mult(*toChangeOutputWeights, scale); //end STEP 2


      /*
       * Step 3: the onlineLearningAutocorrelation is diminished by  delta_w * x(t)^T *itself and divided by the forgetting factor
       */

     temp->mult(*toChangeOutputWeights, *transposedIntermediates);
      temp2->mult(*temp, *onlineLearningAutocorrelation);

      *temp = *onlineLearningAutocorrelation - *temp2;
      onlineLearningAutocorrelation->mult(*temp, 1/forgettingFactor); //end STEP 3




      /*
       * Finally, the weights are updated and temporary matrices are deleted
       */
     toChangeOutputWeights->toTranspose();
      temp->mult(*onlineError,*toChangeOutputWeights);

   /* FOR Least Means Squares Learning*/
       //    temp->mult(*onlineError,*transposedIntermediates);
       //    temp->mult(*temp,forgettingFactor);


      *endweights = *endweights + *temp;

      //std::cout << "online learning step finished \n \n \n";

       delete temp;
       delete temp2;
      delete onlineError;
      delete transposedIntermediates;
      delete toChangeOutputWeights;
    }


    void ESNetwork::trainBackpropagationDecorrelation(float * Outputs, float learningRate)
    {/*
      for (int i = 0; i< outputNeurons;i++)
      errors[i] = (outputs->val(i,0)) - (Outputs[i]);
      double decorrelativeFactor = 0.002;
      double errorPropagator = 0;
      for (int i = 0; i< networkNeurons;i++)
      {
        decorrelativeFactor += oldIntermediates[i]*oldIntermediates[i];
      }
      for (int i = 0; i< outputNeurons;i++)
      {
        decorrelativeFactor += oldOutputs[i]*oldOutputs[i];
      }
     /* for (int i = 0; i < outputNeurons; i++)
      {
        errorPropagator = 0;
        for (int k = 0; k < outputNeurons; k++)
        errorPropagator+= innerweights->val(i,k+networkNeurons)*(deriSigmoid(oldOutputs[k]))*oldErrors[k];
        std::cout << "Weight change 0: " << ((learningRate * oldIntermediates[0]) * (errorPropagator - errors[0])/(decorrelativeFactor +0.002))<< std::endl;
        for (int j = 0; j < networkNeurons; j++)
       {
          endweights->val(i,j) +=((learningRate * oldIntermediates[j]) * (errorPropagator - errors[i])/(decorrelativeFactor));
       }
      }*/

      /*for (int j = 0; j < networkNeurons; j++)//simplest case
      {
        endweights->val(0,j) += learningRate * oldIntermediates[j] *( innerweights->val(0,networkNeurons)*deriSigmoid(oldOutputs[0])*oldErrors[0]-(errors[0])  )/ (decorrelativeFactor);

      }
      std::cout<<"decorrelativeFactor: "<<decorrelativeFactor << std::endl;
      for (int i = 0; i< outputNeurons;i++)
      {
        oldErrors[i] = errors[i];
        oldOutputs[i] = (outputs->val(i,0));
      }
      for (int i = 0; i< networkNeurons;i++)
      {
        oldIntermediates[i] = intermediates->val(i,0);
      }*/
    }


    //computes the new state of all neurons except input
    void ESNetwork::takeStep(float * Outputs, float learningRate, float td_error , bool learn, int timestep)
    {

      matrix::Matrix * temp = new matrix::Matrix(1,networkNeurons);
      matrix::Matrix * temp2 = new matrix::Matrix(1,networkNeurons);
      matrix::Matrix * temp3 = new matrix::Matrix(1,inputNeurons+networkNeurons);
      matrix::Matrix * temp4 = new matrix::Matrix(networkNeurons, networkNeurons);
      matrix::Matrix * temp5 = new matrix::Matrix(1,networkNeurons);


     if(leak== false)
      {
      temp->mult(*startweights, *inputs);

      if (feedbackConnections)
      {
        temp2->mult(*innerweights, intermediates->above(*outputs))  ;
        temp2->removeRows(networkNeurons);
       }
      else
      {
        temp2->mult( *innerweights, *intermediates);
      }
      intermediates->add(*temp, *temp2);
      //std::cout <<  "inner Neurons computed:\n";

      //Add noise to the inner reservoir
      intermediates->add(*intermediates, *noise);

      ESNetwork::cullInnerVector(0.6);

      if (throughputConnections)
      {
        *temp3 = *inputs;
       *temp3 = temp3->beside(*intermediates);


      }
      else
      {
        *temp3 = *intermediates;
      }

    }
     if (leak == true)
      {

    	  temp->mult(*startweights, *inputs);

    	        if (feedbackConnections)
    	        {
    	          temp2->mult(*innerweights, intermediates->above(*outputs))  ;
    	          temp2->removeRows(networkNeurons);
    	        }
    	        else
    	        {
    	          temp2->mult( *innerweights, *intermediates);
    	        }
    	        intermediates->add(*temp, *temp2);
    	        //std::cout <<  "inner Neurons computed:\n";

    	        intermediates->add(*intermediates, *noise);

    	        ESNetwork::cullInnerVector(0.6);


    	        temp5->mult(*leak_mat,*intermediates);

    	        intermediates->add(*temp5, *intermediates);



    	        if (throughputConnections)
    	        {
    	          *temp3 = *inputs;
    	          *temp3 = temp3->beside(*intermediates);
    	        }
    	        else
    	        {
    	          *temp3 = *intermediates;
    	        }




      }


     if (learn)
     {
       if (td_error!= 0)
       {

         trainOnlineRecursive(Outputs, learningRate, (td_error));

         outputs->mult(*endweights, *temp3);

         outputsCollection[timestep] = outputs->val(0,0);

    //     printMatrix(endweights);

       }
     }

       else {
   	   outputs->mult(*endweights, *temp3);

   	   if(timestep >100)
   		 outputsCollection[timestep] = outputs->val(0,0);

   	   printMatrix(endweights);
       }



     ESNetwork::cullOutput(0.6);


      delete temp;
      delete temp2;
      delete temp3;
      delete temp4;
      delete temp5;
    }

    float ESNetwork::evaluatePerformance(int start,int end, float * desiredOutputs)
    {
      std::cout << "Evaluating Performance" <<std::endl;
    //  if (collectedOutputs == NULL) collectedOutputs = outputsCollection;
   //   if (outputsCollection == NULL)
   //   {
    //    std::cout << "Warning: no collected Outputs found or received" <<std::endl;
    //    return 0;
    //  }
      int i = 0;
      float error = 0;
      for(i = start; i < end; i++)
      {
        error += (outputsCollection[i]-desiredOutputs[i])*(outputsCollection[i]-desiredOutputs[i]);
      }
      return error/(end-start);
    }


    void ESNetwork::resetInput()// sets the input neurons to zero
    {
      std::cout<<"Resetting inputs:" << "\n";
      for (int i = 0; i < inputNeurons; i++)
      {
        inputs->val(i,0) = 0;
      }
      std::cout<<" \n done" << "\n";
    }
    void ESNetwork::scrambleInput() //randomizes input, unused
    {
      for(int i = 0; i< inputNeurons; i++)
      {
        if (rand()%100 > 50)
        {
          inputs->val(i,0) = 1;
        }
        else
        {
          inputs->val(i,0) = 0;
        }
      }
    }

    //functions to print matrices to terminal
    void ESNetwork::printMatrix(matrix::Matrix *printedMatrix)
      {
          unsigned int i = 0;
          unsigned int j = 0;
              std::cout <<  "matrix 1 : \n";
              for(i = 0; i < printedMatrix->getM(); j++)
              {
                if (j == printedMatrix->getN())
                  {
                  j = 0;
                  i++;
                  std::cout << "\n";
                  if (i == printedMatrix->getM()) {break;}
                  }
                std::cout <<  printedMatrix->val(i,j) << " ";
              }
       }
    void ESNetwork::printOutputToTerminal()
    {
      std::cout <<  "Output : ";
      for(unsigned int i = 0; i < outputs->getM();i++)
      {
      std::cout <<  outputs->val(i,0) << " ";
      }
      std::cout << "\n ";
    }
    void ESNetwork::printNeuronsToTerminal()
    {
      std::cout <<  "Neurons : \n";
      for(unsigned  int i = 0; i < intermediates->getM();i++)
      {
      std::cout <<  intermediates->val(i,0) << " ";
      }
      std::cout << "\n ";
    }


    //restricts the range of neuron activation using sigmoid
    void ESNetwork::cullInnerVector(float treshold /*unused*/)
    {
      for(unsigned int i = 0; i < intermediates->getM(); i++)
      {
    	//  intermediates->val(i,0) = ESNetwork::sigmoid(intermediates->val(i,0));
    	  intermediates->val(i,0) = ESNetwork::tanh(intermediates->val(i,0),true);
    //   if (intermediates->val(i,0)*intermediates->val(i,0)<=0.0001) intermediates->val(i,0) = 0;
      }
    }
    void ESNetwork::cullOutput(float treshold /*unused*/)
        {
          for( unsigned int i = 0; i < outputs->getM(); i++)
          {
        	//  outputs->val(i,0) = ESNetwork::sigmoid(outputs->val(i,0));
        	 outputs->val(i,0) = ESNetwork::tanh(outputs->val(i,0), false);
          }
        }
    float ESNetwork::sigmoid(float x)
    {
     return 1./(1. +exp(-x));

    }
    float ESNetwork::deriSigmoid(float x)
    {
      if (x < -3) {return 0;}
      if (x > 3) {return 0;}
      return (x*x-9)*(x*x-9)/(9*(x*x+3)*(x*x+3));
    }

    double ESNetwork::tanh(double x, bool flag)
    {
    	//static int a=0;
    	//static int b=0; Flag is for enabling or disabling IP;

    	if(flag == true)
    	{
    		double eta = 0.001; // learning rate for IP stochastic rule

    	    	para_a = 1.0; para_b = 0.0;



    	    	double y =2./(1.+exp(-2*(para_a*x+para_b)))-1.;

    	    	if(enable_IP)
    	    	{
    	    		double del_b = updateGauss_gain(y, eta);

    	    		para_b = para_b + del_b;

    	    		para_a = para_a + ((eta/para_a)+ del_b*x);

    	    	//	std::cout<<"para_b = "<<para_b;
    	    	}

    	    	//store_para_a[a++] = para_a;
    	    	//store_para_b[b++]= para_b;

    	     return y;

    	}

    	 	else
    	     return 2./(1.+exp(-2*x))-1.;




    }

    double ESNetwork::updateGauss_gain(double y, double eta)
    {
    	double del_b= 0.0;
    	double mu = 0.2; // mean of Gaussian distribution (mean firing rate)
    	double sigma = 0.01;  // standard deviation

    	del_b = -eta*(-(mu/(sigma*sigma)) + (y/(sigma*sigma))*(2*sigma*sigma +1 - (y*y)+ (mu*y)));

    	return del_b;
    }



    void ESNetwork::normalizeInnerWeights(float density)
    {
      matrix::Matrix * Eigens = new matrix::Matrix;
      *Eigens =  eigenValuesRealSym(*innerweights);

      // Workaround for Eigen value decomposition NULL matrix issue from LPZrobot (stupid but works ;))
      while(Eigens->val(0,0)== 0)
    	  *Eigens =  eigenValuesRealSym(*innerweights);

      printMatrix(Eigens);
      innerweights->mult(*innerweights, (1/Eigens->val(0,0))*density);
      *Eigens =  eigenValuesRealSym(*innerweights);
     std::cout<<std::endl;
       printMatrix(Eigens);
      delete Eigens;
    }

    float * ESNetwork::readOutputs()
    {
    float * outputvalues = new float[outputNeurons];
      for(int i = 0;i < outputNeurons;)
      {
        outputvalues[i] = outputs->val(i,0);
      }
      return outputvalues;
    }

    double ESNetwork::uniform(double a, double b)
    {
    	return rand()/(RAND_MAX +1.0)*(b-a)+a;
    }



/*
    ESNetwork::~ESNetwork()
    {
    	 delete inputs;// = new matrix::Matrix(inputNeurons,1);
    	 delete      outputs;// = new matrix::Matrix(outputNeurons,1);
    	 delete      intermediates;// = new matrix::Matrix(networkNeurons,1);


    	       //now the weights:
    	 delete     startweights;// = new matrix::Matrix(networkNeurons, inputNeurons);

    	        //innerweights is bigger if the output feeds back
    	        //int temp =  networkNeurons + (feedbackConnections ?  outputNeurons : 0);
    	        delete     innerweights; //= new matrix::Matrix(networkNeurons, temp);

    	        //endweigts is bigger, if the output feeds back
    	        //temp = (throughputConnections?  inputNeurons : 0)+ networkNeurons;
    	        delete      endweights; //  = new matrix::Matrix(outputNeurons, temp);


    }
*/
