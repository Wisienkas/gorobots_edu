#ifndef ASLT_H_
#define ASLT_H_

#include "utils/ann-framework/ann.h"

/************************************************
*** Trigger detection FF NN
************************************************/
class ASLT0 : public ANN {
public:
    ASLT0();
    Neuron* getNeuronOutput();
};

class ASLT1 : public ANN {
public:
    ASLT1();
    Neuron* getNeuronOutput();
};

class ASLT2 : public ANN {
public:
    ASLT2();
    Neuron* getNeuronOutput();
};

class ASLT3 : public ANN {
public:
    ASLT3();
    Neuron* getNeuronOutput();
};

class ASLT4 : public ANN {
public:
    ASLT4();
    Neuron* getNeuronOutput();
};

class ASLT5 : public ANN {
public:
    ASLT5();
    Neuron* getNeuronOutput();
};

class ASLT6 : public ANN {
public:
    ASLT6();
    Neuron* getNeuronOutput();
};

class ASLT7 : public ANN {
public:
    ASLT7();
    Neuron* getNeuronOutput();
};


/************************************************
*** Accumulation of all FF NN
************************************************/
class ASLT : public ANN {
public:

	ASLT();
    const double& getOutputNeuronOutput(const int& index)
    {
      return getOutput(outputNeurons[index]);
    }

    void setInputNeuronInput(const int& index, const double& value)
    {
      setInput(inputNeurons[index], value);
    }
    
    ASLT0* getASLT0(){
    	return aslt0;
    }
    ASLT1* getASLT1(){
    	return aslt1;
    }  
    ASLT2* getASLT2(){
    	return aslt2;
    }    
    ASLT3* getASLT3(){
    	return aslt3;
    }    
    ASLT4* getASLT4(){
    	return aslt4;
    }    
    ASLT5* getASLT5(){
    	return aslt5;
    }    
    ASLT6* getASLT6(){
    	return aslt6;
    }    
    ASLT7* getASLT7(){
    	return aslt7;
    }   
  

	/* seems to only do one layer per step, therefor 1 step per layer
	** only an issue when actually using the main network and not only subnets */	
    void allSteps(){
    	step(); //step(); step(); //step();	step();
    }

private:
	ASLT0*  aslt0;
	ASLT1*  aslt1;
	ASLT2*  aslt2;
	ASLT3*  aslt3;
	ASLT4*  aslt4;
	ASLT5*  aslt5;
	ASLT6*  aslt6;					
	ASLT7*  aslt7;		
	std::vector<Neuron*> inputNeurons;
	std::vector<Neuron*> outputNeurons;
};



#endif /* ASLT_H_ */
