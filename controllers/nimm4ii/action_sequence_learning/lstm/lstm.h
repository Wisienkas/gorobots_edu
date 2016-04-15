#ifndef LSTM_H
#define LSTM_H

#include <vector>
#include <string>
#include "layers/InputLayer.h"
#include "layers/LstmLayer.h"
#include "layers/SoftMaxLayer.h"
#include "layers/FeedForwardLayer.h"

class LSTM
{
public:
    /**
     * Constructs the Layer
     */
    LSTM();
    /**
     * Destructor
     */
    virtual ~LSTM();
    
    /**
     * Sets up Layer
     *
     * @param inputs       Number of Inputs
     * @param lstms        Number of LSTM Blocks in LSTM Layer     
     * @param outputs      Number of Outputs
     * @param bias		   bias value
     */
    virtual void setup(int _inputs, int _lstms, int _outputs, float _bias);
    
    /**
     * reset interal state values
     */
    virtual void reset();
    /**
     * load weights from CURRENNT jsn file
     */
    virtual void loadWeights(std::string filename);
    
    virtual void setInput(std::vector<float> inVec);
    virtual void step();
    /**
     * Output of Softmax from Multiclass Classification
     */
    virtual std::vector<float> getOutput();
    /**
     * argmax(softMax output)
     */
    virtual int getState();
    /**
     * out from FF Logistic Layer in Binary Classification
     */
    virtual float getBinary();    
    
    virtual InputLayer getInputLayer() {return inputLayer;}
    virtual LstmLayer getLstmLayer() {return lstmLayer;}    
    
private:
	int inputs, lstms, outputs;
	InputLayer inputLayer;
	LstmLayer lstmLayer;
	SoftMaxLayer softMaxLayer;	
	FeedForwardLayer feedForwardLayer;
};


#endif
