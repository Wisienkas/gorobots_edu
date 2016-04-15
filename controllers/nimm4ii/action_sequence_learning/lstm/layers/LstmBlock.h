#ifndef LAYERS_LSTMBLOCK_H
#define LAYERS_LSTMBLOCK_H

#include <vector>

    /******************************************************************************************//**
     * Represents a fully connected layer which uses LSTM cells with forget gates, peephole
     * connections and one cell per block
     *
     * weights; with P = precedingLayer().size() and L = size():
     *    ~ weights from preceding layer:
     *        - [0 .. PL-1]:    net input
     *        - [PL .. 2PL-1]:  input gate
     *        - [2PL .. 3PL-1]: forget gate
     *        - [3PL .. 4PL-1]: output gate
     *    ~ bias weights:
     *        - [4PL + 0  .. 4PL + L-1]:  net input
     *        - [4PL + L  .. 4PL + 2L-1]: input gate
     *        - [4PL + 2L .. 4PL + 3L-1]: forget gate
     *        - [4PL + 3L .. 4PL + 4L-1]: output gate
     *    ~ internal weights (from other cells in the same layer):
     *        - [4(P+1)L + 0   .. 4(P+1)L + LL-1]:  net input
     *        - [4(P+1)L + LL  .. 4(P+1)L + 2LL-1]: input gate
     *        - [4(P+1)L + 2LL .. 4(P+1)L + 3LL-1]: forget gate
     *        - [4(P+1)L + 3LL .. 4(P+1)L + 4LL-1]: output gate
     *    ~ peephole weights (from cell state to all gates in the same cell):
     *        - [4(P+1+L)L + 0   .. 4(P+1+L)L + L-1]:  input gate
     *        - [4(P+1+L)L + LL  .. 4(P+1+L)L + 2L-1]: forget gate
     *        - [4(P+1+L)L + 2LL .. 4(P+1+L)L + 3L-1]: output gate
     *
     * @param TDevice The computation device (Cpu or Gpu)
     *********************************************************************************************/

class LstmBlock
{
typedef std::vector<float> real_vector;

public:
    /**
     * Constructs the Layer
     */
    LstmBlock();
    /**
     * Destructor
     */
    virtual ~LstmBlock();
    /**
     * initiailize vectors and weights to correct size and 0.0 value
     */    
    virtual void setup(int _numInputs, int _numBlocksInLayer, float _bias);
    /**
     * set inputs
     */    
	virtual void setInputs(std::vector<float> inputs);
    /**
     * set Internal inputs
     */ 
	virtual void setInternal(std::vector<float> internal);
    /**
     * do a forward pass
     */ 
	virtual void step();
    /**
     * get output
     */ 
	virtual float getOutput();
	
	virtual float sumVecWeight(std::vector<float> inputVec, std::vector<float> weights);

	virtual void reset();

	/* TODO: move to private create functions */
	// weight vectors
	std::vector<float> precedingToNet;
	std::vector<float> precedingToInput;
	std::vector<float> precedingToForget;
	std::vector<float> precedingToOutput;
	
	float biasToNet;
	float biasToInput;
	float biasToForget;
	float biasToOutput;
	
	std::vector<float> internalToNet;
	std::vector<float> internalToInput;
	std::vector<float> internalToForget;
	std::vector<float> internalToOutput;
	
	float peepCellToInput;
	float peepCellToForget;
	float peepCellToOutput;	

    
private:    
//	std::vector<double> neurons;
	
	int numInputs;
	int numBlocksInLayer;
	
	float bias;
	float i,z,f,c,o,y; // naming convention from "LSTM: A Search Space Odyssy"
	std::vector<float> inputsPreceding;
	std::vector<float> inputsInternal; // from this layer, just to stay with the convention


	
};


#endif
