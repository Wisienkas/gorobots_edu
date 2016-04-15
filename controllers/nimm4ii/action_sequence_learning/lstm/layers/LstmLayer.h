#ifndef LAYERS_LSTMLAYER_H
#define LAYERS_LSTMLAYER_H

#include <vector>
#include "LstmBlock.h"

class LstmLayer
{
typedef std::vector<float> real_vector;

public:
    /**
     * Constructs the Layer
     *
     * @param numInputs       Number of Inmputs
     */
    LstmLayer();
    /**
     * Destructor
     */
    virtual ~LstmLayer();
    virtual void reset();
    virtual void setup(int _numInputs, int _numBlocksInLayer, float _bias);
    virtual void setInputs(std::vector<float> _inputs);
	virtual void step();
	virtual std::vector<float> getOutputs();

	/*TODO: move to private and create functions*/
	std::vector<LstmBlock> blocks;    
private:    

	std::vector<float> outputs;
	int numBlocks;
};


#endif
