#ifndef LAYERS_INPUTLAYER_H
#define LAYERS_INPUTLAYER_H

#include <vector>


class InputLayer
{
public:
    /**
     * Constructs the Layer
     *
     * @param numInputs       Number of Inmputs
     */
    InputLayer();
    /**
     * Destructor
     */
    virtual ~InputLayer();

	virtual void setNeurons(int numNeurons);

    virtual void setInput(int id, float value);

    virtual float getInput(int id);

    virtual std::vector<float> getInputVec();
    virtual void setInputs(std::vector<float> values);
    
protected:
	int numNeurons;
	std::vector<float> neurons;
};


#endif
