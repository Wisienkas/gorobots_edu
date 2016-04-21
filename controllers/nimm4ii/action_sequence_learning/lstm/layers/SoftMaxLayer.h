#ifndef LAYERS_SOFTMAXLAYER_H
#define LAYERS_SOFTMAXLAYER_H

#include <vector>

class SoftMaxLayer
{
struct Neuron{
	std::vector<float> weights;
	float biasWeight;
	float activation;
	float output;
};
public:
    /**
     * Constructs the Layer
     *
     * @param numInputs       Number of Inmputs
     */
    SoftMaxLayer();
    /**
     * Destructor
     */
    virtual ~SoftMaxLayer();

	virtual void setNeurons(int _numNeurons, int _numInputs, float _bias);
	
	virtual void setInputs(std::vector<float> _inputs);

	virtual void setWeights(std::vector<float> _inputs);
	virtual void step();
	virtual int getOutput() {return output;}
	virtual std::vector<float> getOutputVec();
	virtual float sumVecWeight(std::vector<float> inputVec, std::vector<float> weights);

	/* TODO: move to private and functions */	
	std::vector<Neuron> neurons;    

protected:
	int numNeurons, numInputs;
	int output;
	float bias;

	std::vector<float> inputs;
};


#endif
