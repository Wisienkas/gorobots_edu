#ifndef LAYERS_FEEDFORWARDLAYER_H
#define LAYERS_FEEDFORWARDLAYER_H

#include <vector>



class FeedForwardLayer
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
    FeedForwardLayer();
    /**
     * Destructor
     */
    virtual ~FeedForwardLayer();

	virtual void setNeurons(int _numNeurons, int _numInputs, float _bias);
	
	virtual void setInputs(std::vector<float> values);

	virtual void setWeights(std::vector<float> values);
	virtual void step();
	virtual float getOutput() {return output;}
	virtual float sumVecWeight(std::vector<float> inputVec, std::vector<float> weights);

	/*TODO: move to private and create functions*/    
  	std::vector<Neuron> neurons; 
   
protected:
	int numNeurons, numInputs;
	float output;
	float bias;
	std::vector<float> inputs;
};


#endif
