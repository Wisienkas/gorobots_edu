#ifndef DNF_H
#define DNF_H

#include <vector>

struct GaussStimulus {
	int pos;
	double sigma;
	double amplitude;	
};

struct LateralInteraction {
	double sigmaExc;
    double amplitudeExc;
    double sigmaInh;
    double amplitudeInh;
    double amplitudeGlobal;
	double cutoffFactor;

	int kernelRangeLeft;
	int kernelRangeRight;
	std::vector<double> kernel;	
	std::vector<double> output;
	double fullSum;
};

class DNF
{
public:
    /**
     * Constructs the Layer
     */
    DNF();
    /**
     * Destructor
     */
    virtual ~DNF();
    /**
     * setup
     *
     * @param size
     * @param tau
     * @param h
     * @param beta               
     */
 	void setup(int size, double tau, double h, double beta);
 
    /**
     * sigmoid
     */	
 	double sigmoid(double x, double beta);
 	/**
     * calc gauss value
     */
 	double gauss(int pos, int mu, double sigma);
 
  	/**
     * Stim functions
     */	
	void addStim(int pos, double sigma);
	void setAmplitudes(std::vector<double> amplitudes);
	std::vector<double> getSumStims();
 
   	/**
     * lateral Interaction
     */
    void setupLateral(double _sigmaExc, double _amplitudeExc, double _sigmaInh, double _amplitudeInh, double _amplitudeGlobal, double _cutoffFactor);
 	std::vector<double> getLateral();
 	 /**
     * Step functions
     */
	void step();
	
 	/**
     * return fuctions
     */
	std::vector<double> getActivity() { return activation;}
	std::vector<double> getOutput() { return output;}
	
 	/**
     * params
     */	
	int size;
	double tau;
	double h;
	double beta;
 	/**
     * stimuli
     */	
	int stimSize;
	std::vector<GaussStimulus> stimuli;

	/**
     * lateral
     */	
	LateralInteraction lateralInteraction;
	
	/**
     * output
     */	
	std::vector<double> activation;
	std::vector<double> output;	
	
private:

};


#endif
