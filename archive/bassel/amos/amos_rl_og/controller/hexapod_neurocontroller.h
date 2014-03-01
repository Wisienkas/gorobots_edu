







#ifndef __HEXAPOD_NEUROCONTROLLER__
#define __HEXAPOD_NEUROCONTROLLER__



class HexapodNeuroMotionGenerator
{
private:
	double WeightH1_H1;
	double WeightH2_H2;
	double WeightH1_H2;
	double WeightH2_H1;
	double fact;
	double direction;
	double bias;


	double outputH1;
	double outputH2;

public:


	HexapodNeuroMotionGenerator();

	void motor_command_interpreter(double motor_comamnd, double* y_);
};

#endif
