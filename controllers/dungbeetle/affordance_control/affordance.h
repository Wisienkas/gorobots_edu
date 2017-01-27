#ifndef AFFORDANCE
#define AFFORDANCE

#include <opencv2/opencv.hpp>
#include "utils/rbf-framework/c++_library/rbf_network.cpp"


using namespace std;
using namespace RBFnetwork;
using namespace cv;



//to re-design the structure

class affordance{
public:

	affordance(){};
	~affordance(){};

	int index;
	vector<double> object_shape;
	double size;
	double TC_angle;
	RBFNetwork *aff_model;
	double error;
	double correct_output;

	vector<Point2f> input_dataset;
	vector<Point2f> training_set;
	vector<double> output_dataset;
	vector<Point2f> cluster_centers;
	vector<Point2d> variances;
};

class affordanceController{
public:
	//constructor
	affordanceController();
	//descructor
	~affordanceController(){};
	//create new obj
	void createAffordance();

	void createClassifier();
	//check if there's already the object model
	void detectObject();

	int getIndex();

	vector<double> getObjectShape();

	double getObjectSize();

	double getLegsAngle();

	double getNetworkInformation();

	void setSize();

	void setAngle();

	void initializeTrainingSetFromFile(affordance *a,int ob);

	void initializeTrainingSetFromFile(int vector_size);
	
	void initializeShapeFromFile(affordance *a,int size_shape);

	void initializeNetwork();
	
	void initializeNetwork(affordance *a);

	void trainNetwork(affordance *a);

	void findCenters();

	void findCenters(affordance *a);

	double getError();

	void saveRBFinfo();
	

public:
	double error,error_classifier;


	RBFNetwork* shape_classification;
	vector<vector<double>> ball,box,capsule;
	
	vector<double> sigma;
	vector<vector<double>> training_set,shape_dataset,testing_set,expected_outputs;
	
	vector<affordance*> objects;
	vector<vector<LearningUnit*>> batch,folds;
	vector<LearningUnit*> test,train;
	int nclusters;
	int step;
	vector<vector<double>> centers;
	vector<Point2f> cluster_centers;
	vector<Point2d> variances;

};


#endif