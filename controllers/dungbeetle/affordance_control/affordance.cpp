#include "affordance.h"
#include <iostream>
#include <fstream>
#include <functional>



affordanceController::affordanceController(){
		objects.resize(3);
		createClassifier();
		createAffordance();
	}
void affordanceController::createClassifier(){
	fstream valid_;
	
	initializeTrainingSetFromFile(90);
	cout << " capsule samples: "<< capsule.size()<<endl;
	cout << " ball samples: "<< ball.size()<<endl;
	cout << " box samples: "<< box.size()<<endl;
	//cout << "creating 10 folds"<<endl;
	int box_size,ball_size,capsule_size;

	box_size = box.size();
	ball_size = ball.size();
	capsule_size = capsule.size();
	
	// int count = 0;
	// while(count < 10){
	// 	vector<LearningUnit*> chunk;
	// 	int a,b,c,range;
	// 	range = capsule_size - capsule_size/10;
	// 	a =rand() % range;
	// 	//creating stratified folds to balance the training
	// 	//random_shuffle ( capsule.begin(), capsule.end() );
	// 	for(int i = a; i < a + capsule_size/10; i++){

	// 		LearningUnit* learn_temp = new LearningUnit;
	// 		learn_temp->inputs = capsule[i];
	// 		learn_temp->outputs.push_back(1.0);
	// 		chunk.push_back(learn_temp);
	// 		//capsule.erase(capsule.begin()+i);
	// 	}
	// 	range = ball_size - ball_size/10;
	// 	b = rand() % range;
	// 	//random_shuffle ( ball.begin(), ball.end() );
	// 	for(int i = b ; i <b + ball_size/10; i++){
	// 		LearningUnit* learn_temp = new LearningUnit;
	// 		learn_temp->inputs = ball[i];
	// 		learn_temp->outputs.push_back(-1.0);
	// 		chunk.push_back(learn_temp);
	// 		//ball.erase(ball.begin()+i);
	// 	}
	// 	range = box_size - box_size/10;
	// 	c =rand() % range;
	// 	//random_shuffle ( box.begin(), box.end() );
	// 	for(int i = c ; i < c+box_size/10; i++){
	// 		LearningUnit* learn_temp = new LearningUnit;
	// 		learn_temp->inputs = box[i];
	// 		learn_temp->outputs.push_back(0.0);
	// 		chunk.push_back(learn_temp);
	// 		//box.erase(box.begin()+i);
	// 	}
	// 	random_shuffle(chunk.begin(),chunk.end());
	// 	cout << " fold size: " << chunk.size()<<endl;
	// 	folds.push_back(chunk);
	// 	count+=1;
	// }


	// 	cout << " number of folds:"<<folds.size()<<endl;
	// 	cout << " capsule samples: "<< capsule.size()/10<<endl;
	// 	cout << " ball samples: "<< ball.size()/10<<endl;
	// 	cout << " box samples: "<< box.size()/10<<endl;
	// 	//K fold cross validation
	// 	for(int k = 0; k < folds.size() ; k++){

	// 	cout << " ---------------fold:"<< k <<"----------------"<<endl;

	// 	cout << " initializing network" <<endl ;
	// 		for(int i = 0; i < folds.size() ; i++){
			
	// 			if(i != k){
	// 				for(int j = 0 ; j < folds[i].size() ; j++){
	// 					training_set.push_back(folds[i][j]->inputs);
	// 				}
	// 			}
	// 		}
	// 		cout << "training set size:"<<training_set.size();

	// 		findCenters();

			

	// 		initializeNetwork();
	// 		//training network
	// 		cout << "training network" << endl;
	// 	for(int iterations = 0; iterations < 1000 ; iterations++){
	// 		for(int i = 0; i < folds.size(); i++){

	// 			if(i != k ){
	// 				error_classifier = shape_classification->learn_one_epoch(folds[i],0.1,1);
					
	// 			}
	// 		}

	// 		valid_.open("/home/michelangelo/experiment_aff_lvl2_classifier.txt", ios::out | ios::app );
			
	// 		if (valid_.is_open()){
	// 			valid_ << error_classifier << " ";
	// 			valid_.close();
	// 		}else
	// 			cerr << "couldn't open the file"<<endl;
	// 		}
	// 		cout << "learning error classifier:"<<error_classifier<<endl;
	// 	//Validation
	// 		cout << " validating model"<< endl;
	// 		double accuracy_a= 0.0;
	// 		double accuracy_b= 0.0;
	// 		double accuracy_c = 0.0;
	// 		int c_a,c_b,c_c;
	// 		c_a = 0;
	// 		c_b = 0;
	// 		c_c = 0;
	// 		for(int i = 0; i < folds[k].size(); i++){
				
	// 			vector<double> expected,prediction;
				
	// 			prediction = shape_classification->get_output(folds[k][i]->inputs);
	// 			expected = folds[k][i]->outputs;
	// 			//cout << " expected:"<<expected[0]<<" classification:"<<prediction[0]<<endl;
	// 			//cout << " classification error"	<< abs(expected[0]-prediction[0])<<endl;
	// 			switch((int)expected[0]){
	// 				case 1:
	// 				c_a++;
	// 				if(abs(expected[0]-prediction[0]) <= 0.1)
	// 				accuracy_a += 1.0;
	// 				break;
	// 				case 0:
	// 				c_b++;
	// 				if(abs(expected[0]-prediction[0]) <= 0.1)
	// 				accuracy_b += 1.0;
	// 				break;
	// 				case -1:
	// 				c_c++;
	// 				if(abs(expected[0]-prediction[0]) <= 0.1)
	// 				accuracy_c += 1.0;
	// 				break;
	// 				default:
	// 				break;   

	// 			}


	// 		}

	// 		valid_.open("/home/michelangelo/experiment_aff_lvl2_classifier.txt", ios::out | ios::app );
			
	// 		if (valid_.is_open()){
	// 			valid_ << endl;
	// 			valid_ << " CLASSIFIER ACCURACY CAPSULE: " << accuracy_a/c_a << endl;
	// 			valid_ << " SAMPLES CAPSULE: " << c_a << endl;
	// 			valid_ << " CLASSIFIER ACCURACY BOX: " << accuracy_b/c_b << endl;
	// 			valid_ << " SAMPLES BOX: " << c_b << endl;
	// 			valid_ << " CLASSIFIER ACCURACY BALL: " << accuracy_c/c_c << endl;
	// 			valid_ << " SAMPLES BALL: " << c_c << endl;
	// 			valid_ << " ---------------fold:"<< k <<"----------------"<<endl;
	// 			valid_.close();
	// 		}else
	// 			cerr << "couldn't open the file"<<endl;
			
	// 		cout << " CLASSIFIER ACCURACY CAPSULE: " << accuracy_a/c_a << endl;
	// 		cout << " CLASSIFIER ACCURACY BOX: " << accuracy_b/c_b << endl;
	// 		cout << " CLASSIFIER ACCURACY BALL: " << accuracy_c/c_c << endl;
	// 		delete shape_classification;
	// 		centers.clear();
	// 		sigma.clear();
	// 		training_set.clear();

		
		
	// }
	// PERMUTATION TEST
	cout << "TRAINING CLASSIFIER"<< endl;
	fstream ptests;
	random_shuffle(shape_dataset.begin(),shape_dataset.end());
	for(int i = 0;i<shape_dataset.size()*0.9;i++){
			vector<double> temp_shape(shape_dataset[i].size()-1);
			copy(shape_dataset[i].begin(),shape_dataset[i].end()-1,temp_shape.begin());
			training_set.push_back(temp_shape);
			LearningUnit* learn_temp = new LearningUnit;
			learn_temp->inputs = training_set[i];
			learn_temp->outputs.push_back(shape_dataset[i][shape_dataset[i].size()-1]);
			train.push_back(learn_temp);
	}
	//training network
	findCenters();
	initializeNetwork();
	double error_l;
	for(int i = 0; i < 1; i++){
		error_l = shape_classification->learn_one_epoch(train,0.1,1);
		//cout << "error training:"<<error_l<<endl;
		// ptests.open("/home/michelangelo/experiment_ptest_classifier_learning.txt", ios::out | ios::app );
			
		// 	if (ptests.is_open()){
		// 		ptests << error_l << " ";
		// 		ptests.close();
		// 	}else
		// 		cerr << "couldn't open the file"<<endl;
			
	}
	cout << "error training:"<<error_l<<endl;
	//testing
//	int index;
	// for(int i = shape_dataset.size()*0.9;i<shape_dataset.size();i++){
	// 	vector<double> temp_shape(shape_dataset[i].size()-1),temp_outputs;
	// 	copy(shape_dataset[i].begin(),shape_dataset[i].end()-1,temp_shape.begin());
	// 	testing_set.push_back(temp_shape);
	// 	temp_outputs.push_back(shape_dataset[i][shape_dataset[i].size()-1]);
	// 	expected_outputs.push_back(temp_outputs);
	// 	LearningUnit* learn_temp = new LearningUnit;
	// 	learn_temp->inputs = testing_set[index];
	// 	learn_temp->outputs = expected_outputs[index];
	// 	test.push_back(learn_temp);
	// 	index++;
	// }
	// vector<double> outcome,expectation;
	// double classification_error = 0;
	
	// for(int i = 0; i < test.size();i++){
	// 	outcome = shape_classification->get_output(test[i]->inputs);
	// 	expectation = test[i]->outputs;
	// 	//cout << " expectation: "<< expectation[0] << "classification: "<< outcome[0]<<endl;

	// 	if(abs(expectation[0]-outcome[0]) >= 0.1)
	// 			classification_error += 1.0;
	// }
	// cout << "CLASSIFICATION ERROR:"<<classification_error/test.size()<<endl;
	// ptests.open("/home/michelangelo/experiment_ptest_classifier_learning.txt", ios::out | ios::app );
			
	// 		if (ptests.is_open()){
	// 			ptests << endl;
	// 			ptests << "learning error is above"<<endl;
	// 			ptests << endl;
	// 			ptests << "CLASSIFICATION ERROR: "<<classification_error/test.size() << endl;
	// 			ptests << "PERMUTATION TEST OUTCOME-----PERMUTATION CLASSIFICATION ERROR"<< endl;
	// 			ptests.close();
	// 		}else
	// 			cerr << "couldn't open the file"<<endl;
	// test.clear();
	
	// //TEST ONE: OUTPUT PERMUTATION
	// int n_permutation = 1000;
	// double p_value_t1=0;
	
	// for(int i = 0; i < n_permutation;i++){
		
	// 	next_permutation(expected_outputs.begin(),expected_outputs.end());
	// 	double permutation_error = 0;
		
		
	// 	for(int j = 0; j < testing_set.size();j++){
	// 		LearningUnit* learn_temp = new LearningUnit;
	// 		learn_temp->inputs = testing_set[j];
	// 		learn_temp->outputs = expected_outputs[j];
	// 		test.push_back(learn_temp);
	// 	}

	// 	for(int h = 0; h < test.size();h++){
	// 	outcome = shape_classification->get_output(test[h]->inputs);
	// 	expectation = test[h]->outputs;
	// 	//cout << " expectation: "<< expectation[0] << "classification: "<< outcome[0]<<endl;

	// 	if(abs(expectation[0]-outcome[0]) >= 0.1)
	// 			permutation_error += 1.0;
	// }
	// //cout << "PERMUTATION ERROR:"<<permutation_error/test.size()<<endl;
	// if(permutation_error/test.size() < classification_error/test.size())
	// 	p_value_t1++;
	
	// ptests.open("/home/michelangelo/experiment_ptest_classifier_learning.txt", ios::out | ios::app );
			
	// 		if (ptests.is_open()){
	// 			ptests << permutation_error/test.size() << " ";
	// 			ptests.close();
	// 		}else
	// 			cerr << "couldn't open the file"<<endl;
	
	// test.clear();


	// }

	// p_value_t1=(p_value_t1+1)/(n_permutation+1);
	// cout<<"p value test 1:"<<p_value_t1<<endl;

	// ptests.open("/home/michelangelo/experiment_ptest_classifier_learning.txt", ios::out | ios::app );
			
	// 		if (ptests.is_open()){
	// 			ptests <<endl;
	// 			ptests << "p value: " <<p_value_t1 << endl;
	// 			ptests << "PERMUTATION TEST FEATURES-----PERMUTATION CLASSIFICATION ERROR"<<endl;
	// 			ptests.close();
	// 		}else
	// 			cerr << "couldn't open the file"<<endl;
	
	//TEST TWO: INPUT PERMUTATION
	// n_permutation = 1000;
	// double p_value_t2=0;
	// for(int i = 0; i < n_permutation ; i++){
	// 	double permutation_error = 0;
		
	// 	for(int l = 0; l < testing_set.size();l++){
	// 		next_permutation(testing_set[l].begin(),testing_set[l].end());
	// 	}
		
	// 	for(int j = 0; j < testing_set.size();j++){
	// 		LearningUnit* learn_temp = new LearningUnit;
	// 		learn_temp->inputs = testing_set[j];
	// 		learn_temp->outputs = expected_outputs[j];
	// 		test.push_back(learn_temp);
	// 	}
	// 	for(int h = 0; h < test.size();h++){
	// 	outcome = shape_classification->get_output(test[h]->inputs);
	// 	expectation = test[h]->outputs;
	// 	//cout << " expectation: "<< expectation[0] << "classification: "<< outcome[0]<<endl;

	// 	if(abs(expectation[0]-outcome[0]) >= 0.1)
	// 			permutation_error += 1.0;
	// }
	// //cout << "PERMUTATION ERROR:"<<permutation_error/test.size()<<endl;
	// if(permutation_error/test.size() < classification_error/test.size())
	// 	p_value_t2++;

	// ptests.open("/home/michelangelo/experiment_ptest_classifier_learning.txt", ios::out | ios::app );	
	// if (ptests.is_open()){
	// 	ptests << permutation_error/test.size() << " ";
	// 	ptests.close();
		
	// }else
	// 	cerr << "couldn't open the file"<<endl;
	
	// test.clear();

	// }
	// p_value_t2=(p_value_t2+1)/(n_permutation+1);
	// cout<<"p value test 2:"<<p_value_t2<<endl;

	// ptests.open("/home/michelangelo/experiment_ptest_classifier_learning.txt", ios::out | ios::app );
			
	// if (ptests.is_open()){
	// 	ptests <<endl;
	// 	ptests << "p value: " <<p_value_t2 << endl;
	// 	ptests.close();
	// }else
	// 	cerr << "couldn't open the file"<<endl;
}



void affordanceController::createAffordance(){
	
	objects[0] = new affordance();
	objects[1] = new affordance();
	objects[2] = new affordance();
	//get dataset
	cout <<" init dataset capsule"<<endl;
	initializeTrainingSetFromFile(objects[0],0);
	cout <<" initi dataset box"<<endl;
	initializeTrainingSetFromFile(objects[1],1);
	cout <<" initi dataset sphere"<<endl;
	initializeTrainingSetFromFile(objects[2],2);

	cout <<" training capsule"<<endl;
	trainNetwork(objects[0]);
	cout <<" training box"<<endl;
	trainNetwork(objects[1]);
	cout <<" training sphere"<<endl;
	trainNetwork(objects[2]);



}

void affordanceController::initializeShapeFromFile(affordance *a,int size_shape){
	fstream file;
	file.open("/home/michelangelo/SDU/Thesis/dataset/experiments.txt", ios::in | ios::app );
			if (file.is_open())
			{
				while(!file.eof() && a->object_shape.size() < size_shape)
				{
					double shape_temp;
					file >> shape_temp;
					a->object_shape.push_back(shape_temp);

					cout << shape_temp << " ";
				}
				cout << "\nshape loaded"<< endl;
				cout << a->object_shape.size()<<endl;
				file.close();

			}else

				cerr << "error opening the shape file";

}

void affordanceController::initializeTrainingSetFromFile(affordance *a,int ob){
	fstream file;
	cout << "initializing dataset"<<endl;
	switch(ob){
	case 0:
	file.open("dataset/capsule_dataset_v2.txt", ios::in | ios::app );
			if (file.is_open())
			{
				cout << "file open "<<endl;
				int i = 0;
				while(!file.eof())
				{	//cout << "i:"<<i<<endl;
					Point2f in_temp;
					double out_temp;
					file >> in_temp.x >> in_temp.y >> out_temp;
					a->input_dataset.push_back(in_temp);
					a->output_dataset.push_back(out_temp);
					i++;
				}
				
				file.close();
				cout << "dataset loaded"<<endl;

			}else

				cerr << "error opening the dataset file";
			break;
	case 1:
	file.open("dataset/box_dataset_v2.txt", ios::in | ios::app );
			if (file.is_open())
			{
				cout << "file open "<<endl;
				int i = 0;
				while(!file.eof())
				{	//cout << "i:"<<i<<endl;
					Point2f in_temp;
					double out_temp;
					file >> in_temp.x >> in_temp.y >> out_temp;
					a->input_dataset.push_back(in_temp);
					a->output_dataset.push_back(out_temp);
					i++;
				}
				
				file.close();
				cout << "dataset loaded"<<endl;

			}else

				cerr << "error opening the dataset file";
			break;
	
	case 2:
	file.open("dataset/ball_dataset_v2.txt", ios::in | ios::app );
			if (file.is_open())
			{
				cout << "file open "<<endl;
				int i = 0;
				while(!file.eof())
				{	//cout << "i:"<<i<<endl;
					Point2f in_temp;
					double out_temp;
					file >> in_temp.x >> in_temp.y >> out_temp;
					a->input_dataset.push_back(in_temp);
					a->output_dataset.push_back(out_temp);
					i++;
				}
				
				file.close();
				cout << "dataset loaded"<<endl;

			}else

				cerr << "error opening the dataset file";
			break;
	}

}

void affordanceController::initializeTrainingSetFromFile(int vector_size){
	fstream file,prediction;
	file.open("dataset/points_shape_capsule.txt", ios::in | ios::app );
	prediction.open("dataset/prediction_dataset_capsule.txt", ios::in | ios::app );
			if (file.is_open() && prediction.is_open())
			{
				while(!file.eof() || !prediction.eof())
					{	
					vector<double> t;
					for(int i = 0;i<vector_size;i++){
					double in_temp;
					file >> in_temp;
					t.push_back(in_temp);
					
				}
					double pred_temp;
					prediction >> pred_temp;
					t.push_back(pred_temp);
					capsule.push_back(t);
				
					t.push_back(1.0);
					shape_dataset.push_back(t);

				}
				
				file.close();
				prediction.close();
				cout << "dataset capsule loaded"<<endl;

			}else

				cerr << "error opening the dataset file";

			file.open("dataset/points_shape_box.txt", ios::in | ios::app );
			prediction.open("dataset/prediction_dataset_box.txt", ios::in | ios::app );
			if (file.is_open() && prediction.is_open())
			{
				while(!file.eof() || !prediction.eof() )
					{		
					vector<double> t;
					for(int i = 0;i<vector_size;i++){
					double in_temp;
					file >> in_temp;
					t.push_back(in_temp);


					
				}	
					double pred_temp;
					prediction >> pred_temp;
					t.push_back(pred_temp);
					box.push_back(t);

					t.push_back(0.0);
					shape_dataset.push_back(t);
				}
				
				prediction.close();
				file.close();
				cout << "dataset box loaded"<<endl;

			}else

				cerr << "error opening the dataset file";
			
			file.open("dataset/points_shape_ball.txt", ios::in | ios::app );
			prediction.open("dataset/prediction_dataset_ball.txt", ios::in | ios::app );
			if (file.is_open() && prediction.is_open())
			{
				while(!file.eof() || !prediction.eof())
					{		
					vector<double> t,t2;
					for(int i = 0;i<vector_size;i++){
					double in_temp;
					file >> in_temp;
					t.push_back(in_temp);

				}
					double pred_temp;
					prediction >> pred_temp;
					t.push_back(pred_temp);
					ball.push_back(t);


					t.push_back(-1.0);
					shape_dataset.push_back(t);
					
				

				}
				
				prediction.close();
				file.close();
				cout << "dataset ball loaded"<<endl;

			}else

				cerr << "error opening the dataset file";

}


void affordanceController::findCenters(affordance *a){
	//find centers
	nclusters = 7;
	Mat cluster_labels;
	Mat centers;
	cout << "clustering dataset"<<endl;
	cv::kmeans(a->training_set, nclusters, cluster_labels, TermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 10000, 0.0001), 5/*attempts*/,KMEANS_PP_CENTERS,centers);

	//find variance
	for(int labels = 0; labels < nclusters; labels++){
		a->cluster_centers.push_back(centers.at<Point2f>(labels));
		double tc_mean = 0;
		double size_mean = 0;
		int nsamples = 0;
		double tc_var = 0;
		double size_var = 0;
		vector<Point2f> cluster;

		//compute mean;
		for(int i = 0; i < a->training_set.size() ; i++){
			
			if(cluster_labels.at<int>(i) == labels){

				size_mean += a->training_set[i].x;
				tc_mean += a->training_set[i].y;
				nsamples++;
				cluster.push_back(a->training_set[i]);

			}

		}
		size_mean = size_mean/nsamples;
		tc_mean = tc_mean/nsamples;

		//compute variance
		for(int i = 0; i < cluster.size(); i++){
			size_var += pow((size_mean-cluster[i].x),2);
			tc_var +=  pow((tc_mean-cluster[i].y),2);
		}
		size_var = size_var/nsamples;
		tc_var = tc_var/nsamples;

		Point2d var_temp;

		var_temp.x = size_var;
		var_temp.y = tc_var;

		a->variances.push_back(var_temp);
		


	}
		cout << " dataset centers:\n " <<a->cluster_centers <<endl;
		cout << " cluster variances:\n " <<a->variances <<endl;
}

void affordanceController::findCenters(){
	//find centers
	nclusters = 90;
	centers.resize(nclusters);
	// cout << "finding centers"<<endl;

	random_shuffle ( training_set.begin(), training_set.end() );

	for(int i = 0; i < centers.size();i++){
		
		double random;
		random = rand() % training_set.size()-1;
		centers[i] = training_set[random];
		//centers[i].erase(centers[i].end()-1);
	 	//cout << " center: " <<endl;
	 	//cout << centers[i][j] << " ";
	 	//cout <<endl;
	}
	//find distance between close centers in the list
	
	sigma.resize(centers[0].size());
	int dim_sigma = 0;
	double sum,distance;
	for(int i = 1; i < centers.size();i++){
		vector<double> temp_distance(centers[i].size() );
		
		for(int j = 0; j < centers[i].size() ; j++){
			sum +=pow(centers[i][j]-centers[i-1][j],2);
			//cout << temp_distance[j] << " ";

		}
		distance += sqrt(sum);
		sum = 0;
		//centers_d.push_back(temp_distance);
		// transform(sigma.begin(),sigma.end(),temp_distance.begin(),sigma.begin(),plus<double>());
		 dim_sigma++;
	}
	//average distance between centers
	//cout << " euclidian distance: " << distance<<endl;
	//cout << " sigma: "<<endl;
	for(int i = 0; i <sigma.size() ; i++){
		// cout << " distance sum: "<<sigma[i]<<endl;
		 sigma[i] = distance/dim_sigma;
		 
		 //cout <<sigma[i]<<" ";
	}
		//cout << endl;
	

}
void affordanceController::initializeNetwork(affordance *a){

	//Init Hidden Layer
	vector<RBFneuron*> hidden_layer_temp;

	for(int i = 0; i < nclusters; i++){

		RBFneuron *unit;
		vector<double> center;
		vector<double> variance;

		center.push_back(a->cluster_centers[i].x);
		center.push_back(a->cluster_centers[i].y);

		variance.push_back(a->variances[i].x);
		variance.push_back(a->variances[i].y);

		unit = new RBFneuron(center,variance);
		hidden_layer_temp.push_back(unit);
		
	}
	//cout << " number of hidden neurons: "<<hidden_layer_temp.size()<<endl;

	//init network

	a->aff_model = new RBFNetwork(2,1,hidden_layer_temp);

}

void affordanceController::initializeNetwork(){

	//Init Hidden Layer
	vector<RBFneuron*> hidden_layer_temp;

	for(int i = 0; i < nclusters; i++){

		RBFneuron *unit;
		vector<double> center(centers[i].size());
		vector<double> variance(sigma.size());

		center = centers[i];
		variance = sigma;
		

		unit = new RBFneuron(center,variance);
		hidden_layer_temp.push_back(unit);
		
	}

	this->shape_classification = new RBFNetwork(91,1,hidden_layer_temp);

}



void affordanceController::trainNetwork(affordance *a){
	fstream valid,pred;
	//k-fold validation
// 	int count = 0 ;
// 	//create folds
// 		while(count < 10){
// 			vector<LearningUnit*> chunk;
// 			for(int i=0; i<a->input_dataset.size()/10 ; i++){
				
// 				LearningUnit* learn_temp = new LearningUnit;
// 				learn_temp->inputs.push_back(a->input_dataset[a->input_dataset.size()/10*count+i].x);
// 				learn_temp->inputs.push_back(a->input_dataset[a->input_dataset.size()/10*count+i].y);
// 				learn_temp->outputs.push_back(a->output_dataset[a->input_dataset.size()/10*count+i]);
// 				chunk.push_back(learn_temp);
// 		}
// 			cout << "fold size: "<<chunk.size()<<endl;
// 			batch.push_back(chunk);
// 			count+=1;
// 	}
// 		cout << " number of folds: " << batch.size()<<endl;
// 	random_shuffle(batch.begin(),batch.end());
// 	//training and validation
// 	for(int fold=0;fold<batch.size();fold++){
// 	int start,end;
// 	start = fold*a->input_dataset.size()/10;
// 	end = start + a->input_dataset.size()/10-1;
// 	cout << "range:"<<start<<" "<<end<<endl;
// 	for(int h = 0; h < a->input_dataset.size(); h++){
// 		if(!(h >= start && h <= end))
// 			a->training_set.push_back(a->input_dataset[h]);

// 	}
// 	cout << "training size"<<a->input_dataset.size()<<endl;
// 	cout << "training size"<<a->training_set.size()<<endl;
// 	cout << "----------------"<<fold<<"------------------"<<endl;
// 	//clustering
// 	findCenters(a);
// 	//create rbfnetwork
// 	initializeNetwork(a);
// 	//network training for each fold
// 	cout << "training network"<<endl;
// for(int iterations =0;iterations<100;iterations++ ){
// 	for(int i=0;i<batch.size();i++){
// 	if(i != fold){
// 	// cout << " index fold training: "<<i<<endl;
// 	error = a->aff_model->learn_one_epoch(batch[i],0.08,1);
	
// 	// valid.open("/home/michelangelo/experiment_aff_lvl1_capsule.txt", ios::out | ios::app );
// 	// if (valid.is_open()){
// 	// 	valid << error << " ";
// 	// 	valid.close();
// 	// }else
// 	// 	cerr << "couldn't open the file"<<endl;
// 		cout << "learning error:"<<error<<endl;
// 	valid.open("/home/michelangelo/experiment_learning_aff_lvl1_box.txt", ios::out | ios::app );
// 	if (valid.is_open()){
// 		valid << error << " ";
// 		valid.close();
// 	}else
// 		cerr << "couldn't open the file"<<endl;
		  
// }	

// 	}	


// }
// 		valid.open("/home/michelangelo/experiment_learning_aff_lvl1_box.txt", ios::out | ios::app );
// 	if (valid.is_open()){
// 		valid << endl;
// 		valid << endl;
// 		valid.close();
// 	}else
// 		cerr << "couldn't open the file"<<endl;
// 	cout <<"validating network"<<endl;
// 	double err_va = 0.0;
// 	for(int j = 0;j<batch[fold].size();j++){

// 	vector<double> out;
// 	out = a->aff_model->get_output(batch[fold][j]->inputs);
// 	err_va +=  pow(out[0]-batch[fold][j]->outputs[0],2);

// 	// pred.open("/home/michelangelo/prediction_dataset_capsule.txt", ios::out | ios::app );
// 	// if (pred.is_open()){
// 	// 	pred << out[0] << endl;
// 	// 	//pred << "fold:"<<fold<<endl;
// 	// 	pred.close();
// 	// }else
// 	// 	cerr << "couldn't open the file"<<endl;
    
// 	}
// 	// cout <<"validation error:"<< err_va/batch[fold].size()<<endl;
// 	// 	valid.open("/home/michelangelo/experiment_aff_lvl1_ball.txt", ios::out | ios::app );
// 	// if (valid.is_open()){
// 	// 	valid << err_va/batch[fold].size() << endl;
// 	// 	valid << "fold:"<<fold<<endl;
// 	// 	valid.close();
// 	// }else
// 	// 	cerr << "couldn't open the file"<<endl;
		  
// 	//check
// 	delete a->aff_model;
// 	variances.clear();
// 	cluster_centers.clear();
// 	a->training_set.clear();
// }
	// 	valid.open("/home/michelangelo/experiment_aff_lvl1_ball.txt", ios::out | ios::app );
	// if (valid.is_open()){
	// 	valid<<endl;
	// 	//valid <<"\n"<< err_va/batch[fold].size() << endl;
	// 	//valid << "fold:"<<fold<<endl;
	// 	valid.close();
	// }else
	// 	cerr << "couldn't open the file"<<endl;
				vector<LearningUnit*> chunk;
				
				for(int i=0; i<a->input_dataset.size()*70/100; i++){
				a->training_set.push_back(a->input_dataset[i]);
				LearningUnit* learn_temp = new LearningUnit;
				learn_temp->inputs.push_back(a->input_dataset[i].x);
				learn_temp->inputs.push_back(a->input_dataset[i].y);
				learn_temp->outputs.push_back(a->output_dataset[i]);
				chunk.push_back(learn_temp);
		}
		random_shuffle(chunk.begin(),chunk.end());
		findCenters(a);
		initializeNetwork(a);

		for(int i = 0;i < 100;i++){
			error=a->aff_model->learn_one_epoch(chunk,0.1,1);
			
		// valid.open("/home/michelangelo/experiment_learning_aff_lvl1_box.txt", ios::out | ios::app );
		// if (valid.is_open()){
		// valid << error << endl;
		// valid.close();
		// }else
		// cerr << "couldn't open the file"<<endl;
		 }
		 cout << "learning error:"<<error<<endl;


}


double affordanceController::getError(){
	return error;
}

// void affordanceController::setSize(){
// 	objects[0]->size = 1.35;
// }

// void affordanceController::setAngle(){
// 	objects[0]->TC_angle = 0.601;

// }

void saveRBFinfo(){}