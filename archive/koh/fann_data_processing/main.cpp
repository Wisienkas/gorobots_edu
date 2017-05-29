/********************************************************
Benutzt die von extract_training_data extrahierten Daten
und der Winkelmesser als Inputs fuer ein mehrschichtiges feed-forward FANN.
Das FANN soll Motor-Ausgangssignal des linken Knies vorhersagen.

Basiert auf den Arbeiten Johannes zur Vorhersage des AS-Signals.
(Die Anpassungen betreffen nur die verwendeten Kanäle.)
 ********************************************************/


#include <stdio.h>
#include <fstream>
#include <iostream>
#include <string.h>
#include <floatfann.h>
#include <cstring>
#include "fann_cpp.h"


#include "fann.h"
#include "network.h" // test


using namespace std;


int train ( const string filename, const float desired_error, float learning_rate ) {

	/////// Added // the log file for saving data and ploting
	std::ofstream test_log;
	test_log.open ("./train_vali.dat");

	ofstream outputF1, outputF2, outputF3;
	outputF1.open("error.dat");


	float error;

	const unsigned int num_layers = 3; // Can Change Number of layers
	unsigned int neurons_per_layer[num_layers] = {INPUTS, HIDDEN, OUTPUTS};
	const unsigned int max_iterations = 50000;//500000;//2000000;//50000;
	const unsigned int iterations_between_reports = 100;


	char fninput[200];
	char fnnet[200];
	strcpy ( fninput, filename.c_str() );
	strcpy ( fnnet, ( filename + ".net" ).c_str() );

	cout << "Training with desired error = " << desired_error <<
			" and learning rate = " << learning_rate << "..." << endl;
	float actual_error = 0;

	struct fann *ann = fann_create_standard_array ( num_layers, neurons_per_layer );
	fann_set_learning_rate ( ann, learning_rate );
	struct fann_train_data *train_data;

	train_data = fann_read_train_from_file ( fninput );
	fann_shuffle_train_data ( train_data );

	fann_set_activation_function_hidden ( ann, FANN_SIGMOID_SYMMETRIC );
	//fann_set_activation_function_hidden ( ann, FANN_SIGMOID);

	fann_set_activation_function_output ( ann, FANN_LINEAR );
	// fann_set_activation_function_output ( ann, FANN_SIGMOID_SYMMETRIC );
	// Training alorithm definitions: http://leenissen.dk/fann/fann_1_2_0/r1996.html

	//  fann_set_training_algorithm ( ann, FANN_TRAIN_INCREMENTAL);
	//  fann_set_training_algorithm ( ann,  FANN_TRAIN_RPROP );

	//fann_set_training_algorithm ( ann,   FANN_TRAIN_QUICKPROP);
	fann_set_training_algorithm ( ann,   FANN_TRAIN_BATCH);


	fann_randomize_weights ( ann, -0.25, 0.25 );
	fann_reset_MSE ( ann );

	//fann_train_on_file ( ann, fninput, max_iterations,iterations_between_reports, desired_error );

	// PATH TO TRAINING DATA HERE
	struct fann_train_data *data = fann_read_train_from_file("training");


	// PATH TO VALIDATION DATA HERE
	FANN::training_data vali;
	vali.read_train_from_file("validation"); //It should be the file that contains validation data



	for(int i = 1 ; i <= max_iterations; i++) {
		error = fann_train_epoch(ann, data);
		if ( error < desired_error ) {
			break;
		}


		printf("Training Error: %f", error); // Training error

		/////// Added // for testing validation
		fann_reset_MSE(ann);

		for(int i = 0 ; i != vali.length_train_data() ; i++ ) {
			fann_test(ann,vali.get_input()[i], vali.get_output()[i]);
		}

		printf("  \tValidation Error: %f\n", fann_get_MSE(ann)); // Validation error
		test_log << i << "\t" << error << "\t" << fann_get_MSE(ann) << "\n";
		/////// Added // for testing validation


		//cout << "error" << error << endl;
	}
	fann_destroy_train(data);

	// Testing network
	//http://leenissen.dk/fann/html/files/fann_train-h.html#fann_test_data


	//	actual_error = fann_train_epoch ( ann, train_data );
	//	outputF1<< actual_error <<endl;

	fann_save ( ann, fnnet );

	fann_destroy ( ann );

	/////// Added // the log file for saving data and ploting
	test_log.close();

	cout << "Training done!!!!" << endl;
	return 0;


	///////////////////////////////////////////////////??????????????
	bool stop = false;

	for ( unsigned int epoch=0; epoch<max_iterations; epoch++ ) {

		actual_error = fann_train_epoch ( ann, train_data );
		cout << "test5" << epoch << ", error " << actual_error << endl;
		outputF1<< actual_error <<endl;

		if ( ( epoch%iterations_between_reports ) ==0 ) {
			cout << "epoch " << epoch << ", error " << actual_error << endl;
		}

		char ch;
		cin.get(ch);
		switch ( ch ) {
		case 27:
			stop = true;
			break;
		case 'l':
			cout << "new learning rate: " << endl;
			cin >> learning_rate;
			break;
		}

		if ( actual_error < desired_error || stop ) {
			cout << "epoch " << epoch << ", error" << actual_error << endl;
			break;
		}


	}

	fann_save ( ann, fnnet );

	fann_destroy ( ann );
	cout << "Training done." << endl;
}


int main( int argc, char *argv[] ) {



	char key;
	float desired_error = 0.0;
	float learning_rate = 0.0;

	if ( argc < 2 ) {
		printf ( "Arguments: <input>\n" );
		printf ( "<input> needs to be the filename.\n" );
		exit ( 0 );
	}

	string filename = argv[1];

	cout << "\nPlease supply the following two values." << endl;
	cout << "\nDesired error [0.001]: ";
	cin >> desired_error;
	cout << "\nlearning_rate [0.01]: ";
	cin >> learning_rate;
	cout << endl;
	train ( filename, desired_error, learning_rate );



	cout << "\nPress <anykey> to end program.";
	cin.get(key);

	return 0;
}
// kate: indent-mode cstyle; space-indent on; indent-width 4;
