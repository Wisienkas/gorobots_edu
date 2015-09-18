#include <iostream>

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <iostream>
#include <fstream>

#include <thread>
#include <mutex>
#include <string>
#include <time.h>

//ROS Stuff
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"

//#include "../../../utils/ann-framework/ann.h"
#include "bluetooth.h"
#include "dataVector.h"
#include "featureVector.h"

using namespace std;

void bluetoothRead();
void doClassification();
int z;
double frequency = 530;
double freq_delay = (1/frequency)*1000000;
int sample_size = 10;
int sample_delay = sample_size/2;
int sample_delay_counter = sample_delay;
int ctr;
int ctr2;
int ctr3;

vector<DataVector> sampleFrames;
mutex dataMutex;
bool newdata = false;

int main(int argc, char**argv) {
	thread btThread(bluetoothRead);
	thread classifyThread(doClassification);

	btThread.join();
	classifyThread.join();

	return 0;
	}





void bluetoothRead() {

	Bluetooth bt;
//	ros::init(argc, argv, "bluetoothnode");
//	//initialize node handler
//	ros::NodeHandle nh;
//
//	//initialize publishers and their topics
//	ros::Publisher pub1 = nh.advertise<std_msgs::Int32>("bluetooth", 1);
//	std_msgs::Int32 msg;

	bt.init();

	DataVector data;

	while (ctr2<100)
	{
		ctr2++;
		ctr++;
		//double newValue = bt.readValue();
		data.add(ctr);

		if (data.size() >= sample_size)
		{
			sample_delay_counter++;

			if(sample_delay_counter >= sample_delay)
			{
				dataMutex.lock();
				sampleFrames.push_back(data);

				cout << "bt thread : "<< sampleFrames.size() << "\n";


				/*for (int i=0; i<data.size(); i++)
				{
				cout << "BTThread: " << data[i]<<"\n";
				}*/

				data.erase(data.begin());
				dataMutex.unlock();
				sample_delay_counter = 0;
				usleep(2000);
			}
			else
			{
				data.erase(data.begin());
				usleep(2000);
			}
		}

//		msg.data = newValue
//		pub1.publish(msg);

	}

	bt.shutdown();

}

void doClassification() {
	bool check;

	DataVector input;
	while (ctr3<100)
	{
		check = true;
		//cout << sampleFrames.size();
		ctr3++;
		dataMutex.lock();
		check = sampleFrames.size() > 0;
		cout << "process thread : "<< sampleFrames.size() << "\n";
		dataMutex.unlock();

		usleep(20000);


		if(check)
		{
			input.clear();
			dataMutex.lock();
			input = DataVector(sampleFrames.at(0));

			cout << "been here\n";
			/*for (int i=0; i<input.size(); i++)
			{
				cout << "ClassThread: " << input[i]<<"\n";
			}*/

			sampleFrames.erase(sampleFrames.begin());
			dataMutex.unlock();

			input.getMax();

		}
	}
}
