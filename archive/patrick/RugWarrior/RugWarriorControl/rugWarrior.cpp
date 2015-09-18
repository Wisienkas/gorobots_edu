#include "rugWarrior.h"
namespace RWControl{

RugWarrior::RugWarrior(const char* cfgFileName) : imuDevice(cfgFileName){
	this->init(cfgFileName);
}

RugWarrior::~RugWarrior(){
	this->disconnect();
}

void RugWarrior::init(const char* cfgFileName){
	std::string deviceName;

	libconfig::Config cfg;
	cfg.readFile(cfgFileName);
	const libconfig::Setting& root = cfg.getRoot();
	const libconfig::Setting& setting = root["RugWarrior"];
	setting.lookupValue("deviceName", deviceName);
	setting.lookupValue("obstacleThreshold", this->obstacleThreshold);
	setting.lookupValue("refrPeriod", this->refrPeriodBase);
	setting.lookupValue("steerGainOA", this->steerGainOA);
	setting.lookupValue("brake", this->brake);
	setting.lookupValue("biasLeft", this->biasLeft);
	setting.lookupValue("biasRight", this->biasRight);
	setting.lookupValue("translationalVelocity", this->veloStraight);
	setting.lookupValue("rotationalVelocity", this->veloRot);

	setting.lookupValue("sampleRateLRF", this->sampleRateLRF);
	setting.lookupValue("sampleRateVelo", this->sampleRateVelo);

	this->lastTouch = 0;
	this->refrPeriod = 0;
	this->connect(deviceName.c_str());
	this->logging = false;
	this->isTurning = false;
}

bool RugWarrior::connect(const char* deviceName){
	//Init USB Dux
	this->comediDev = NULL;
	this->comediDev=comedi_open(deviceName);
	if(!this->comediDev)
		comedi_perror(deviceName);
	//Init LRF
	bool lrfConnected = this->urg.connect();
	this->connected = this->comediDev != 0 && this->imuDevice.isConnected() && lrfConnected;
	return this->connected;
}

void RugWarrior::writeDataToFile(std::string dataFolder){
	//Write IMU data
	this->imuDevice.writeDataToFile(dataFolder + "dataIMU.dat");
	//Write Velocity and position
	std::string fileName = dataFolder + "rw.dat";
	std::ofstream out(fileName.c_str());
	out << "#Sample time [sec]: "<<this->getSampleTime()<<std::endl;
	out<<"#step\tposX\tposY\tposZ\tvX\tvY\tvZ\tvYaw"<<std::endl;
	for(int i = 0; i < (int)this->positions.size(); i++){
		out<<this->indicesVelo[i]<<"\t"<<positions[i][0]<<"\t"<<positions[i][1]<<"\t"<<positions[i][2]<<"\t";
		out<<velocities[i][0]<<"\t"<<velocities[i][1]<<"\t"<<velocities[i][2]<<"\t"<<velocities[i][5]<<std::endl;
	}
	out.close();
	///Write LRF
	std::string baseFileName = dataFolder + "lrf";
	std::stringstream ss;
	for(int i = 0; i < (int)this->urgData.size(); i++){
		ss << this->indicesLRF[i];
		fileName  = baseFileName + ss.str() + ".dat";
		out.open(fileName.c_str());
		out << "#Step\tLength"<<std::endl;
		for(int j = 0; j < (int)this->urgData[i].size(); j++)
			out << j << "\t"<< this->urgData[i][j]<<std::endl;
		ss.str("");
		ss.clear();
		out.close();
	}
}

void RugWarrior::disconnect(){
	this->stopLogging();
	this->stop();
	comedi_close(this->comediDev);
	this->imuDevice.disconnect();
	this->urg.disconnect();
	this->connected = false;
}

int RugWarrior::getSizeData(){
	return (int)this->velocities.size();
}

bool RugWarrior::isConnected(){
	return this->connected;
}

bool RugWarrior::isLogging(){
	return this->logging;
}

void RugWarrior::startLogging(){
	if(!this->logging && this->connected){
		this->logging = true;
		this->imuDevice.startReading();
		//IMU takes some time to start reading thread
		usleep(100000);
		this->startPoint = boost::chrono::high_resolution_clock::now();
		this->threadLogging = boost::thread(&RugWarrior::doLogging,this);
	}
}

void RugWarrior::stopLogging(){
	if(this->logging){
		this->threadLogging.interrupt();
		this->totalTime = boost::chrono::high_resolution_clock::now() - startPoint;
		this->imuDevice.stopReading();
		this->logging = false;
	}

}

void RugWarrior::resetVelocityIMU(){
	this->imuDevice.resetVelocity();
}

double RugWarrior::getSampleTime(){
	return this->totalTime.count() / this->getSizeData();
}

void RugWarrior::doLogging(){
	int steps = 0;
	while(true){
		if(steps % this->sampleRateVelo == 0){
			//If robot is turning, forward velocity is zero
			if(this->isTurning)
				this->resetVelocityIMU();
			this->positions.push_back(this->imuDevice.getPosition());
			this->velocities.push_back(this->imuDevice.getVelocity());
			this->indicesVelo.push_back(steps);
		}

		if(steps % this->sampleRateLRF == 0){
			this->urg.updateData();
			//If robot is turning, ignore LRF reading and store neutral one instead
			//(Neutral = Consists only of error readings, thus that mapping will ignore it)
			if(this->isTurning){
				std::vector<long> zeros(urg.data.size());
				urgData.push_back(zeros);
			}
			else
				urgData.push_back(urg.data);
			this->indicesLRF.push_back(steps);
		}
		//Wait some time between samples
		usleep(5000);
		steps++;
		//Set interruption point to be able to cancel thread
		boost::this_thread::interruption_point() ;
	}
}

void RugWarrior::stop(){
	this->isTurning = false;
	this->setMotors(2047,2047);
	this->imuDevice.resetVelocity();
}

void RugWarrior::setMotors(int valueLeft, int valueRight){
	//Send command to left motor (channel 0 in subdevice 1)
	lsampl_t ldata = this->checkMotorLimit(valueLeft);
	comedi_data_write(this->comediDev,1,0,0,0,ldata);

	//Send command to right motor (channel 1 in subdevice 1)
	lsampl_t rdata = this->checkMotorLimit(valueRight);
	comedi_data_write(this->comediDev,1,1,0,0,rdata);
}

void RugWarrior::moveForward(){
	this->moveStraight(this->veloStraight);
}

void RugWarrior::moveBackward(){
	this->moveStraight(-this->veloStraight);
}

void RugWarrior::turnLeft(){
	this->turn(this->veloRot);
}

void RugWarrior::turnRight(){
	this->turn(-this->veloRot);
}

void RugWarrior::moveStraight(int velo){
	this->isTurning = false;
	this->setMotors(2047 + velo + this->biasLeft, 2047 + velo + this->biasRight);
}

void RugWarrior::turn(int omega){
	this->isTurning = true;
	this->setMotors(2047 + omega, 2047 - omega);
}

lsampl_t RugWarrior::checkMotorLimit(double velo) {
	lsampl_t v;
	if (velo<=100)
		v = 100;
	else if (velo>=3995)
		v = 3995;
	else
		v = (lsampl_t)velo;
	return v;
}

}
