/***************************************************************************
 *   Copyright (C) 2012 by Robot Group Goettingen                          *
 *                                    									   *
 *    fhesse@physik3.gwdg.de     			                               *
 *    xiong@physik3.gwdg.de                  	                           *
 *    poramate@physik3.gwdg.de                                             *
 *   LICENSE:                                                              *
 *   This work is licensed under the Creative Commons                      *
 *   Attribution-NonCommercial-ShareAlike 2.5 License. To view a copy of   *
 *   this license, visit http://creativecommons.org/licenses/by-nc-sa/2.5/ *
 *   or send a letter to Creative Commons, 543 Howard Street, 5th Floor,   *
 *   San Francisco, California, 94105, USA.                                *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                  *
 *                                                                         *
 *   AMOSII v2 has now only 18 sensors                                     *
 ***************************************************************************/

// include header file
#include <utils/real_robots/cdbot/cdbotSerial.h>




namespace lpzrobots {

cdbotSerial::cdbotSerial(const char *port)
: AbstractRobot("cdbotSerial", "$Id: main.cpp,v 0.1 2011/14/07 18:00:00 fhesse $"),
  port(port) {

	fd1=open(port, O_RDWR | O_NOCTTY | O_NDELAY);//make sure your account in PC can have access to serial port


	if(fd1 == -1)
	{
		std::cout<<std::endl<<"unable to open"<<port<<std::endl;
		std::cout<<"check provided port (hint: \"/dev/ttyS0\" or \"/dev/ttyUSB0\" required?)"<<std::endl;
		assert (fd1 != -1);
	}
	else
	{
		fcntl(fd1, F_SETFL, 0);
		printf("port is open");

		memset(&tio,0,sizeof(tio));
		tio.c_iflag=0;
		tio.c_oflag=0;
		tio.c_cflag=CS8|CREAD|CLOCAL;           // 8n1, see termios.h for more information
		tio.c_lflag=0;
		tio.c_cc[VMIN]=1;
		tio.c_cc[VTIME]=5;


		cfsetospeed(&tio,B57600);            // 57600 baud
		cfsetispeed(&tio,B57600);            // 57600 baud

		tcsetattr(fd1,TCSANOW,&tio);


		printf("port finish configuration \n");
	}
	comByte=2;
	end=0;
	index=0;
	sensor1=1;


	t=0;  // global step counter

	sensornumber = CDBOT_SENSOR_MAX;
	motornumber = CDBOT_MOTOR_MAX;

	//Setting Motors
	for (int t=0;t<33;t++)
	{
		serialPos[t]=128; //Setting all motor to middle position as initialization
	}

}


cdbotSerial::~cdbotSerial(){


	//Close COM0
	if(fd1 != -1){
		close(fd1);
		std::cout<<"closed the serial communication port"<<std::endl;
	}
}

// robot interface
/** returns actual sensorvalues
  @param sensors sensors scaled to [-1,1]
  @param sensornumber length of the sensor array
  @return number of actually written sensors
 */
int cdbotSerial::getSensors(sensor* sensors, int sensornumber){

	assert(sensornumber >= this->sensornumber);

	for(int i=0; i<=CDBOT_SENSOR_MAX;i++){
		sensors[i]=0;
	}

	comByte=2;
	end=0;

	char serial_msg[COMMAND_BUFFER_NUM];
	do{

		//Sending "getSensors" command to the board
		sprintf(serial_msg, "%c%c",comByte,end);

		wr = write(fd1, serial_msg,sizeof(serial_msg));

		// --- Reading the potentiometer values
		for (int i=1;i<SENSOR_BUFFER_NUM;i++){
			do{

				rd = read(fd1, &chBuff, 1);
				if (rd){
					potValue[i]=(unsigned char)(chBuff);// potvalue are AMOS sensor data
				}
			}while(!rd);

		}

	}while((unsigned char)(chBuff)!=0);// "0" is sync byte


	// LpzRobot <-- AMOS
	//Foot sensors (FS,Group 1)
	sensors[IR_RIGHT1]=potValue[IR_RIGHT1_REAL]; //[min = 7 (off ground), max =  207 (touch ground)]
	sensors[IR_RIGHT2]=potValue[IR_RIGHT2_REAL]; //[min = 15 (off ground), max = 196 (touch ground)]
	sensors[IR_LEFT1]=potValue[IR_LEFT1_REAL]; //[min = 20 (off ground), max = 200 (touch ground)]
	sensors[IR_LEFT2]=potValue[IR_LEFT2_REAL]; //[min = 20 (off ground), max = 200 (touch ground)]
	sensors[LIGHT_RIGHT]=potValue[LIGHT_RIGHT_REAL]; //[min = 7 (off ground), max =  207 (touch ground)]
	sensors[LIGHT_LEFT]=potValue[LIGHT_LEFT_REAL]; //[min = 15 (off ground), max = 196 (touch ground)]
	sensors[SOUND_RIGHT]=potValue[SOUND_RIGHT_REAL]; //[min = 20 (off ground), max = 200 (touch ground)]
	sensors[SOUND_LEFT]=potValue[SOUND_LEFT_REAL]; //[min = 20 (off ground), max = 200 (touch ground)]




	//Conversion to positive range [0,..,255]
	for(int i=0; i<=CDBOT_SENSOR_MAX;i++){
		if (sensors[i] < 0){
			sensors[i]+=256;
		}
	}


	bool default_preprocessing = true;
	if (default_preprocessing){
		processSensors(sensors);
	}

	return this->sensornumber;

}

/*Different sensors processing*/////// THIS ONE HAS TO BE SET UP BY SKRETCH
void cdbotSerial::processSensors(sensor* psensors){



	psensors[IR_RIGHT1]= ((psensors[IR_RIGHT1]-7)/(207-7));
	psensors[IR_RIGHT2]=((psensors[IR_RIGHT2]-7)/(207-7));
	psensors[IR_LEFT1]=((psensors[IR_LEFT1]-7)/(207-7));
	psensors[IR_LEFT2]=((psensors[IR_LEFT2]-7)/(207-7));
	psensors[LIGHT_RIGHT]=((psensors[LIGHT_RIGHT]-7)/(207-7));
	psensors[LIGHT_LEFT]=((psensors[LIGHT_LEFT]-7)/(207-7));
	psensors[SOUND_RIGHT]=((psensors[SOUND_RIGHT]-7)/(207-7));
	psensors[SOUND_LEFT]=((psensors[SOUND_LEFT]-7)/(207-7));


	/*
	if(psensors[TC0_RIGHT]>1)
		psensors[TC0_RIGHT] = 1;
	if(psensors[TC0_RIGHT]<0)
		psensors[TC0_RIGHT] = 0;

	if(psensors[CT0_RIGHT]>1)
		psensors[CT0_RIGHT] = 1;
	if(psensors[CT0_RIGHT]<0)
		psensors[CT0_RIGHT] = 0;

	if(psensors[FT0_RIGHT]>1)
		psensors[FT0_RIGHT] = 1;
	if(psensors[FT0_RIGHT]<0)
		psensors[FT0_RIGHT] = 0;

	 */


}




/** sets actual motorcommands
  @param motors motors scaled to [-1,1]
  @param motornumber length of the motor array
 */
void cdbotSerial::setMotors(const motor* motors, int motornumber){

	assert(motornumber >= this->motornumber);


	end =0; // Null as Sync-byte
	comByte=1;


	// -------------------- initializing the Motor range ------------------------



	// ##################### move motors ################
	for(int i=0;i<CDBOT_MOTOR_MAX;i++)
	{
		motorCom[i] = motors[i];// set LpzMotor value before processing and sending ??????????what is this?

		if (motorCom[i]>1) motorCom[i]=1;
		if (motorCom[i]<-1) motorCom[i]=-1;
	}

	motorCom[0] = 1;
	         motorCom[1]=1;

	servoPosMax[22] = 250;
	servoPosMin[22] = 130;

	servoPosMax[21] = 250;
	servoPosMin[21] = 130;

	serialPos[22] = 10;//(int) (double)(((motorCom[0]+1.0)/2.0)*(servoPosMax[22]-servoPosMin[22])+servoPosMin[22]) ; //Left
	serialPos[21] = 10;//(int) (double)(((motorCom[1]+1.0)/2.0)*(servoPosMax[21]-servoPosMin[21])+servoPosMin[21]) ; //Right

//	serialPos[20] =250;//(int) (double)(((motorCom[0]+1.0)/2.0)*(servoPosMax[22]-servoPosMin[22])+servoPosMin[22]) ; //Left
//	serialPos[23] =250;//(int) (double)(((motorCom[1]+1.0)/2.0)*(servoPosMax[21]-servoPosMin[21])+servoPosMin[21]) ; //Right



	//usleep(1000);
	//usleep (10000);//10000);
	// do some processing for motor commands before sending AMOS sensors

	sprintf(serial_motor, "%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c"
			,comByte,serialPos[1],serialPos[2],
			serialPos[3],serialPos[4],serialPos[5],serialPos[6],serialPos[7],serialPos[8],
			serialPos[9],serialPos[10],serialPos[11],serialPos[12],serialPos[13],serialPos[14],
			serialPos[15],serialPos[16],serialPos[17],serialPos[18],serialPos[19],serialPos[20],
			serialPos[21],serialPos[22],serialPos[23],serialPos[24],serialPos[25],serialPos[26],
			serialPos[27],serialPos[28],serialPos[29],serialPos[30],serialPos[31],serialPos[32],end);

	//Sendding command to serial port
	write(fd1, serial_motor, 34);//sizeof(serial_msg));

	// to slow down process a bit
	//usleep(10000);

	//usleep (10000);//10000);

	// increase time counter
	t++;

}
}



