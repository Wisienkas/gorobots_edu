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
#include <utils/real_robots/dacbot/dacbot_serial.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string.h>

unsigned int triang = 0;
bool sw = true;
int abc=0;
int feedback[4];

int cntr=0;

int motcom[4];


namespace lpzrobots {

  dacbot_serial::dacbot_serial(const char *port)
  : AbstractRobot("dacbot_serial", "$Id: main.cpp,v 0.1 2011/14/07 18:00:00 fhesse $"),
    port(port) {

    std::cout<<"Opening serial port"<<std::endl;
    fd1=open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NDELAY);//make sure your account in PC can have access to serial port


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
      std::cout<<"Serial port open"<<std::endl;

      memset(&tio,0,sizeof(tio));
      tio.c_iflag=0;
      tio.c_oflag=0;
      tio.c_cflag=CS8|CREAD|CLOCAL;           // 8n1, see termios.h for more information
      tio.c_lflag=0;
      tio.c_cc[VMIN]=1;
      tio.c_cc[VTIME]=5;


      cfsetospeed(&tio,B115200);            // 115200 baud
      cfsetispeed(&tio,B115200);            // 115200 baud

      tcsetattr(fd1,TCSANOW,&tio);


      printf("port finish configuration \n");
    }
    comByte=2;
    end=0;
    index=0;
    sensor1=1;


    t=0;  // global step counter

    //Variables in dungBeetle_hindlegSensMotDef.h
    sensornumber = DUNGBEETLE_SENSOR_MAX;
    motornumber = DUNGBEETLE_MOTOR_MAX;

    //Setting Motors
    for (int t=0;t<33;t++)
    {
      serialPos[t]=128; //Setting all motor to middle position as initialization
    }

    alph = 1.5;//1.5;
    phi = 0.25;

    activityH1=0;
    activityH2=0;

    outputH1 = 0.01;
    outputH2 = 0.01;



    serialPlot.open("/home/dacbot/Documents/serialplot.dat");




  }


  dacbot_serial::~dacbot_serial(){


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
  int dacbot_serial::getSensors(sensor* sensors, int sensornumber){

    assert(sensornumber >= this->sensornumber);

    for(int i=0; i<=DUNGBEETLE_SENSOR_MAX;i++){
      sensors[i]=0;
    }

    char serial_msg[2]= {2,0};
    wr = write(fd1, serial_msg,sizeof(serial_msg));
    //std::cout<<"Request sent: "<<wr<<std::endl;

    //sleep(2);
    do{

      // --- Reading the potentiometer values
      for (int i=0;i<SENSOR_BUFFER_NUM;i++){
        do{
          rd = read(fd1, &chBuff, 1);
          if (rd){
            potValue[i]=(unsigned char)(chBuff);// potvalue are AMOS sensor data
            //std::cout<<"FB "<< i << ":" <<potValue[i]<<std::endl;
            if(chBuff==0){
              received=true;
              //break;
            }
          }
        }while(!rd);
      }
    }while(!received);// "0" is sync byte

    received=false;
    //std::cout<<"Loop ended!!"<<std::endl;output

    // LpzRobot <-- AMOS
    //Foot sensors (FS,Group 1)
    sensors[0]=potValue[0]; //HL
    sensors[1]=potValue[1]; //HR
    sensors[2]=potValue[2];	//KL
    sensors[3]=potValue[3]; //KR
    sensors[4]=0.5;			//BA
    potValue[4]-=1;
    potValue[5]-=1;
    sensors[5]=((-potValue[4]+1)+1)*2047;//FSL
    sensors[6]=((-potValue[5]+1)+1)*2047;//FSR

    /*std::cout<<"sensors 1 : "<<sensors[0]<<std::endl;
   	std::cout<<"sensors 2 : "<<sensors[1]<<std::endl;
	std::cout<<"sensors 3 : "<<sensors[2]<<std::endl;
   	std::cout<<"sensors 4 : "<<sensors[3]<<std::endl;
	std::cout<<"sensors 5 : "<<sensors[5]<<std::endl;
   	std::cout<<"sensors 6 : "<<sensors[6]<<std::endl;*/


    //Conversion to positive range [0,..,255]
    /*for(int i=0; i<=DUNGBEETLE_SENSOR_MAX;i++){
		if (sensors[i] < 0){
			sensors[i]+=256;
		}
	}*/


    bool default_preprocessing = true;//true
    if (default_preprocessing){
      processSensors(sensors);
    }

    //Your own,e.g.,
    bool giuliano_preprocessing = false;
    if (giuliano_preprocessing){
      processSensorsGiuliano(sensors);
    }


    return this->sensornumber;

  }

  /*Different sensors processing*/////// THIS ONE HAS TO BE SET UP BY SKRETCH
  void dacbot_serial::processSensors(sensor* psensors){

    //Need to ADJUST again 12.04.2012 max min range
    //Foot sensor (FS, Group 1): Scaling to 0 (off ground),..,1 (on ground)

    //psensors[TC0_RIGHT]= ((psensors[TC0_RIGHT]-38)/(70-38));   //[min = 7 (off ground), max =  207 (on ground)]
    //psensors[R_HIP_S]= (((psensors[R_HIP_S]-0)/(255))*90)-45;   //[min = 7 (off ground), max =  207 (on ground)]
    //((value-oldMin)/(oldMax-oldMin)) * (newMax-newMin) + newMin;
    //psensors[0]= ((psensors[0]-14)/(255-14))*(130-0)+0;
    //psensors[1]= ((psensors[1]-14)/(255-14))*(130-0)+0;
    psensors[0]= ((psensors[0]-8)/(155-8))*(135-1)+1;
    psensors[1]= ((psensors[1]-8)/(155-8))*(135-1)+1;

    if(psensors[0] < 0)
    {
      psensors[0]=0;
    }

    if(psensors[1] < 0)
    {
      psensors[1]=0;
    }

    psensors[2]= ((psensors[2]-82)/(245-82))*(180-65)+65;
    psensors[3]= ((psensors[3]-72)/(235-72))*(180-65)+65;

    for (int i=0; i<4; i++) {
      if (psensors[i] < 0) {
        psensors[i]=0;
      }
    }

    /*feedback[0]= psensors[0];
	feedback[1]= psensors[1];
	feedback[2]= psensors[2];
	feedback[3]= psensors[3];*/


    /*std::cout<<"RA HL:"<< psensors[0]<<std::endl;
	std::cout<<"RA HR:"<< psensors[1]<<std::endl;
	std::cout<<"RA KL:"<< psensors[2]<<std::endl;
	std::cout<<"RA KR:"<< psensors[3]<<std::endl;
	std::cout<<"RA LFS:"<< psensors[5]<<std::endl100;
	std::cout<<"RA RFS:"<< psensors[6]<<std::endl;*/


    //psensors[R_KNEE_S]= ((psensors[R_KNEE_S]-0)/(255))*90-45; //[min = 20 (off ground), max = 200 (on ground)]
    //psensors[L_KNEE_S]= ((psensors[L_KNEE_S]-0)/(255))*90-45; //[min = 20 (off ground), max = 200 (on ground)]


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

  void dacbot_serial::setMotors(const motor* motors, int motornumber){

    assert(motornumber >= this->motornumber);

    //std::cout<<"Setting motors"<<std::endl;

    // -------------------- initializing the Motor range ------------------------

    //Setting max and min values for the motors, where max is streched knee and forward lifted hip
    //HL
    /*	servoPosMin[0] = 130;//120;
	servoPosMax[0] = 215;//25;
	//HR
	servoPosMin[1] = 130;//160;
	servoPosMax[1] = 215;//80;
	//KL
	servoPosMin[2] = 124;//120;
	servoPosMax[2] = 15;//25;
	//KR
	servoPosMin[3] = 137;//160;
	servoPosMax[3] = 30;//80;*/








    WeightH1_H1  =  1.4;
    WeightH2_H2  =  1.4;
    WeightH1_H2  =  0.18+0.1;
    WeightH2_H1  = -0.18-0.1;


    BiasH1      = 0.0;
    BiasH2      = 0.0;

    activityH1 = WeightH1_H1*outputH1+WeightH1_H2*outputH2+BiasH1;
    activityH2 = WeightH2_H2*outputH2+WeightH2_H1*outputH1+BiasH2;

    outputH1 = tanh(activityH1);
    outputH2 = tanh(activityH2);





    /****************/



    printf("CPG %f %f\n",outputH1, outputH2);



    /*	if (outputH1>=0.5) {
			motcom[0]=4;
		} else if (outputH1<=-0.5) {
			motcom[0]=2;
		} else {
			motcom[0]=3;
		} */





    serialPlot<<outputH1<<' '<<motcom[0]<<' '<<endl;
    //std::cout<<"printing "<<std::endl;


    // ##################### move motors ################
    for(int i=0;i<4;i++)
    {
      motorCom[i] = motors[i];

      if (motorCom[i]>=0.5) {
        motcom[i]=4;
      } else if (motorCom[i]<=-0.5) {
        motcom[i]=2;
      } else {
        motcom[i]=3;
      }

      //std::cout<<"MC : "<< motors[i]<<std::endl;

    }





    /*std::cout<<"MC 1:"<< motcom[0]<<std::endl;
std::cout<<"MC 2:"<< motcom[1]<<std::endl;
std::cout<<"MC 3:"<< motcom[2]<<std::endl;
std::cout<<"MC 4:"<< motcom[3]<<std::endl;
     */

    //*******************************
    /*
//state 1
if (feedback[0]<= 90 && feedback[1]>= 90 && feedback[2]>= 170 && feedback[3]>= 170)  {
	motcom[0]= 1; //LH
	motcom[1]= 3; //RH
	motcom[2]= 1; //LK
	motcom[3]= 1; //RK
}

//state 2
if (feedback[0]<= 90 && feedback[1]>= 120 && feedback[2]>= 170 && feedback[3]>= 170){
	motcom[0]= 3; //LH
	motcom[1]= 1; //RH
	motcom[2]= 1; //LK
	motcom[3]= 1; //RK
}

//state 3
if (feedback[0]>= 120 && feedback[1]<= 90 && feedback[2]<= 90 && feedback[3]>= 170){
	motcom[0]= 1; //LH
	motcom[1]= 1; //RH
	motcom[2]= 3; //LK
	motcom[3]= 1; //RK
}

//state 4
if (feedback[0]<= 90 && feedback[1]<= 60 && feedback[2]>= 170 && feedback[3]<= 100){
	motcom[0]= 1; //LH
	motcom[1]= 3; //RH
	motcom[2]= 1; //LK
	motcom[3]= 3; //RK
}
     */
    //*******************************


    /*cntr++;
if (cntr <= 10) {
	motcom [0] = 2;
} else if ( (cntr > 10) && (cntr < 20)){
	motcom[0] = 4;
} else {
	cntr=0;
}*/

    //std::cout<<"cntr : "<< cntr<<std::endl;
    //std::cout<<"motcom : "<< motcom[0]<<std::endl;

    /*serialPos[0] = (int) (double)(((motorCom[0]+1.0)/2.0)*(servoPosMax[0]-servoPosMin[0])+servoPosMin[0]) ;
	serialPos[1] = (int) (double)(((motorCom[1]+1.0)/2.0)*(servoPosMax[1]-servoPosMin[1])+servoPosMin[1]) ;
	serialPos[2] = (int) (double)(((motorCom[2]+1.0)/2.0)*(servoPosMax[2]-servoPosMin[2])+servoPosMin[2]);
	serialPos[3] = (int) (double)(((motorCom[3]+1.0)/2.0)*(servoPosMax[3]-servoPosMin[3])+servoPosMin[3]) ;
     */

    // do some processing for motor commands before sending AMOS sensors
    char serial_motor2[34] = {1,motcom[0],motcom[1],motcom[2],motcom[3],121,121,121,120,120,120,120,120,120,120,120,120,120,120,120,120,120,120,5,5,5,5,5,5,5,5,5,5,0};
    /*sprintf(serial_motor, "%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c"
				,comByte,serialPos[1],serialPos[2],
				serialPos[3],serialPos[4],serialPos[5],serialPos[6],serialPos[7],serialPos[8],
				serialPos[9],serialPos[10],serialPos[11],serialPos[12],serialPos[13],serialPos[14],
				serialPos[15],serialPos[16],serialPos[17],serialPos[18],serialPos[19],serialPos[20],
				serialPos[21],serialPos[22],serialPos[23],serialPos[24],serialPos[25],serialPos[26],
				serialPos[27],serialPos[28],serialPos[29],serialPos[30],serialPos[31],serialPos[32],end);
     */
    //Sendding command to serial port
    int n = write(fd1, serial_motor2, sizeof(serial_motor2));
    usleep (10000);//10000);

    // increase time counter
    t++;


  }

  /*Process your sensor signals here to match to your need*/
  void dacbot_serial::processSensorsGiuliano(sensor* sensors){

  }


}