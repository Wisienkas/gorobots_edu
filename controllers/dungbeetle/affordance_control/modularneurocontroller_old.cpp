#include <math.h>
#include "modularneurocontroller.h"
#include "utils/delayline.cpp"
#include <controllers/dungbeetle/hind_leg_control/adaptivecpg/shiftregister.cpp>


using namespace matrix;
using namespace cv;

int dilation_size = 2;

bool less_by_x(const Point2d& lhs, const Point2d& rhs)
{
    return lhs.x < rhs.x;
}
//utility to draw outputs of the neurons
void modularNeuroController::updateGui(){


	if(!mul_cpgs){

	o0 = cpg->getCpgOut0();
	o1 = cpg->getCpgOut1();
	o2 = cpg->getCpgOut2();
	omega = cpg->getCpgFrequency();
	//training_error = aff->getError();
	
	}else if(mul_cpgs){

	R0_H0 = cpg->getCpgOut0(0);
	R0_H1 = cpg->getCpgOut1(0);
	R0_H2 = cpg->getCpgOut2(0);
	R0_Per = cpg->getP(0);

	R1_H0 = cpg->getCpgOut0(1);
	R1_H1 = cpg->getCpgOut1(1);
	R1_H2 = cpg->getCpgOut2(1);
	R1_Per = cpg->getP(1);

	R2_H0 =cpg->getCpgOut0(2);
	R2_H1 = cpg->getCpgOut1(2);
	R2_H2 = cpg->getCpgOut2(2);
	R2_Per = cpg->getP(2);

	L0_H0 = cpg->getCpgOut0(3);
    L0_H1 =cpg->getCpgOut1(3);
    L0_H2 =cpg->getCpgOut2(3);
    L0_Per =cpg->getP(3);

    L1_H0 = cpg->getCpgOut0(4);
	L1_H1 = cpg->getCpgOut1(4);
	L1_H2 = cpg->getCpgOut2(4);
	L1_Per = cpg->getP(4);

	L2_H0 = cpg->getCpgOut0(5);
	L2_H1 = cpg->getCpgOut1(5);
	L2_H2 = cpg->getCpgOut2(5);
	L2_Per = cpg->getP(5);
	

	omega0=cpg->getCpgFrequency(0);
	omega1=cpg->getCpgFrequency(1);
	omega2=cpg->getCpgFrequency(2);
	omega3=cpg->getCpgFrequency(3);
	omega4=cpg->getCpgFrequency(4);
	omega5=cpg->getCpgFrequency(5);
	}

}




modularNeuroController::modularNeuroController():AbstractController("modularNeuroController", "$Id: modularneurocontroller.cpp,v 0.1 $"){
	initialize(2,false,false);
}
//Constructor of the abstract controller
modularNeuroController::modularNeuroController(int dungBeetletype,bool mCPGs,bool mMuscleModelisEnabled):AbstractController("modularNeuroController", "$Id: modularneurocontroller.cpp,v 0.1 $"){
	initialize(dungBeetletype, mCPGs, mMuscleModelisEnabled);
	img = Mat(1000,1000, CV_8UC1,Scalar::all(255));
	aff = new affordanceController();
	aff->createAffordance();
	current=stop;
	pushed = true;
	//save = true;
	sensorAngle =  M_PI/180*90;
	d1 = 0;
	counter = 0;
	///Single CPG Modular neuaral controller
	if(!mCPGs){

	o0 = cpg->getCpgOut0();
	o1 = cpg->getCpgOut1();
	o2 = cpg->getCpgOut2();
	omega = cpg->getCpgFrequency();
	perturbation = cpg->getP();
	//training_error=aff->getError();
	addInspectableValue("RBFerror",&training_error,"RBFerror");
	addInspectableValue("frequency",&omega,"frequency");
    addInspectableValue("VRN_LEFT",&pattern2TC,"VRN_LEFT");
    addInspectableValue("VRN_RIGHT",&pattern1TC,"VRN_RIGHT");
    addInspectableValue("PSN_PATTERN1",&pattern1CT,"PSN_PATTERN1");
    addInspectableValue("PSN_PATTERN2",&pattern1FT,"PSN_PATTERN2");

    addInspectableValue("outputH0",&o0,"outputH0");
	addInspectableValue("outputH1",&o1,"outputH1");
    addInspectableValue("outputH2",&o2,"outputH2");
    addInspectableValue("perturbation",&perturbation,"perturbation");
	
	//Multiple CPG Modular Neural controller
	}else if(mCPGs){
	//low pass filter to clean the feedback signals of the different legs,cut off frequency has been set to 0.3,
	//setting it to a frequency bigger than 0.4 results in too much sensitive adaptive oscillator's response
	joint_R0 = new lowPass_filter(0.3);
	joint_R1 = new lowPass_filter(0.3);
	joint_R2 = new lowPass_filter(0.3);
	joint_L0 = new lowPass_filter(0.3);
	joint_L1 = new lowPass_filter(0.3);
	joint_L2 = new lowPass_filter(0.3);
	
	R0_H0 = cpg->getCpgOut0(0);
	R0_H1 = cpg->getCpgOut1(0);
	R0_H2 = cpg->getCpgOut2(0);
	R0_Per = cpg->getP(0);

	R1_H0 = cpg->getCpgOut0(1);
	R1_H1 = cpg->getCpgOut1(1);
	R1_H2 = cpg->getCpgOut2(1);
	R1_Per = cpg->getP(1);

	R2_H0 =cpg->getCpgOut0(2);
	R2_H1 = cpg->getCpgOut1(2);
	R2_H2 = cpg->getCpgOut2(2);
	R2_Per = cpg->getP(2);

	L0_H0 = cpg->getCpgOut0(3);
    L0_H1 =cpg->getCpgOut1(3);
    L0_H2 =cpg->getCpgOut2(3);
    L0_Per =cpg->getP(3);

    L1_H0 = cpg->getCpgOut0(4);
	L1_H1 = cpg->getCpgOut1(4);
	L1_H2 = cpg->getCpgOut2(4);
	L1_Per = cpg->getP(4);

	L2_H0 = cpg->getCpgOut0(5);
	L2_H1 = cpg->getCpgOut1(5);
	L2_H2 = cpg->getCpgOut2(5);
	L2_Per = cpg->getP(5);
	

	omega0=cpg->getCpgFrequency(0);
	omega1=cpg->getCpgFrequency(1);
	omega2=cpg->getCpgFrequency(2);
	omega3=cpg->getCpgFrequency(3);
	omega4=cpg->getCpgFrequency(4);
	omega5=cpg->getCpgFrequency(5);


	addInspectableValue("R0_outputH0",&R0_H0,"R0_outputH0");
	addInspectableValue("R0_outputH1",&R0_H1,"R0_outputH1");
    addInspectableValue("R0_outputH2",&R0_H2,"R0_outputH2");
    addInspectableValue("R0_perturbation",&R0_Per,"R0_perturbation");

    addInspectableValue("R1_outputH0",&R1_H0,"R1_outputH0");
	addInspectableValue("R1_outputH1",&R1_H1,"R1_ooutputH1");
    addInspectableValue("R1_outputH2",&R1_H2,"R1_outputH2");
    addInspectableValue("R1_perturbation",&R1_Per,"R1_perturbation");

 	addInspectableValue("R2_outputH0",&R2_H0,"R2_outputH0");
	addInspectableValue("R2_outputH1",&R2_H1,"R2_ooutputH1");
    addInspectableValue("R2_outputH2",&R2_H2,"R2_outputH2");
    addInspectableValue("R2_perturbation",&R2_Per,"R2_perturbation");

	addInspectableValue("L0_outputH0",&L0_H0,"L0_outputH0");
	addInspectableValue("L0_outputH1",&L0_H1,"L0_L0_ooutputH1");
    addInspectableValue("L0_outputH2",&L0_H2,"L0_outputH2");
    addInspectableValue("L0_perturbation",&L0_Per,"L0_perturbation");

    addInspectableValue("L1_outputH0",&L1_H0,"L1_outputH0");
	addInspectableValue("L1_outputH1",&L1_H1,"L1_ooutputH1");
    addInspectableValue("L1_outputH2",&L1_H2,"L1_outputH2");
    addInspectableValue("L1_perturbation",&L1_Per,"L1_perturbation");

	addInspectableValue("L2_outputH0",&L2_H0,"L2_outputH0");
	addInspectableValue("L2_outputH1",&L2_H1,"L2_ooutputH1");
    addInspectableValue("L2_outputH2",&L2_H2,"L2_outputH2");
    addInspectableValue("L2_perturbation",&L2_Per,"L2_perturbation");
     
    addInspectableValue("frequency_R0",&omega0,"frequency_R0");
    addInspectableValue("frequency_R1",&omega1,"frequency_R1");
    addInspectableValue("frequency_R2",&omega2,"frequency_R2");
    addInspectableValue("frequency_L0",&omega3,"frequency_L0");
    addInspectableValue("frequency_L1",&omega4,"frequency_L1");
    addInspectableValue("frequency_L2",&omega5,"frequency_L2");
     
	}
}

void modularNeuroController::initialize(int aAMOSversion,bool mCPGs,bool mMuscleModelisEnabled)
{	
	
	I_l = 0.0;
	I_r = 0.0;
	I3 = 0.0;
	t = 0;
	mul_cpgs=mCPGs;
	cpg = new ModularNeural(mCPGs,6);
	
}
//function to set the inputNeurons input according to the keyboard button,allows almost omnidirectional walking using the single CPG controller (amosII controller).
void getCommand(char key){
				switch(char(key)){
					case 'w':
					std::cout << "forward"<<std::endl;
					I_l = -1.0;
					I_r = -1.0;
					//I2 = 1.0;
					break;
					case 's':
					std::cout << "backward"<<std::endl;
					I_l = 1.0;
					I_r = 1.0;
					//I2 = 1.0;
					break;
					case 'q':
					std::cout << "left"<<std::endl;
					I_l = -1.0;
					I_r = 1.0;
					//I2 = 1.0;
					break;
					case 'e':
					std::cout << "right"<<std::endl;
					I_l = 1.0;
					I_r = -1.0;
					//I2 = 1.0;
					break;
					case 'z':
					std::cout << "stop"<<std::endl;
					I_l = 0.0;
					I_r = 0.0;
					//I2 = 1.0;
					break;					
					case 'a':
					std::cout << "I3=1.0"<<std::endl;
					I3 = 1.0;
					break;
					case 'd':
					std::cout << "I3=0.0"<<std::endl;
					I3 = 0.0;
					break;
					case 'i':
					std::cout << "I4=0.0 TF disinhibithed"<<std::endl;
					if(I2 == 1.0)
					I2 = 0.0;
					else if(I2 == 0.0)
					I2 = 1.0;	
					break;
				}
}

void modularNeuroController::lineSegmentation(){
	double a;
	double b;
	img = Scalar::all(250);

	for(int i = 122 ; i < 302 ; i++){


		
		b =round(((x[i]*sin(sensorAngle)-8)/-16)*1000) ;

		a=round(((x[i]*cos(sensorAngle))/8)*1000);
		beams.push_back(Point2d(a,b));
		if(x[i] > 3.8){
		img.at<uchar>(Point2d(a,b)) = 255;
		}else{
		

		img.at<uchar>(Point(a,b)) = 0;

		}
		sensorAngle += angleStep;
	}
	
		
		adaptiveThreshold(img, img, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 5, -2); 
		Mat element = getStructuringElement(1, Size(2,2));
		Mat element1 = getStructuringElement(1, Size(1,1));
		dilate(img,img,element,Point(-1,-1),2);
		erode(img,img,element,Point(-1,-1),2);
		blur( img, img, Size(5,5));
		erode(img,img,element,Point(1,1),5);
		cvtColor( img, color_dst, CV_GRAY2RGB );
		// Find contours
  findContours( img, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE, Point(0, 0) );
  vector<vector<Point> > contours_poly( contours.size() );
   contours_poly.resize(contours.size());
   vector<Rect> boundRect( contours_poly.size() );
   vector<Point2d> vertexl(contours_poly.size());
   vector<Point2d> vertexr(contours_poly.size());
   vector<Point2d> mc( contours_poly.size() );
  // /// Draw contours
  color_dst = Mat::zeros( img.size(), CV_8UC3 );

  for( int i = 0; i< contours.size(); i++ )
     {
  
       Scalar color(rand() % 255,rand() % 255,rand() % 255);
       approxPolyDP( Mat(contours[i]), contours_poly[i], 0.5, true );
       boundRect[i] = boundingRect( Mat(contours_poly[i]) );
   	   vertexl[i] =  boundRect[i].br();
   	   vertexl[i] += Point2d(0.0,-boundRect[i].height);
   	   vertexr[i] =  boundRect[i].br();
       mc[i] = boundRect[i].tl();
       mc[i] += Point2d(0.0,boundRect[i].height/2);


       if(boundRect[i].height>15){

       	
       		
       
       circle( color_dst, mc[i], 1, color, -1, 8, 0 );
       circle( color_dst, vertexl[i], 1, color, -1, 8, 0 );
       circle( color_dst,vertexr[i], 1, color, -1, 8, 0 );
       
       drawContours( color_dst, contours_poly, i, color, 1, 8, hierarchy, 0, Point() );
       for(int j = 0;j < beams.size(); j++){
       	

       				dx.push_back(Point2d(norm(vertexr[i]-beams[j]),j));
       				sx.push_back(Point2d(norm(vertexl[i]-beams[j]),j));
       				cen.push_back(Point2d(norm(mc[i]-beams[j]),j));


       }
       auto mmxr = std::minmax_element(dx.begin(), dx.end(), less_by_x);
       auto mmxc = std::minmax_element(cen.begin(), cen.end(), less_by_x);
       auto mmxl = std::minmax_element(sx.begin(), sx.end(), less_by_x);

       dl=x[122+int(mmxl.first->y)];
       dr=x[122+int(mmxr.first->y)];
       theta=(M_PI/180*90+mmxr.first->y*angleStep)-(M_PI/180*90+mmxl.first->y*angleStep);

      

       // cout <<"\n/---------------------------------------------------------------------------------------/\n"<<endl;
       // cout << " min norm left: " << mmxl.first->x << " beam: "<< beams[int(mmxl.first->y)] << " distance j: "<< x[122+int(mmxl.first->y)] << " distance j-1: " << x[122+int(mmxl.first->y)-1] << endl;
       // cout << " min norm center: " << mmxc.first->x << " beam: "<< beams[int(mmxc.first->y)] << " distance j: "<< x[122+int(mmxc.first->y)] << endl;
       // cout << " min norm right: " << mmxr.first->x << " beam: "<< beams[int(mmxr.first->y)] << " distance j: "<< x[122+int(mmxr.first->y)] << " distance j+1: " << x[122+int(mmxr.first->y)+1] << endl;
       // cout<< " left: "<< vertexl[i] <<" center: "<<mc[i] <<" right: "<<vertexr[i] <<endl;
       // cout<< " angle_l: "<< (M_PI/180*90+mmxl.first->y*angleStep)*180/M_PI << " angle_r: " <<(M_PI/180*90+mmxr.first->y*angleStep)*180/M_PI<<endl;
       // cout << " SIZE: " << sqrt(pow(dl,2)+pow(dr,2)-2*dl*dr*cos(theta))<< endl;

		int leftmost,rightmost,currentbeam,nbeams,step,pointapprox;
		pointapprox=90;
		nbeams = (122+int(mmxr.first->y))-(122+int(mmxl.first->y));
		step = (nbeams)/pointapprox;
		leftmost = 122+int(mmxl.first->y);
		currentbeam = leftmost;
		rightmost = 122+int(mmxr.first->y);
		vector<double> points_shape;
		
		for(int i= (211-pointapprox/2);i<(211+pointapprox/2);i++){
			
			points_shape.push_back(x[i]);

		}
	if(counter == 50){	
		vector<double> test_in;
		counter = 0;
		 	for(int j = 0 ; j < points_shape.size() ; j++){
		 //cout <<" i: "<<j<<" dist: "<<round(abs(points_shape[j]-x[211])*1000)/1000<<endl;
	 	//cout <<" i: "<<j<< "norm :" << round((points_shape[j]/norm(points_shape))*1000)/1000<<endl;}
		//  	//cout <<" i: "<<j<< "norm :" << points_shape[j]<<endl;}
		 	test_in.push_back(round(abs(points_shape[j]-x[211])*1000)/1000);
	

		   }



	 // 	   aff->test_in.clear();
		// cout << " leftmost: " << x[leftmost] <<endl;
		// cout << " beams" <<nbeams<< endl;
		// cout << " step " <<step<< endl;
		// cout << " rightmost: " << x[rightmost]<<endl;
		// cout << " angle: "<< -theta/M_PI*180/pointapprox  <<endl;
	   vector<double> lvl1_in;
	   vector<double> lvl1_out;
	   size = floor(sqrt(pow(dl,2)+pow(dr,2)-2*dl*dr*cos(theta))*100+0.5)/100;
	   if(!detected && size > 0.0){
	   cout << "size:"<<size<<endl;
	   if(cylinder_object){
	   	lvl1_in.push_back(size);
	   lvl1_in.push_back(0.5);
	   lvl1_out = aff->objects[0]->aff_model->get_output(lvl1_in);
	   }
	   if(box_object){
	   lvl1_in.push_back(size);
	   lvl1_in.push_back(0.7);
	   lvl1_out = aff->objects[1]->aff_model->get_output(lvl1_in);
		}
	   if(sphere_object){
	   lvl1_in.push_back(size);	
	   lvl1_in.push_back(0.7);
	   lvl1_out = aff->objects[2]->aff_model->get_output(lvl1_in);
		}
		
		test_in.push_back(lvl1_out[0]);
				vector<double> out;
		   out = aff->shape_classification->get_output(test_in);
	 	   cout << " output: " << round(out[0])<<endl;
	 	   mode = round(out[0]);
	 	   if(mode == 1 || mode == 0 || mode == -1)
	 	   detected = true;
	 	 }
}	   
	   counter ++;	
	   points_shape.clear();
       dx.clear();
       sx.clear();
       cen.clear();
       beams.clear();

          	   
   		}
   	}
     
    
	sensorAngle = M_PI/180*90;
}


void modularNeuroController::init(int sensornumber, int motornumber, RandGen* randGen) {
	numbersensors = sensornumber;
  	numbermotors = motornumber;
  	x.resize(sensornumber);
  //y.resize(AMOSII_MOTOR_MAX);

}

//implement controller here
void modularNeuroController::step(const sensor* x_, int number_sensors, motor* y_, int number_motors){
		


	
	assert(number_sensors == numbersensors);
  	assert(number_motors == numbermotors);

  //0) Sensor inputs/scaling  ----------------


  	for (unsigned int i = 0; i < AMOSII_SENSOR_MAX; i++) {
    	x.at(i) = x_[i];
  	}


  	//Locomotion control Single CPG
  	if(!mul_cpgs)
  	{
	//update neurons output for visualization in the GUI
  		lineSegmentation();
  		updateGui();

  		cpg->setInputNeuronInput(I_l,I_r,I3);
		cpg->update();
	
	//CT Signal 
		pattern1CT=cpg->getPsnOutput(11);
		pattern2CT= -pattern1CT;
		pattern3CT= pattern1CT;
		pattern4CT= -pattern1CT;

	//FT Signal
		pattern1FT=cpg->getPsnOutput(10);
		pattern2FT=-pattern1FT;

	//TC Signal
  		pattern1TC=cpg->getVrnRightOut();
		pattern2TC=cpg->getVrnLeftOut();

		if(I2==1.0){
		
		if(detected && !turning){
			I_l = -1.0;
			I_r = 1.0;
			I2 = 0.0;
			turning = true;
		}
	  // //left back
	  // 		y_[5]=0;//(pattern2CT);
	  // 		y_[11]=0;//(-pattern2TC)*0.1;
	  // 		y_[17]=0;//0;//-pattern1FT+0.4;

	  // //right back
	  // 		y_[2]=0;//(pattern1TC-0.2);
	  // 		y_[8]=0;//(pattern1CT);
	  // 		y_[14]=0;//pattern1FT+0.4;

	  // //left middle
	  // 		y_[4]=0;//(pattern2TC-0.2);
	  // 		y_[10]=0;//(pattern1CT);
	  // 		y_[16]=0;//pattern1FT+0.4;

	  // //right middle
	  // 		y_[1]=0;//(pattern2CT);
	  // 		y_[7]=0;//(-pattern1TC)*0.5;
	  // 		y_[13]=0;//-pattern1FT+0.4;

	  		  		  //top left
	  		y_[3]=0;//(pattern2TC+0.1)*0.3;
	  		y_[9]=-0.1;//-0.1+(pattern2CT-0.05)*0.6;
	  		y_[15]=0;//-pattern1FT+0.4;

	  //top right
	  		y_[0]=0;//(0;//-pattern1TC+0.1)*0.3;
	  		y_[6]=-0.1;//-0.1+(pattern1CT-0.05)*0.6;
	  		y_[12]=0;//pattern1FT+0.4;

	  		//top left
	  		y_[3]=0;
	  		y_[15]=0;
			//top right
	  		y_[0]=0;
	  		y_[12]=0;

	  		y_[5]=0;
	  		y_[11]=0;
	  		y_[17]=0;//0;//-pattern1FT+0.4;

	  		//right back
	  		y_[2]=0;
	  		y_[8]=0;
	  		y_[14]=0;//pattern1FT+0.4;

	  		//left middle
	  		y_[4]=0;
	  		y_[10]=0;
	  		y_[16]=0;//pattern1FT+0.4;

	  		//right middle
	  		y_[1]=0;
	  		y_[7]=0;
	  		y_[13]=0;//-pattern1FT+0.4;

	  		//TO DO optimal distance for each object for push

	  		switch(current){
	  			case stop:
	  			if(!pushed){
	  				if(floor(x_[211]*100)/100 <= 0.15){
	  					cout << floor(x_[211]*100)/100 <<endl;
	  					cout << "PUSH!" << endl;
	  					ds=0;
	  					current=zero;
	  					//d1=0.0001;
	  					if(previous == five){
	  						cout << "pushing again"<<endl;
	  						current=four;
	  					}	
	  					
	  				}
	  			}else
	  			 	//sim_flag = true;
	  				break;
	  			case zero:
	  					cout << "0" << endl;
	  					//left up
	  					y_[4]=-1;
	  					//right up;
	  					y_[1]=-1;
	  					current=one;
	  			
	  				break;
				case one:
					cout << "1" << endl;
					//left forward
	  				y_[10]=1;
	  				//right forward;
	  				y_[7]=1;

	  			current=two;
	  			break;
	  			case two:
	  				cout << "2" << endl;
	  				//left down
	  				y_[4]=0.5;
	  				//right down;
	  				y_[1]=0.5;

	  				current=three;
	  			break;
	  			case three:
	  				cout << "3" << endl;
	  				//left down
	  				y_[10]=0.4;
	  				//right down;
	  				y_[7]=0.4;

	  				current=four;
	  			break;
	  			case four:
	  				d0=floor(x_[211]*100)/100 ;
	  				cout << "4" << endl;
	  				//top left
	  				y_[3]=0.8;
	  				y_[15]=1;
					//top right
	  				y_[0]=0.8;
	  				y_[12]=1;
	  				current=five;
				break;
					case five:
						//cout << " d0-d1 "<< abs(floor(x_[211]*100)/100-d0) <<endl;
						if(abs(floor(x_[211]*100)/100-d0) <= 0.01) {
							//cout << "5" << endl;
	  						
	  						//cout << " size: " << size << endl;
	  						tc0=floor(x_[0]*100)/100;
	  						
	  						}else{
	  							
	  							d1 = floor(x_[211]*100)/100;
	  							//cout << " d1 " << floor(x_[211]*100)/100 << endmesl;
	  							save = true;
	  							
	  							//top left
	  							y_[3]=0;
	  							y_[15]=0;
								//top right
	  							y_[0]=0;
	  							y_[12]=0;

	  							if(ds != d1 && timesteps != 50){	
	  								
	  								timesteps++;
	  								ds = d1;

								}else{
									if(timesteps>=9)
										cout <<"done"<<endl;
									size = floor(sqrt(pow(dl,2)+pow(dr,2)-2*dl*dr*cos(theta))*100+0.5)/100;
									cout << " size: " << size << endl;
									cout << "OK" << endl;
									cout << " TC0: " << tc0 << endl;
									cout << " ds " << ds << endl;
									cout << " d0 " << d0 << endl;
									file.open("/home/michelangelo/ball_dataset_v2.txt", ios::out | ios::app );
									if (file.is_open()){
										file << size <<" "<< tc0 <<" "<< ds-d0 << endl;
										file.close();
									}else
										cerr << "couldn't open the file"<<endl;
									
	  								//  	if(counter <= 7000){
		 								// counter++;
		 								// 	sim_flag = true;}
									pushed = true;
									timesteps = 0;
									//save = false;
	  								current=stop;
	  								previous = five;
								}
	  						
	  						}
				break;

	  			default:
	  			break;

	  	}


	  								//  	if(counter < 3002)
		 								// counter++;
		 								// else{
		 								// 	counter = 0; 
		 								// 	sim_flag = true;}

  		imshow("laser",color_dst);
		//waitKey(0);
	  	
	  //backbone joint
	    	y_[18]=0;
	    }
	    else{
	    	
	    	if(x[118] >= 0.99){
	    		cout << "stop"<<endl;
	    		I2 = 1.0;
	    		transporting=true;}
	    		  //left back
	  		y_[5]=(pattern2CT+0.2)*0.4;
	  		y_[11]=-(pattern2TC+0.3)*0.6;
	  		y_[17]=0;//0;//-pattern1FT+0.4;

	  //right back
	  		y_[2]=(pattern1CT-0.2)*0.4;
	  		y_[8]=(pattern1TC+0.3)*0.6;
	  		y_[14]=0;//pattern1FT+0.4;

	  //left middle
	  		y_[4]=(pattern1CT-0.2)*0.3;
	  		y_[10]=(pattern2TC+0.3)*0.6;
	  		y_[16]=0;//pattern1FT+0.4;

	  //right middle
	  		y_[1]=(pattern2CT+0.2)*0.3;
	  		y_[7]=-(pattern1TC+0.3)*0.6;
	  		y_[13]=0;//-pattern1FT+0.4;

	  //top left
	  		y_[3]=-(pattern2TC+0.1)*0.3;
	  		y_[9]=(pattern1CT-0.05)*0.6;
	  		y_[15]=0;//-pattern1FT+0.4;

	  //top right
	  		y_[0]=(pattern1TC+0.1)*0.3;
	  		y_[6]=(pattern2CT+0.05)*0.6;
	  		y_[12]=0;//pattern1FT+0.4;

	  //backbone joint
	    	y_[18]=0;
	    }
	}else if(mul_cpgs)
	{	
	//filtering of the feedback signal
	feedback0 =joint_R0->update(x.at(0));
    feedback1 =joint_R1->update(x.at(7));
    feedback2 =joint_R2->update(x.at(8));
    
    feedback3 =joint_L0->update(x.at(3));
    feedback4 =joint_L1->update(x.at(10));
    feedback5 =joint_L2->update(x.at(11));
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  	//input perturbation to the adaptive oscillator the number is the ID of the single CPGs
  	//front right ID 0
  	//middle right ID 1
  	//rear right ID 2
  	//front left ID 3
  	//middle left ID 4
  	//rear left ID 5
  	//the feedback corresponds to the filtered signal from the CT angle sensor for middle/hind legs and TC angle sensor for front legs.
  	//Infact for the given dungbeetle simulation,the CT joint is responsible for forward/backward movement (middle/hind legs).
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	cpg->update(feedback0,0);    
	cpg->update(feedback1,1);
	cpg->update(feedback2,2);    
	cpg->update(feedback3,3);
	cpg->update(feedback4,4);    
	cpg->update(feedback5,5);
	
	  //left back
	  		y_[5]=0;//(cpg->getPsnOutput(5,10)*1.2)+0.25;
	  		y_[11]=0;//cpg->getCpgOut1(5);
	  		y_[17]=0;//-cpg->getCpgOut1(5);

	  //right back
	  		y_[2]=0;//cpg->getPsnOutput(2,10)*1.2+0.25;
	  		y_[8]=0;//cpg->getCpgOut1(2);
	  		y_[14]=0;//-cpg->getCpgOut1(2);
	 
	  //left middle
	  		y_[4]=0;//cpg->getPsnOutput(4,10)*1.2+0.25;
	  		y_[10]=0;//cpg->getCpgOut1(4);
	  		y_[16]=0;//-cpg->getCpgOut1(4);

	  //right middle
	  		y_[1]=0;//cpg->getPsnOutput(1,10)*1.2+0.25;
	  		y_[7]=0;//cpg->getCpgOut1(1);
	  		y_[13]=0;//-cpg->getCpgOut1(1);

	  //top left
	  		y_[3]=0;//cpg->getCpgOut1(3);
	  		y_[9]=0;//-cpg->getPsnOutput(3,10)*1.2-0.20;
	  		y_[15]=0;//-cpg->getCpgOut1(3);

	  //top right
	  		y_[0]=0;//cpg->getCpgOut1(0);
	  		y_[6]=0;//-cpg->getPsnOutput(0,10)*1.2-0.20;
	  		y_[12]=0;//-cpg->getCpgOut1(0);

// 	  //backbone joint
	    	y_[18]=0;

	    	lineSegmentation();
	    	updateGui();
	    	imshow("laser",color_dst);
			//waitKey(0);
		}  	
  	/***Don't touch****Set Motor outputs begin *******************/
  	for(unsigned int j=0; j<y_MCPGs.size();j++)
  		for(unsigned k=0; k< 3; k++)//index of angel joints
  	 		y_[j+6*k] = y_MCPGs.at(j).at(j+6*k);

	
  	t++;
}

/** stores the controller values to a given file. */
bool modularNeuroController::store(FILE* f) const {
  return true;
}

/** loads the controller values from a given file. */
bool modularNeuroController::restore(FILE* f) {
  //	Configurable::parse(f);
  return true;
}
