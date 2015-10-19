/*
 * main.cpp
 *
 *  Created on: Oct 18, 2015
 *      Author: giuliano
 */
#include "getColorTrajectory.h"

int main(int argc, char* argv[])
{

  vector<int> line1, line2, line3;
  string path1,path2,path3;
  VideoCapture cap("ExampleVideo.mp4"); // open the video file for reading
  //VideoCapture cap("videoBeetle2.mp4"); // open the video file for reading
  ofstream color1Trajectory,color2Trajectory,color3Trajectory;
  getColorTrajectory color1, color2,color3;

  //SETTINGS//

  Size outputFrameSize(750,450);
  //output frame size
  //choose the size of your image. This is very important.
  //if you set a small size, the contours detected will be too small
  //and therefore the centroids can't be computed.

  bool showVideo = false;
  //show video
  //the video that you updated will be shown
  //even if nothing is detected the video will still be shown


  bool trajectoriesInOneFrame = true;
  //trajectories in one frame
  //false: each trajectory will be in one individual window
  //true:  each trajectory will be in one individual window, one additional window will
  //show all the trajectories together

  bool printTrajectoriesToFiles=false;
  // print trajectories to file
  // false: trajectories will not be saved
  // true: trajectories will be saved in a file, you can set the path

  //choose the path where you want your trajectories to be saved
  path1="";
  path2="";
  path3="";

  //which colors you want to detect???
  //here you need to add minimum and maximum HSV values.
  //In order to get these values do the following:
  //Donwload gimp for Ubuntu, and open the toolbox
  //Select a color in the color box
  //The color will have an H value=x
  //then your minimum and max value for H will be (x/2)-10 and (x/2)+10 respectively
  //the other values are not VERY important, is ok to tune them manually..
  //EXAMPLE..H value RED in Gimp: 360
  //minH=(360/2)-10=170 maxH=(360/2)+10=190
  color1.setColor(170,100,160,190,255,255);//red
  color2.setColor(15,100,100,40,255,255);//yellow
  color3.setColor(110,100,100,130,255,255);//blue
  //color3.setColor(110,100,100,130,255,255);//green



  //Which color you want to use for the trajectory?
  //Of course the best would be to use the same color as the detected one
  //but you can change it if you want. Here you have to useRGB values
  //you can use Gimp agin to detecg RGB values.
  //PLEASE NOTE..in OpenCV a color is defined as Scalar(a,b,c)
  //where a=Blue b=Green c=Red
  //Example Red in Gimp = 255 0 0
  //Red in Opencv Scalar(0,0,255).. use this definition to set your color line
  color1.setLineColor(0,0,255);//red
  color2.setLineColor(0,255,255);//yellow
  color3.setLineColor(255,0,0);//blue
  //color3.setLineColor(0,255,0);//green

  //SETTINGS

  if(printTrajectoriesToFiles==true)
  {
    color1Trajectory.open(path1.c_str());
    color2Trajectory.open(path2.c_str());
    color3Trajectory.open(path3.c_str());
  }

  if ( !cap.isOpened() )
  {
    cout << "Cannot open the video file" << endl;
    return -1;
  }

  double fps = cap.get(CV_CAP_PROP_FPS); //get the frames per seconds of the video

  while(1)
  {

    Mat frameColor1,frameColor2,frameColor3, frame, totalFrame, showVideoFrame;

    bool bSuccess = cap.read(frame); // read a new frame from video

    if (!bSuccess) //if not success, break loop
    {
      cout << "Cannot read the frame from video file" << endl;
      break;
    }

    resize(frame, frame, outputFrameSize, 0, 0);//set image size

    frameColor1=frame.clone();
    frameColor2=frame.clone();
    frameColor3=frame.clone();
    totalFrame=frame.clone();
    showVideoFrame=frame.clone();

    //Drawing trajectories in individual frames
    color1.detectTrajectory(frameColor1);
    color1.drawTrajectory(color1.getLineColor().at(0),color1.getLineColor().at(1),color1.getLineColor().at(2));

    color2.detectTrajectory(frameColor2);
    color2.drawTrajectory(color2.getLineColor().at(0),color2.getLineColor().at(1),color2.getLineColor().at(2));

    color3.detectTrajectory(frameColor3);
    color3.drawTrajectory(color3.getLineColor().at(0),color3.getLineColor().at(1),color3.getLineColor().at(2));


    //Drawing trajectories in one frame
    if(color1.getTrajectory().size()>=2)
    {
      for(int i=1;i<color1.getTrajectory().size();i++)
        line(totalFrame, color1.getTrajectory().at(i-1),color1.getTrajectory().at(i), Scalar(color1.getLineColor().at(0),color1.getLineColor().at(1),color1.getLineColor().at(2)), 3, 8, 0);
    }

    if(color2.getTrajectory().size()>=2)
    {
      for(int i=1;i<color2.getTrajectory().size();i++)
        line(totalFrame, color2.getTrajectory().at(i-1),color2.getTrajectory().at(i), Scalar(color2.getLineColor().at(0),color2.getLineColor().at(1),color2.getLineColor().at(2)), 3, 8, 0);
    }

    if(color3.getTrajectory().size()>=2)
    {
      for(int i=1;i<color3.getTrajectory().size();i++)
        line(totalFrame, color3.getTrajectory().at(i-1),color3.getTrajectory().at(i), Scalar(color3.getLineColor().at(0),color3.getLineColor().at(1),color3.getLineColor().at(2)), 3, 8, 0);
    }




    //

    if(trajectoriesInOneFrame==true)
      imshow("Trajectories", totalFrame);

    if(color1.getTrajectory().size()> 0)
      imshow("Color1", color1.getFrame());

    if(color2.getTrajectory().size()> 0)
      imshow("Color2", color2.getFrame());

    if(color3.getTrajectory().size()> 0)
      imshow("Color3", color3.getFrame());

    if(showVideo==true)
      imshow("Video", showVideoFrame);

    if(waitKey(30) == 27) //wait for 'esc' key press for 30 ms. If 'esc' key is pressed, break loop
    {
      cout << "esc key is pressed by user" << endl;
      break;
    }
  }

  if(printTrajectoriesToFiles=true)
  {
    for(int i=0;i<color1.getTrajectory().size();i++)
      color1Trajectory << color1.getTrajectory().at(i).x << " "<< color1.getTrajectory().at(i).y << endl;
    color1Trajectory.close();

    for(int i=0;i<color2.getTrajectory().size();i++)
      color2Trajectory << color2.getTrajectory().at(i).x << " "<<color2.getTrajectory().at(i).y << endl;
    color2Trajectory.close();
    for(int i=0;i<color3.getTrajectory().size();i++)
      color3Trajectory << color3.getTrajectory().at(i).x << " "<< color3.getTrajectory().at(i).y << endl;
    color3Trajectory.close();
  }


  return 0;

}




