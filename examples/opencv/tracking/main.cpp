
//Giuliano Di Canio
//giuliano.dicanio@gmail.com

#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <fstream>



using namespace cv;
using namespace std;

vector<Point2f> trajectory;

int main(int argc, char* argv[])
{

  ofstream myfilex,myfiley;

  myfilex.open("trajectoryx.txt");
  myfiley.open("trajectoryy.txt");

  VideoCapture cap("videoBeetle.mp4"); // open the video file for reading

  if ( !cap.isOpened() )  // if not success, exit program
  {
    cout << "Cannot open the video file" << endl;
    return -1;
  }


  //cap.set(CV_CAP_PROP_POS_MSEC, 300); //start the video at 300ms

  double fps = cap.get(CV_CAP_PROP_FPS); //get the frames per seconds of the video

  //   cout << "Frame per seconds : " << fps << endl;


  while(1)
  {

    Mat frame, resized;


    bool bSuccess = cap.read(frame); // read a new frame from video


    if (!bSuccess) //if not success, break loop
    {
      cout << "Cannot read the frame from video file" << endl;
      break;
    }
    resize(frame, resized, Size(), 0.5, 0.5);//reduce image size
    //imshow("MyVideo", resized); //show the frame in "MyVideo" window

    cv::Mat hsv_image, red;
    cv::medianBlur(resized, resized, 3);//blur the image before changing to HSV
    cv::cvtColor(resized, hsv_image, cv::COLOR_BGR2HSV);//change from BGR to HSV, easier to detect colors



    //You can change the color point here !!
    //detect red..change here to detect another color
    cv::inRange(hsv_image, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), red);

    cv::medianBlur(red, red,3);//blur again

    //Canny

    Mat canny_output;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    RNG rng(12345);

    /// Detect edges using canny
    Canny( red, canny_output, 100, 100*2, 3 );
    /// Find contours
    findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );


    /// Draw contours
    Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
    for( int i = 0; i< contours.size(); i++ )
    {
      Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
      drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
    }

    /// Show in a window
    namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
    imshow( "Contours", drawing );

    //Canny



    //centroid

    findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );


    /// Get the moments
    vector<Moments> mu(contours.size() );
    if(contours.empty()==false)
    {
      for( int i = 0; i < contours.size(); i++ )
      { mu[i] = moments( contours[i], false ); }
    }


    ///  Get the mass centers:
    vector<Point2f> mc( contours.size() );


    for( int i = 0; i < contours.size(); i++ )
    { mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ); }

    // cout << "X1XXCannot read the frame from video file" << endl;


    //Problem
    if((mc[0].x != mc[0].x) == false)//VERY IMPORTANT!!! THIS line delete all the NaN value from the trajectory

      //  cout << "X1.5XXCannot read the frame from video file" << endl;

      //Problem
      trajectory.push_back(mc[0]); // Put X & Y into trajectory for drawing later!

    //cout << "X2XXCannot read the frame from video file" << endl;

    // Save  X  & Y points into files
    myfilex<<mc[0].x<<endl;
    myfiley<<mc[0].y<<endl;


    //here we only return the first contour....this might change with different applications
    //take the points of the trajectory and draw a line..
    if(trajectory.size()>=2)
    {
      for(int i=1;i<trajectory.size();i++)
        // Draw a red line of each frame
        line(resized, trajectory.at(i-1)/*x & y values*/,trajectory.at(i) /*x & y values*/, Scalar(100,100,255) /*Color*/, 3 /*Thickness*/, 8, 0);
    }


    imshow("Tarsus", red); //show the frame in "MyVideo" window
    imshow("Video", resized);


    if(waitKey(30) == 27) //wait for 'esc' key press for 30 ms. If 'esc' key is pressed, break loop
    {
      cout << "esc key is pressed by user" << endl;
      break;
    }
  }
  myfilex.close();
  myfiley.close();
  return 0;

}

