#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#include <iostream>
#define drawCross( center, color, d )                                 \
line( img, Point( center.x - d, center.y - d ), Point( center.x + d, center.y + d ), color, 2, CV_AA, 0); \
line( img, Point( center.x + d, center.y - d ), Point( center.x - d, center.y + d ), color, 2, CV_AA, 0 )
 
using namespace cv;
using namespace std;
 struct POINT;


struct POINT
{
  
  int x = 0;
  int y =0;
};

POINT mousePos;

void mouseCallback(int event, int x, int y, int flags, void* userdata) 
{
if ( event == EVENT_MOUSEMOVE )
{
mousePos.x = x;
mousePos.y = y;
}

}



int main( )
{ 
 
KalmanFilter KF(4, 2, 0);

mousePos.x = 2;
mousePos.y = 2;
 
// intialization of KF...
KF.transitionMatrix = (Mat_<float>(4, 4) << 1,0,.1,0,   0,1,0,.1,  0,0,1,0,  0,0,0,1);
Mat_<float> measurement(4,1); measurement.setTo(Scalar(0));
//KF.controlMatrix = (Mat_<float>(4, 2) << .005,0,   0,.0005,  .1,0,  0,.1);
KF.statePost.at<float>(0) = mousePos.x;
KF.statePost.at<float>(1) = mousePos.y;
KF.statePost.at<float>(2) = 0;
KF.statePost.at<float>(3) = 0;
//setIdentity(KF.measurementMatrix);
KF.measurementMatrix = (Mat_<float>(2, 4) << 1,0,0,0 ,  0,1,0,0);
setIdentity(KF.processNoiseCov);
setIdentity(KF.measurementNoiseCov);
setIdentity(KF.errorCovPost, Scalar::all(.1));
// Image to show mouse tracking
Mat img(600, 800, CV_8UC3);
vector<Point> mousev,kalmanv;
mousev.clear();
kalmanv.clear();
 

namedWindow("mouse kalman", 1);
setMouseCallback("mouse kalman", mouseCallback, NULL);
int px =0; 
int py = 0;
int i = 0;
while(1)
{

 // First predict, to update the internal statePre variable
 Mat prediction = KF.predict();
 Point predictPt(prediction.at<float>(0),prediction.at<float>(1));
              
 // Get mouse point
         px =  mousePos.x ;
    py = mousePos.y;
    if(i++ >0)
  {

     measurement(2) = px - measurement(0);
 measurement(3) = py- measurement(1); 
    
  }
  
      measurement(0) = px;
 measurement(1) = py; 
 // The update phase 
 
  cout<<KF.measurementMatrix*KF.errorCovPre* KF.measurementMatrix.t()+KF.measurementNoiseCov;
 cout<<endl<<endl;
 Mat estimated = KF.correct(KF.measurementMatrix*measurement);
// cout<<KF.measurementMatrix<<endl;
 //   cout << KF.errorCovPost<<endl;
    cout<<KF.measurementMatrix<<endl<<endl;
 Point statePt(prediction.at<float>(0),prediction.at<float>(1));
 
 Point measPt(measurement(0),measurement(1));
    // plot points
    imshow("mouse kalman", img);
    img = Scalar::all(0);
 
    mousev.push_back(measPt);
    kalmanv.push_back(statePt);
    drawCross( statePt, Scalar(255,255,255), 5 );
    drawCross( measPt, Scalar(0,0,255), 5 );
 
    for (int i = 0; i < mousev.size()-1; i++) 
     line(img, mousev[i], mousev[i+1], Scalar(255,255,0), 1);
     
    for (int i = 0; i < kalmanv.size()-1; i++) 
     line(img, kalmanv[i], kalmanv[i+1], Scalar(0,155,255), 1);
 
     waitKey(100); 
   /* px =  mousePos.x ;
    py = mousePos.y;*/
    // measurement(2) = px - measurement(0);
 //measurement(3) = py- measurement(1); 
    
    measurement(0) = px;
 measurement(1) = py; 
 
 //Mat B = 
   
 //waitKey(10);  
}
                                           
    return 0;
}