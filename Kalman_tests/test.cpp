#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#include <iostream>
#include <cmath>
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


KF.transitionMatrix = (Mat_<float>(4, 4) << 1,0,1,0,   0,1,0,1,  0,0,1,0,  0,0,0,1);
KF.controlMatrix = (Mat_<float>(4, 2) << .5,0,   0,.5,  1,0,  0,1);
KF.measurementMatrix = (Mat_<float>(2, 4) << 1,0,0,0 ,  0,1,0,0);
//KF.measurementNoiseCov = (Mat_<float>(2, 2) << 25,0,   0,25);
//KF.processNoiseCov = (Mat_<float>(2, 2) << 100,0,   0,100);
Mat_<float> measurement(2,1); measurement.setTo(Scalar(0));
Mat_<float> measurement2(2,1); measurement.setTo(Scalar(0));
Mat_<float> me(2,1);
 
KF.statePost.at<float>(0) = mousePos.x;
KF.statePost.at<float>(1) = mousePos.y;
KF.statePost.at<float>(2) = 0;
KF.statePost.at<float>(3) = 0;

setIdentity(KF.processNoiseCov);
setIdentity(KF.measurementNoiseCov);
setIdentity(KF.errorCovPost, Scalar::all(.1));

KF.processNoiseCov =KF.processNoiseCov * 10;
KF.measurementNoiseCov = KF.measurementNoiseCov*5;
//cout<< KF.processNoiseCov;
cout<< KF.measurementNoiseCov;
//setIdentity(KF.measurementNoiseCov, Scalar::all(10));
//setIdentity(KF.errorCovPost, Scalar::all(.1));
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
    // cout<< prediction;
 Point predictPt(prediction.at<float>(0),prediction.at<float>(1));
              
 // Get mouse point
         px =  mousePos.x ;
    py = mousePos.y;
    
    measurement(0) = px;
    measurement(1) = py;
    
      Mat_<float> noise(2,1); noise.setTo(Scalar(0));
      randn(noise,Scalar::all(0),Scalar(5));
      
      measurement = measurement + noise ;
      
 // The update phase 
 Mat B = (KF.measurementMatrix*KF.errorCovPre* KF.measurementMatrix.t()+KF.measurementNoiseCov);
  	Mat_<float> sub= ( Mat_<float> ( 2, 1 ) << ((prediction.at<float>(0)-measurement(0),(prediction.at<float>(1)-measurement(1)))));
	//.5 comes from the fact we only have two measurements m .... 1/m is the pow 
	float event_probability = 0;
	
 cout << (determinant(B)+-0.5*sub.t()* B.inv()*sub).at<float>(0) <<endl;
 
 //cout<< "MAT erroCove Pre"<< KF.errorCovPre <<endl;
 

 
 Mat estimated = KF.correct(measurement);
//cout<<"INITIAL"<<KF.errorCovPost<<endl;
 //   cout << KF.errorCovPost<<endl;
   
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
 //cout<< estimated<< endl;
     waitKey(10); 
   
 //Mat B = 
   
 //waitKey(10);  
}
                                           
    return 0;
}
