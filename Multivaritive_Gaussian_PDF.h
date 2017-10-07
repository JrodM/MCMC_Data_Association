#ifndef MVG_PDF_H
#define MVG_PDF_H
#include <stdlib.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#include <iostream>
#include <cmath>
#include "mcmc_types.h"
using namespace std;
using namespace cv;


/* IT needs to be noted that for images the x,y start in the top left corner
 * this probably won't be an issue but for debugging commented */

// This allows us to determine the pdf given the mean and covariance. This is only
// intended for a 2x2 covariance matrix.

typedef class Multivaritive_Gaussian_PDF
{
public:
   // state is 4 for x,y cordinates and their acceleration. 
   KalmanFilter KF ( 4, 2, 0 );
   // the predicted points for the kalman filter. We need this for missing points in time.
Mat prediction;
   
  // use this for each seperate track just a utility function
  float Track_Likelihood();
 const float pi= 3.1415927;
  // returns the probablities of all the tracks
    float Probability(vector<Node *> & track_START);
  
  
};



#endif
