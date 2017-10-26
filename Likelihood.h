#ifndef LIKELIHOOD_H
#define LIKELIHOOD_H
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

typedef class Likelihood
{
public:
  Likelihood();
   // state is 4 for x,y cordinates and their acceleration. 
   KalmanFilter KF;
   // the predicted points for the kalman filter. We need this for missing points in time.
  Mat prediction;
   
  // use this for each seperate track just a utility function
  float Track_Likelihood(float measurement_x,float measurement_y);
  const float pi= 3.1415927;
  // returns the probablities of all the tracks
  float Probability(vector<TNode *> & track_START);
  int track_Length ( TNode * n );
  
};



#endif
