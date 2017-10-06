#ifndef MVG_PDF_H
#define MVG_PDF_H
#include <stdlib.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#include <iostream>
#include <cmath>
# define M_PIl          3.141592653589793238462643383279502884L /* pi */
#include "mcmc_types.h"
using namespace std;
using namespace cv;



// This allows us to determine the pdf given the mean and covariance. This is only
// intended for a 2x2 covariance matrix.

typedef class Multivaritive_Gaussian_PDF
{
public:
   Mat covariance = 0;
   Mat mean = 0;
   float coefficient = 0;//everything in front of the e^(moremath)
   Mat inverse_covariance = 0;
        KalmanFilter KF ( 4, 2, 0 );
   
  // use this for each seperate track just a utility function
  float Track_Likelihood();
    
  // returns the probablities of all the tracks
    float Probability(vector<Node *> & track_START);
  
  
};



#endif
