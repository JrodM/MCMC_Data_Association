
#include "Multivaritive_Gaussian_PDF.h"

float Multivaritive_Gaussian_PDF::Probability ( vector<Node *> & track_START )
{

        vector<Node *>::iterator i;


        for ( i = track_START.begin(); i!= track_START.end() ;  i++ ) {

                Node * path = i;
                // intialize kalman filter for each path
                KF.init ( 4,2,0 );
                KF.transitionMatrix = ( Mat_<float> ( 4, 4 ) << 1,0,1,0,   0,1,0,1,  0,0,1,0,  0,0,0,1 );
                KF.controlMatrix = ( Mat_<float> ( 4, 2 ) << .5,0,   0,.5,  1,0,  0,1 );
                KF.measurementMatrix = ( Mat_<float> ( 2, 4 ) << 1,0,0,0 ,  0,1,0,0 );
                setIdentity ( KF.processNoiseCov );
                setIdentity ( KF.measurementNoiseCov );
                KF.processNoiseCov =KF.processNoiseCov * 10;
                KF.measurementNoiseCov = KF.measurementNoiseCov*5;

                // set the post state to our intial pt
                KF.statePost.at<float> ( 0 ) = i->x;
                KF.statePost.at<float> ( 1 ) = i->y;
                KF.statePost.at<float> ( 2 ) = 0;
                KF.statePost.at<float> ( 3 ) = 0;

        }


}

float Multivaritive_Gaussian_PDF::Track_Likelihood()
{

        Mat_<float> measurement ( 2,1 );
        measurement.setTo ( Scalar ( 0 ) );


        int px =0;
        int py = 0;
        int i = 0;

// First predict, to update the internal statePre variable
        Mat prediction = KF.predict();



        Mat_<float> noise ( 2,1 );
        noise.setTo ( Scalar ( 0 ) );
        randn ( noise,Scalar::all ( 0 ),Scalar ( 5 ) );

        measurement ( 0 ) = px;

        measurement ( 1 ) = py;

        measurement = measurement + noise;


        Mat B = ( KF.measurementMatrix*KF.errorCovPre* KF.measurementMatrix.t() +KF.measurementNoiseCov );

        KF.correct ( measurement );

}


