
#include "Likelihood.h"
Likelihood::Likelihood()
{
  //might be wasteful tbh
  KF.init ( 4, 2, 0 );
}

float Likelihood::Probability ( vector<TNode *> & track_START )
{

        vector<TNode *>::iterator i;

	float probability;
	
	/* Do the track likelihood fist*/
	
        for ( i = track_START.begin(); i!= track_START.end() ;  i++ ) {

                TNode * path = *i;
                // intialize kalman filter for each path
                KF.init ( 4,2,0 );
                KF.transitionMatrix = ( Mat_<float> ( 4, 4 ) << 1,0,1,0,   0,1,0,1,  0,0,1,0,  0,0,0,1 );
                KF.controlMatrix = ( Mat_<float> ( 4, 2 ) << .5,0,   0,.5,  1,0,  0,1 );
                KF.measurementMatrix = ( Mat_<float> ( 2, 4 ) << 1,0,0,0 ,  0,1,0,0 );
                setIdentity ( KF.processNoiseCov );
                setIdentity ( KF.measurementNoiseCov );
                KF.processNoiseCov =KF.processNoiseCov * 10;
                KF.measurementNoiseCov = KF.measurementNoiseCov*5;
		
		
		// too lazy too add noise oh well.
                // set the post state to our intial pt
                KF.statePost.at<float> ( 0 ) = (*i)->location.x;
                KF.statePost.at<float> ( 1 ) = (*i)->location.y;
                KF.statePost.at<float> ( 2 ) = 0;
                KF.statePost.at<float> ( 3 ) = 0;
		//next state
		//path = path->active_out;
		
		// calculate the path probability
		
		//we always assume that a track length is greater then one
		// no need to check.
		do{
		  
		
		  // First predict, to update the internal statePre variable
		   prediction = KF.predict();
		  
		  //difference in frames to detect missed detections
		  // and account for where they are in between those times
		
		  if(path->active_out->time_distance>1)
		  {
		   
		    int counter = path->active_out->time_distance;
		    
		    //account for missed detections
		    do{
		    probability += Track_Likelihood(prediction.at<float>(0),prediction.at<float>(1));
		    prediction = KF.predict();
		    }while(--counter>1);
		    
		  }
		  
		path = path->active_out->target;
		
		probability+=Track_Likelihood(path->location.x,path->location.y);
		  
		}while(path != 0);
		
        }
        
        return probability;

}

float Likelihood::Track_Likelihood(float measurement_x,float measurement_y)
{

        Mat_<float> measurement ( 2,1 );
        measurement.setTo ( Scalar ( 0 ) );



        Mat_<float> noise ( 2,1 );
        noise.setTo ( Scalar ( 0 ) );
        randn ( noise,Scalar::all ( 0 ),Scalar ( 5 ) );

        measurement ( 0 ) = measurement_x;

        measurement ( 1 ) = measurement_y;

        measurement = measurement + noise;

	//covariance matrix
        Mat B = ( KF.measurementMatrix*KF.errorCovPre* KF.measurementMatrix.t() +KF.measurementNoiseCov );
	
	
	Mat_<float> sub= ( Mat_<float> ( 2, 1 ) << ((prediction.at<float>(0)-measurement_x),(prediction.at<float>(1)-measurement_y)));
	//.5 comes from the fact we only have two measurements m .... 1/m is the pow 
	float event_probability = 0;
	event_probability = ((cv::Mat)(-0.5*log(2 *pi)+-0.5 *log(determinant(B))+(-0.5*sub.t()* B.inv()*sub))).at<float>(0);
	
	
        KF.correct ( measurement );
	
	return event_probability;
}


