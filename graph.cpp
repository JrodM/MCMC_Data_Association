#include "graph.h"  
  
  Temporal_Entity_Tracking_Graph() {
        // this is used to fill and initialize our time_eventdow

        for ( int i = 0; i < WINDOW_SIZE; i++ ) {
            sliding_window.push_back ( blank );
        }

    }


    // pop off the most recent event and correctly initiate the next time frame
    // that contains x,y events in the form of Nodes
    void Temporal_Entity_Tracking_Graph:: newTimeEvent() {

        vector<Node *> earliest_window = sliding_window[0];
        sliding_window.pop_front();

        sliding_window.push_back ( blank );

        // clean up memmory with edges etc and correctly move the beginning nodes over


        for ( vector<Node *>::iterator i = earliest_window.begin(); i != earliest_window.end(); i++ ) {

            Node * time_event = *i;

            // for each edge in the outedges delete
            //clean up edges

            for ( vector<Edge *>::iterator j = time_event->out_edges.begin(); j!= time_event->out_edges.end(); j++ ) {

                Edge * delete_edge = *j;

                // if we have a start path node instantiate the next temporal node as the beginning
                // path start node.
                if ( delete_edge->active == true && time_event->start_of_path == true ) {
                    // add to our
                    vector_erase ( start_nodes,time_event );
                    start_nodes.push_back ( delete_edge->target );
                    delete_edge->target->start_of_path = true;

                }

                //vector_erase(edge_list, delete_edge);

                // remove each of the  edges from the
                //in_edges
                for ( vector<Edge *>::iterator k = delete_edge->target->in_edges.begin(); k!= delete_edge->target->in_edges.end(); k++ ) {

                    Edge * delete_in_edge = *k;

                    if ( delete_edge == delete_in_edge ) {
                        delete_edge->target->in_edges.erase ( k );
                        break;
                    }


                }

                delete delete_edge;
            }

            delete time_event;

        }

        //earliest_window.clear();

    }

    void Temporal_Entity_Tracking_Graph::ConstructPaths ( int max_eucldiean_distance = 30, int max_missed_frames = 60 ) {

        // loop through each window and construct paths to potential nodes in t+maxmissedframes of time.
        // O(n**2)
        for ( int i = 0; i < WINDOW_SIZE; i++ ) {
            for ( vector<Node *>::iterator j = sliding_window.begin() ; j!= sliding_window.end(); j++ ) {
                // go through all possible cconnecting nodes
                for ( vector<Node *>::iterator k = j +1; k != sliding_window.end() && k!= j + max_missed_frames; k++ ) {
                    Node * k1 = *k;
                    Node * j2 =*j;
                    if ( pow ( pow ( j1->location.x-k1->location.x,2 ) + pow ( j1->location.y-k1->location.y,2 ),.5 );
                }

        }
                //look at each frame up to the max number of missed frames for a matcch

    }

}

        // the likelihood function of all of the proposed paths
float Temporal_Entity_Tracking_Graph::likelihood() {

        KF.init ( 4,2,0 );
        KF.transitionMatrix = ( Mat_<float> ( 4, 4 ) << 1,0,1,0,   0,1,0,1,  0,0,1,0,  0,0,0,1 );
        KF.controlMatrix = ( Mat_<float> ( 4, 2 ) << .5,0,   0,.5,  1,0,  0,1 );
        KF.measurementMatrix = ( Mat_<float> ( 2, 4 ) << 1,0,0,0 ,  0,1,0,0 );
        setIdentity ( KF.processNoiseCov );
        setIdentity ( KF.measurementNoiseCov );
        KF.processNoiseCov =KF.processNoiseCov * 10;
        KF.measurementNoiseCov = KF.measurementNoiseCov*5;

        Mat_<float> measurement ( 2,1 );
        measurement.setTo ( Scalar ( 0 ) );

        KF.statePost.at<float> ( 0 ) = mousePos.x;
        KF.statePost.at<float> ( 1 ) = mousePos.y;
        KF.statePost.at<float> ( 2 ) = 0;
        KF.statePost.at<float> ( 3 ) = 0;

        setIdentity ( KF.processNoiseCov );
        setIdentity ( KF.measurementNoiseCov );

        KF.processNoiseCov =KF.processNoiseCov * 10;
        KF.measurementNoiseCov = KF.measurementNoiseCov*5;


        int px =0;
        int py = 0;
        int i = 0;
        while ( 1 ) {

// First predict, to update the internal statePre variable
            Mat prediction = KF.predict();



            Mat_<float> noise ( 2,1 );
            noise.setTo ( Scalar ( 0 ) );
            randn ( noise,Scalar::all ( 0 ),Scalar ( 5 ) );

            measurement ( 0 ) = px;

            measurement ( 1 ) = py;

            measurement = measurement + noise;

// The update phase
            Mat B = ( KF.measurementMatrix*KF.errorCovPre* KF.measurementMatrix.t() +KF.measurementNoiseCov );



            KF.correct ( measurement );


        }


    }
