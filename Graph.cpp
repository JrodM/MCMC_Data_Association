#include "Graph.h"

Temporal_Entity_Tracking_Graph::Temporal_Entity_Tracking_Graph()
{
        // this is used to fill and initialize our time_eventdow

        for ( int i = 0; i < WINDOW_SIZE; i++ ) {
                sliding_window.push_back ( blank );
        }

}


// pop off the most recent event and correctly initiate the next time frame
// that contains x,y events in the form of TNodes

void Temporal_Entity_Tracking_Graph:: newTimeEvent()
{

        Time_Frame earliest_window = sliding_window[0];
        total_observations -= earliest_window.frame.size();
        sliding_window.pop_front();

        sliding_window.push_back ( blank );

        // clean up memmory with edges etc and correctly move the beginning nodes over


        for ( vector<TNode *>::iterator i = earliest_window.frame.begin(); i != earliest_window.frame.end(); i++ ) {

                TNode * time_event = *i;

                // for each edge in the outedges delete
                //clean up edges

                for ( vector<Edge *>::iterator j = time_event->out_edges.begin(); j!= time_event->out_edges.end(); j++ ) {

                        Edge * delete_edge = *j;

                        // if we have a start path node instantiate the next temporal node as the beginning


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
                // dont need to delete the out edge list because we are deleting a node
                delete time_event;

        }

        //earliest_window.clear();

}

void Temporal_Entity_Tracking_Graph::construct_Paths ( int max_eucldiean_distance , int max_missed_frames )
{
        // set all of the times up in the window frame
        set_Time_Frames();
        // loop through each window and construct paths to potential nodes in t+maxmissedframes of time.
        //
        // NO LONGER FIRST WINDOW WE DO THIS FOR OUR PROPOSAL WINDOW SIZE
        // go through all possible cconnecting nodes
        for ( deque<Time_Frame>::reverse_iterator start_propose = sliding_window.rbegin(); start_propose != sliding_window.rbegin() + PROPOSAL_WINDOW_SIZE; start_propose++ ) {
                int distance = 0;
                for ( deque<Time_Frame>::reverse_iterator k = start_propose +1 ; k != sliding_window.rend() && k!= start_propose + max_missed_frames; k++ ) {
                        vector<TNode*> k1 = ( *k ).frame;
                        // add one to the distance, this helps for probabilty calculations
                        distance++;

                        // for eachnode in the time window
                        for ( vector<TNode*>::iterator i =k1.begin(); i != k1.end(); i++ ) {
                                // k1 is the source

                                for ( vector<TNode *>::iterator j = ( *start_propose ).frame.begin() ; j!= ( *start_propose ).frame.end(); j++ ) {

                                        // construct the paths if they are close enough
                                        ///distance * max movement per frame stivk with euclidean fot now


                                        //1/30 messages per second. migt need to adjust threshold.
                                        if ( pow ( pow ( ( *j )->location.x- ( *i )->location.x,2 ) + pow ( ( *j )->location.y- ( *i )->location.y,2 ),.5 ) < distance * 20 ) {
                                                //the i  is the source and jis the target.
                                                Edge * new_edge = new Edge();
                                                new_edge->source = *i;
                                                new_edge->target  = *j;
                                                new_edge->time_distance = distance;
                                                ( *i )->out_edges.push_back ( new_edge );
                                                ( *j )->in_edges.push_back ( new_edge );
                                                cout<<"New Edge to: "<< new_edge->target->frame->time<< " From:" << new_edge->source->frame->time<<endl;
                                        }

                                }
                        }
                }
        }

        /* Clean up the mutable node's */
        /*   for ( vector<vector<TNode *>>::reverse_iterator start_propose = sliding_window.rbegin() +PROPOSAL_WINDOW_SIZE; start_propose != sliding_window.rend() && start_propose !=sliding_window.rbegin() + PROPOSAL_WINDOW_SIZE+ PROPOSAL_WINDOW_SIZE; start_propose++ ) {

                   //clean up node in each window
                   for ( auto &clean: *start_propose ) {
                           clean->is_mutable = false;
                   }

           }*/



}

void Temporal_Entity_Tracking_Graph::set_Time_Frames()
{
        int i = 0;
        for ( auto & f:sliding_window ) {
                f.time = i;
		i++;

        }
}


// obviously add to the current window
void Temporal_Entity_Tracking_Graph::add_Location ( int x, int y )
{
        sliding_window[sliding_window.size()-1].frame.push_back ( new TNode ( x,y,&sliding_window[sliding_window.size()-1] ) );
        total_observations++;

}




//stats at time t
void Temporal_Entity_Tracking_Graph::graph_Stats ( int t,int & at, int & zt, int & ct, int & dt, int & ut,int & ft ,int et_[WINDOW_SIZE])
{
        //et: number of targets from t-1
        //at: number of new targets at time t
        //zt: number of targets terminated at time t
        //ct: et-zt targets persisted
        //dt: number of detections at time T
        //ut: et-zt+at-dt be the number of undetected targets
        //ft: nt-dt the number of false alarms
        // nt is the total points at t
	  //dt detected targets
        zt = 0;
        // find targets that terminate

        // num targets at et
        int et = 0;
        at = 0;
        dt = 0;
	
        if ( t ) { // if it isnt 0
	  
	  et =et_[t-1];

        }
        
        

        int nt = sliding_window[t].frame.size();

	
        for ( auto & node: sliding_window[t].frame ) {
	   bool detected = false;
	   
                if ( node->start_of_path == true ) {
		  
                        at++;
	
                }
                
                if(!node->active_out)
		{
		  
                for ( auto & e: node->in_edges ) {
                        if ( e->active == true) {
			
				   zt++;
				   dt++;
				   break;
				
                               
                        }
                
		}
		  
		}
		else
		{
		  dt++;
		}

                // if we've gone through both the in and out edge lists
                // without seeing an active edge false detection.
        }




       /* for ( auto & node: sliding_window[t].frame ) {
                //if the track from time t doesn't have anothe active out node
                // then we know that it terminates
                


        }// end of dt and ft*/
        ct = et - zt;
        ut  = et -zt + at - dt;
        ft = nt -dt;

}

// Prior functions
float Temporal_Entity_Tracking_Graph::Prior()
{
        cout<<"Calculating Prior"<<endl<<endl;
        float prob =0;
        int t =0;
        int at = 0;
        int zt = 0;
        int  ct = 0;
        int dt = 0;
        int  ut = 0;
        int  ft = 0;

	int et[WINDOW_SIZE] = {0};
	// find our et's
	for(auto & nodex: start_nodes)
	{
	 //for each path lets figured 
	  TNode * n = nodex;
	  while(n->active_out)
	  {
	    int time = n->frame->time;
	    int time_difference  = n->active_out->time_distance;
	    
	    while(time_difference-- > 1)
	    {
	      et[time + time_difference]++;
	    }
	      et[time]++;
	     n = n->active_out->target; 
	  }
	  
	    int time = n->frame->time;
	    et[time]++;
	     //n = n->active_out->target; 
	  
	}
	
        for ( t = 0; t<sliding_window.size(); t++ ) {
                graph_Stats ( t,at,zt,ct,dt,ut,ft ,et);

                prob+= zt*log ( pz );
                prob+=ct*log ( 1-pz );
                prob+=dt*log ( pd );
                prob+=ut*log ( 1-pd );
                prob+=at*log ( lambda_b );
                prob+=ft*log ( lambda_f );
        }
        cout<<"DONE calculating Prior"<<endl<<endl;
        return prob;
}

//posterior
float Temporal_Entity_Tracking_Graph::Posterior()
{
        cout<<endl<<"Calculating Posterior"<<endl;
	float prior = Prior() ;
	float likelihood = track_likelihood.Probability ( start_nodes )/10;
	
	cout<<"|Prior: "<< prior<<"|Likelihood: "<<likelihood<<"|"<<endl;
        return (prior+ likelihood );
}

