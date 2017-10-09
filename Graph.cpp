#include "Graph.h"

Temporal_Entity_Tracking_Graph()
{
        // this is used to fill and initialize our time_eventdow

        for ( int i = 0; i < WINDOW_SIZE; i++ ) {
                sliding_window.push_back ( blank );
        }

}


// pop off the most recent event and correctly initiate the next time frame
// that contains x,y events in the form of Nodes
void Temporal_Entity_Tracking_Graph:: newTimeEvent()
{

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

        // loop through each window and construct paths to potential nodes in t+maxmissedframes of time.
        // O(n**2) we actually avoid this by only constructing for the first window/event
        // go through all possible cconnecting nodes
        int distance = 0;
        for ( vector<vector<Node *>>::iterator k = sliding_window.begin() +1 ; k != sliding_window.end() && k!= sliding_window.begin() + max_missed_frames; k++ ) {
                vector<Node*> k1 = *k;
                // add one to the distance, this helps for probabilty calculations
                distance++;

                for ( vector<Node*>::iterator i =k1.begin(); i != k1.end(); i++ ) {

                        //we only need to compare paths for the first time entry since we only
                        // get one time event at a time.
                        for ( vector<Node *>::iterator j = sliding_window[0].begin() ; j!= sliding_window[0].end(); j++ ) {

                                // construct the paths if they are close enough
                                if ( pow ( pow ( *j->location.x-*i->location.x,2 ) + pow ( *j->location.y-*i->location.y,2 ),.5 ) < max_eucldiean_distance ) {
                                        //the j is the source and j is the target.
                                        Edge * new_edge = new Edge();
                                        new_edge->source = *j;
                                        new_edge->target  = *i;
                                        new_edge->time_distance = distance;
                                        *j->out_edges.push ( new_edge );
                                        *i->in_edges.push ( new_edge );
                                }

                        }
                }
        }

}


// obviously add to the current window
void Temporal_Entity_Tracking_Graph::add_Location ( int x, int y )
{
        sliding_window[0].pushback ( new Node ( x,y ) );
}




//stats at time t
void Temporal_Entity_Tracking_Graph::graph_Stats ( int t,int & at, int & zt, int & ct, int & dt, int & ut,int & ft )
{
        //et: number of targets from t-1
        //at: number of new targets at time t
        //zt: number of targets terminated at time t
        //ct: et-zt targets persisted
        //dt: number of detections at time T
        //ut: et-zt+at-dt
        //ft: nt-dt the number of false alarms
        // nt is the total points at t

        zt = 0;
        // find targets that terminate

        // num targets at t-1
        int et = 0;

        for ( auto & node: sliding_window[t-1] ) {
                //if the track from time t doesn't have anothe active out node
                // then we know that it terminates
                if ( node->active_out !=0 ) {
                        if ( node->active_out->target->active_out == 0 ) {
                                zt++;
                        }

                        et++;

                } else {
                        for ( auto & e: node->in_edges ) {

                                if ( e.active ) {
                                        et++;
                                        break;
                                }

                        }

                }
        }

        ct = et - zt;

        int nt = sliding_window[t].size();

        at = 0;

        for ( auto & node: sliding_window[t] ) {
                if ( node->start_of_path == true ) {
                        at++;
                }
        }


        dt = 0;

        for ( auto & node: sliding_window[t] ) {
                //if the track from time t doesn't have anothe active out node
                // then we know that it terminates
                bool detected = false;

                for ( auto & e: node->in_edges ) {
                        if ( e.active == true ) {
                                detected = true;
                                dt++;
                                break;
                        }
                }

                if ( !detected ) {

                        for ( auto & e: node->out_edges ) {
                                if ( e.active == true ) {
                                        //detected = true;
                                        dt++;
                                        break;
                                }
                        }

                }
                // if we've gone through both the in and out edge lists
                // without seeing an active edge false detection.


        }// end of dt and ft

        ut  = et -zt + at - dt;
        ft = nt -dt;

}

    // Prior functions
    float Temporal_Entity_Tracking_Graph::Prior()
    {
      float prob =0;
       int t =0;
       int at = 0;
       int zt = 0;
       int  ct = 0;
       int dt = 0;
       int  ut = 0;
       int  ft = 0;
       
       for(t = 0; t<sliding_window.size();t++)
       {
	 graph_Stats (t,at,zt,ct,dt,ut,ft );
	  
	 prob+= zt*log(pz);
	 prob+=ct*log(1-pz);
	 prob+=dt*log(pd);
	 prob+=ut*log(1-pd);
	 prob+=at*log(lambda_b);
	 prob+=ft*log(lambda_f);
       }
      
      return prob;
    }
    
    //posterior
    float Temporal_Entity_Tracking_Graph::Posterior()
    {
      return (Prior() + track_likelihood.Probability(start_nodes));
    }

