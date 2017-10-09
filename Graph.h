#ifndef graph_H
#define graph_H
#include "mcmc_types.h"
#include "Likelihood.h"

#define WINDOW_SIZE 80

// We currently have 30 frames per second

typedef class Temporal_Entity_Tracking_Graph
{


private:
    //not really needed but we push this blank array for each new time event.
    vector<Node *> blank;
    
    // responsible for track likelihood functionality. Easier to wrap and modularize the kalman filter in here.
    Likelihood track_likelihood;

public:

    // a list of all of the edges in the graph.
    // we map all of the addresses of the node.outedges
    // and delete only if we delete the node producing the connection.
    vector<Edge*> proposal_edge_list;
    vector<Node*> start_nodes;



    // our sliding time_eventdow of points. Each index is another point in time
    deque<vector<Node *> > sliding_window;
    
    /* Parameters that can be tuned */
    float pz; //probability of an object dissappearing
    float pd;
    float  lambda_b;//birth rate of new objects per unit time per unit voluma
    float lambda_f;// false alarm rate  
    /*********************************/
    
    // maximum posterior: this is the configuration the produces the highest p(w|Y)
    vector<vector<Node>> MAP_estimate;
    float MAP_prob;

    Temporal_Entity_Tracking_Graph();


    // pop off the most recent event and correctly initiate the next time frame
    // that contains x,y events in the form of Nodes
    void newTimeEvent() ;
    //add a new point in the current time event
    void add_location_current_event(int x, int y);
    //adds edges to the newest time event 
    void construct_Paths ( int = 30, int = 60 );
    //add current event/points to the current window
    void add_Location(int x, int y);
    //stats at time t. For more on the stats go to function
    void graph_Stats( int t,int & at, int & zt, int & ct, int & dt, int & ut,int & ft );
    // Prior functions
    float Prior();
    //posterior
    float Posterior();
    


} Graph;

#endif