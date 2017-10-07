#ifndef graph_H
#define graph_H
#include "mcmc_types.h"


#define WINDOW_SIZE 80

// We assume our time_eventdow is 40 frames. We currently have 30 frames per second

typedef class Temporal_Entity_Tracking_Graph
{


private:
    //not really needed but we push this blank array for each new time event.
    vector<Node *> blank;
    //KalmanFilter KF ( 4, 2, 0 );

public:

    // a list of all of the edges in the graph.
    // we map all of the addresses of the node.outedges
    // and delete only if we delete the node producing the connection.

    vector<Edge*> proposal_edge_list;
    vector<Node*> start_nodes;
     //KalmanFilter KF ( 4, 2, 0 );


    // our sliding time_eventdow of points. Each index is another point in time
    deque<vector<Node *> > sliding_window;
    float pz; //probability of an object dissappearing
    float  lambda_b;//birth rate of new objects per unit time per unit voluma
    float lambda_f;// false alarm rate   

    Temporal_Entity_Tracking_Graph();


    // pop off the most recent event and correctly initiate the next time frame
    // that contains x,y events in the form of Nodes
    void newTimeEvent() ;
    //add a new point in the current time event
    void add_location_current_event(int x, int y);
    void ConstructPaths ( int = 30, int = 60 ) ;
    


} Graph;

#endif