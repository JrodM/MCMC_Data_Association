#ifndef MCMCDA_H
#define MCMCDA_H
#include "Graph.h"

class MCMCDA
{
public:
    Graph proposal_graph;

    // maximum posterior: this is the configuration the produces the highest p(w|Y)
    vector<vector<Node>> MAP_estimate;
    float MAP_prob;

    // may have to reconsider probability functions but this holds all unassigned nodes
    vector<vector<Node *>> noise;

    std::random_device rd;  //Will be used to obtain a seed for the random number engine
    std::mt19937 * gen;
    const float gamma  = .001; // the chance of ending a track early
    
    
    /******************FUNCTIONS*******************/

    //add a new point in the current time event

    void add_location_current_event ( int x, int y );

    void Propose_Deactivate ( Edge * e );
    void Propose_Activate ( Edge * e );
    int track_Length ( Node * n );
    vector<Node*> get_Tracks_At_T(int t);
    Edge * nodes_2_Edge(Node * n1, Node * n2);
    vector<Node*> extendable_Tracks();
    vector<tuple<Node*,vector<Edge *>>> mergable_Vectors();
    vector<Edge * > Inactive_Nodes ( Node *n );
    bool Is_Active ( Node *n );
    

    //proposal moves
    bool Extend ( Node * n );
    bool Birth_Move();
    bool Death_Move();
    bool Update_Move();
    bool Extension_Move();
    bool Reduction_Move();
    bool Switch(Node* t1, Node * t2);
    bool Switch_Move();
    bool Merge_Move();
    bool Split_Move();

    void Accept_Proposal();
    void Reject_Proposal();
    
    MCMCDA();
    ~MCMCDA();
    
/************************************************/
    //list of different proposal functions
    vector<bool (*) ()> proposal_list;

};




#endif
