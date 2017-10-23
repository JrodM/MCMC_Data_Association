#ifndef MCMCDA_H
#define MCMCDA_H
#include "Graph.h"



const cv::Scalar SCALAR_BLACK = cv::Scalar(0.0, 0.0, 0.0);
const cv::Scalar SCALAR_WHITE = cv::Scalar(255.0, 255.0, 255.0);
const cv::Scalar SCALAR_YELLOW = cv::Scalar(0.0, 255.0, 255.0);
const cv::Scalar SCALAR_GREEN = cv::Scalar(0.0, 200.0, 0.0);
const cv::Scalar SCALAR_RED = cv::Scalar(0.0, 0.0, 255.0);

class MCMCDA
{
public:
    Graph proposal_graph;

    // maximum posterior: this is the configuration the produces the highest p(w|Y)
    vector<vector<TNode>> MAP_estimate;
    float MAP_prob = 0.00;

    // may have to reconsider probability functions but this holds all unassigned nodes
  //  vector<vector<TNode *>> noise;

    std::random_device rd;  //Will be used to obtain a seed for the random number engine
    std::mt19937 * gen;
    const float gamma  = .001; // the chance of ending a track early
    
    
    /******************FUNCTIONS*******************/

    //add a new point in the current time event

    void add_location_current_event ( int x, int y );

    void Propose_Deactivate ( Edge * e );
    void Propose_Activate ( Edge * e );
    int track_Length ( TNode * n );
    vector<TNode*> get_Tracks_At_T(int t);
    Edge * nodes_2_Edge(TNode * n1, TNode * n2);
    vector<TNode*> extendable_Tracks();
    vector<tuple<TNode*,vector<Edge *>>> mergable_Vectors();
    vector<Edge * > Inactive_TNodes ( TNode *n );
    bool Is_Active ( TNode *n );
    void track_Start_Search();
    void Load_Tracks(vector<vector<TNode>> & load_this);
    
    //printing
    void drawEntityPaths ( Mat &image );
    

    //proposal moves
    bool Extend ( TNode * n );
    bool Birth_Move();
    bool Death_Move();
    bool Update_Move();
    bool Extension_Move();
    bool Reduction_Move();
    bool Switch(TNode* t1, TNode * t2);
    bool Switch_Move();
    bool Merge_Move();
    bool Split_Move();

    void Accept_Proposal();
    void Reject_Proposal();
    
    void Sampler();
    
    MCMCDA();
    ~MCMCDA();
    
/************************************************/
    //list of different proposal functions
    vector<bool (MCMCDA::*) ()> proposal_list;

};




#endif
