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
    
    
    //add a new point in the current time event
    
    void add_location_current_event(int x, int y);
    
    
};




#endif