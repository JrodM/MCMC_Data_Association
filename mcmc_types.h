#ifndef MCMC_TYPES_H
#define MCMC_TYPES_H

#include <stdlib.h>
#include <vector>
#include <deque>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#include <iostream>
#include <cmath>


#define WINDOW_SIZE 80

using namespace std;
using namespace cv;
struct point;
class Edge;
class Node;
class Time_Frame;

// shortcut having to write loops.
template <class T>
void vector_erase ( vector<T *> vec, T * erase_this )
{

    int iter = 0;
    typename vector<T *>::iterator i;

    for ( i = vec.begin(); i!= vec.end() ;  i++ ) {
        if ( *i == erase_this ) {
            break;
        }
        iter ++;
    }

    if ( iter == vec.size() ) {
        return;
    }

    vec.erase ( vec.begin() +iter );

    return;
}


/*
 *
 * Structures below
 *
 *
 *
 */


// keeps track of two points
 struct point {
    u_int8_t x;
    u_int8_t y;

    point() {
        x = 0;
        y = 0;
    }
    point ( u_int8_t x1,u_int8_t y1 ) {
        x= x1;
        y= y1;
    }

};



//represents a head location
class Node {
public:
    point location;
    unsigned int ID = 0;
// might not need this
    bool start_of_path = false;
// use pointers to prevent copies
    vector<Edge *> in_edges;
    vector<Edge *> out_edges;
    // the single active edge
     Edge * active_out=0;
     // this is to store the old active out for when we propose
     Edge * saved_out=0;
    //this is to do faster calculations for probablity less time searching the state space
   
    // is this node able to be changed or modified
     //bool is_mutable = true;
    
     //a pointer to the window frame the point is in. We can find the time in an O(1) fashion.
     Time_Frame * frame = 0;
     
     Node(int x, int y, Time_Frame * t)
     {
       location.x = x;
       location.y = y;
       frame = t;
     }
     
   /*  ~Node()
     {s
       
       for(vector<Edge *>::iterator i = out_edges.begin(); i != out_edges.end();i++)
       {
	  delete *i; 
       }
     }*/

};

//represents a POSSIBLE path between two temporal nodes in our TemporalGraph
class Edge {
public:
    // use pointers to prevent copies
    Node * source = 0;
    Node * target = 0;
    bool active = false;
    bool proposed = false;
    int time_distance = 0;
    
    Edge()
    {
    }

};

class Time_Frame{
public:
  Time_Frame()
  {
    
  }
  vector<Node *> frame;
  int time =0;
};





#endif
