#include "MCMCDA.h"
// obviously add to the current window
void MCMCDA::add_Location ( int x, int y )
{
        proposal_graph.sliding_window[0].pushback ( new Node ( x,y ) );
}