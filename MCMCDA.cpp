#include "MCMCDA.h"


MCMCDA()
{

        gen = new std::mt19937 ( rd() );

}

~MCMCDA()
{
        free gen;
}

// obviously add to the current window
void MCMCDA::add_Location ( int x, int y )
{
        // proposal_graph.sliding_window[proposal_graph.sliding_window.size()-1].pushback ( new Node ( x,y ) );
}


//is the node active
bool MCMCDA::Is_Active ( Node *n )
{


        for ( auto & x : n->in_edges ) {
                if ( x.active ==true ) {
                        return true;
                }
        }

        if ( n->active_out != 0 ) {
                return true;
        }

        return false;

}

// return a list of possible edges that dont lead to activated nodes.
// also ONLY leads to nodes that are in our proposal window
// this is used in our extend functions.
// basically new nodes can be added to our tracks
vector<Edge * > MCMCDA::Inactive_Nodes ( Node * n )
{
        vector<Edge * >  m;
        for ( auto & x : n->out_edges ) {
                if ( x->target->frame->time >= proposal_graph.sliding_window.size()-PROPOSAL_WINDOW_SIZE  && ( !Is_Active ( x->target ) || x->active ) ) {
                        m.push_back ( x );
                }
        }

        return m;
}

void MCMCDA::Propose_Deactivate ( Edge * e )
{

        if ( e->source->start_of_path == true ) {
                e->source->start_of_path = false;
                vector_erase ( proposal_graph.start_nodes,e->source );
        }
        e->proposed = true;
        e->active = false;
        e->source->saved_out =  e->source->active_out;
        e->source->active_out = 0;
        proposal_graph.proposal_edge_list.push_back ( e );

}

// ONLY DEACTiVATE messess with the saved out
//means we must deactivated teh current active edge first
void MCMCDA::Propose_Activate ( Edge * e )
{
        // if were already active
        if ( e->active == true ) {
                return;
        }

        e->active = true;
        // set source to save its out edge
        // delete target out path if there is outpath
        if ( e->source->active_out != 0 ) {
                // next node in path
                Edge * tmp = e->source->active_out->target->active_out;

                Propose_Deactivate ( e->source->active_out );

                while ( tmp != 0 ) {
                        Edge * tmp2 = tmp;// save current location
                        tmp = tmp->source->active_out->target->active_out;//advance to the next
                        MCMCDA::Propose_Deactivate ( tmp2 ); //deactivate saved
                }
        }

        e->source->active_out = e;
        proposal_graph.proposal_edge_list.push_back ( e );
}

// get the active tracks at time t
vector<Node*> MCMCDA::get_Tracks_At_T ( int t )
{

        vector<Node*> pts;

        for ( auto & pt: proposal_graph.sliding_window[t].frame ) {
                if ( Is_Active ( pt ) ) {
                        pts.push_back ( pt );
                }
        }

        return pts;
}


//finds an edge between two points/nodes
//n1 comes first temporally then two
Edge * nodes_2_Edge ( Node * n1, Node * n2 )
{

        for ( auto & e: n1->out_edges ) {
                if ( e->target == n2 ) {
                        return e;
                }
        }

        return 0;
}

// true means we didnt meet any reasons to reject the move.
// false means we should reject
bool MCMCDA::Extend ( Node * n )
{
        std::uniform_real_distribution<> dis ( 0.0,1.0 );
        int track_len = 0;
        vector<Edge * > valid_edges = Inactive_Nodes ( n );

        // if size is only equal to none
        if ( valid_edges.size() ==0 ) {
                return false;
        }

        while ( valid_edges.size() > 0 && dis ( *gen ) >  gamma ) {



                // if our only option is is one that one advanced forward to reassign.
                if ( valid_edges.size() ==1 && valid_edges[0].active == true ) {

                        valid_edges = Inactive_Nodes ( valid_edges[0]->target );

                } else {

                        Edge * e = 0;


                        std::uniform_real_distribution<> dis2 ( 0,valid_edges.size()-1 );

                        e = valid_edges[dis2 ( *gen )];

                        Propose_Activate ( e );
                        valid_edges = Inactive_Nodes ( e->target );
                        track_len++;
                }


        }
        // lets not calculate the probability :)
        if ( track_len == 0 ) {
                return false;
        }

        return true;
}

//returns false if creation chosen isn't possible or breaks some rules
bool MCMCDA::Birth_Move()
{
        //
        // the back end of our graph is were we are sampling from
        std::uniform_real_distribution<> dis ( proposal_graph.sliding_window.size()-PROPOSAL_WINDOW_SIZE,proposal_graph.sliding_window.size()-1 );
// the time position to birth a new track
        int r = dis ( *gen );

        vector<Node*> frame = proposal_graph.sliding_window[r];

        vector<Node *> in_active;

        for ( auto & x: proposal_graph.sliding_window[r] ) {

                if ( !Is_Active ( x ) ) {
                        in_active.push_back ( x );
                }

        }


        if ( in_active.size() == 0 ) {
                return false;
        }

        std::uniform_real_distribution<> dis2 ( 0,in_active.size()-1 );

        r = dis2 ( *gen );

        if ( Extend ( in_active[r] ) ) {

                in_active[r]->start_of_path = true;
                proposal_graph.start_nodes.push_back ( in_active[r] );
        }

        return false;
}

bool MCMCDA::Death_Move()
{
        if ( proposal_graph.start_nodes.size() == 0 ) {
                return false;
        }

        std::uniform_real_distribution<> dis2 ( 0,proposal_graph.start_nodes.size()-1 );

        int r = dis2 ( *gen );

        Edge * e = proposal_graph.start_nodes[r]->active_out;

        // can only propose deaths and births in new area PROPOSAL WINDOW
        if ( proposal_graph.start_nodes[r]->frame->time < proposal_graph.sliding_window.size()-PROPOSAL_WINDOW_SIZE ) {
                return false;
        }

        // remove our start from the list
        vector_erase ( proposal_graph.start_nodes,proposal_graph.start_nodes[r] );
// go through and propose deletion of tracks
        while ( e != 0 ) {

                Edge * tmp = e->source->active_out;
                Propose_Deactivate ( e );

                e = tmp;
        }



        return true;

}

int MCMCDA::track_Length ( Node * n )
{
        int i =0;

        while ( n ) {
                n = n ->active_out;
                i++;
        }

        return i;
}


// for our update move we choose a track and if it doesnt have a point within our proposal window return false
// else choose uniform at random the points after too extend
bool MCMCDA:: Update_Move()
{

        if ( proposal_graph.start_nodes.size() == 0 ) {
                return false;
        }

        std::uniform_real_distribution<> dis2 ( 0,proposal_graph.start_nodes.size()-1 );


        int r = dis2 ( *gen );// start node
        Node * n = proposal_graph.start_nodes[r];
        //r = dis ( *gen );// time

        //find on this path where we can start updating
        int start_index = 0;
        while ( n->active_out &&n->active_out->target->frame->time < proposal_graph.sliding_window.size()-PROPOSAL_WINDOW_SIZE ) {
                n = n->active_out->target;
                start_index++;
        }




        // now our n points to the node before the path jumps into our proposal window
        int len =  track_Length ( n );

        std::uniform_real_distribution<> dis ( start_index, len-1 );

        //we pick a spot to update.

        int index = dis ( *gen ) - start_index;

        // go to our new spot to extend
        while ( index ) {

                n = n->active_out->target;
                index--;
        }
        //since extend doesnt allow us to extend to the memmoy part of our paths and only
        // uses edges that lead to the propsal window areas we are fine to just call extend

        return Extend ( n );


}


//returns a list of tracks that are extendable and return ptrs to the end of them
vector<Node*> MCMCDA:: extendable_Tracks()
{
        //return
        vector<Node*> r_this;

        for ( auto & n: proposal_graph.start_nodes ) {
                while ( n->active_out ) {
                        n = n->active_out;
                }

                // this is cheapy because it still loops thru whole vector
                vector<Edges*> m = Inactive_Nodes ( n );
                if ( m.size() >0 ) {
                        r_this.push_back ( n );
                }

        }

        return r_this;
}

bool MCMCDA::Propose_Extension()
{
        vector<Node*> acceptable_tracks = extendable_Tracks();

        if ( acceptable_tracks.size() == 0 ) {
                return false;
        }

        std::uniform_real_distribution<> dis ( 0,acceptable_tracks.size()-1 );
        int r = dis ( *gen );
        // chose an radom track and a random poin

        Node * n = acceptable_tracks[r];


        return Extend ( n );

}


bool MCMCDA::Propose_Reduction()
{

        vector<Node*> first_mutable_node;

        for ( auto & n: proposal_graph.start_nodes ) {
                while ( n->frame->time < proposal_graph.sliding_window.size()-PROPOSAL_WINDOW_SIZE ) {
                        n = n->active_out;
                }
                if ( n != 0 ) {
                        first_mutable_node.push_back ( n );
                }

        }

        if ( first_mutable_node.size() == 0 ) {
                return false;
        }

        std::uniform_real_distribution<> dis ( 0 ,first_mutable_node.size()-1 );
        int r = dis ( *gen );
        // chose an radom track and a random point on that track
        Node * n = first_mutable_node[r];
        int len = track_Length ( n );
	  std::uniform_real_distribution<> dis ( 0 ,len-1 );
	  r = dis(*gen);
	  
	  while(r)
	  {
	    n = n->active_out->target;
	    r--;
	  }
	  
	  Propose_Deactivate(n);
	  
	  return true;
}


// attempt to switch tracks attach t1 to t2 +1 and vice versa
bool MCMCDA::Switch ( Node* t1, Node * t2 )
{
        Edge *  t1_NE = t1->active_out;
        Edge *  t2_NE = t2->active_out;

        if ( t1_NE ==0 || t2_NE ==0 ) {
                return false;
        }
        Node *  t1_N = t1->active_out->target;
        Node *  t2_N = t2->active_out->target;

        Edge *  t1_t2N = nodes_2_Edge ( t1,t2_N );
        Edge *  t2_t1N = nodes_2_Edge ( t2,t1_N );

        if ( t1_t2N ==0 || t2_t1N ==0 ) {
                return false;
        }

        Propose_Activate ( t1_t2N );
        Propose_Activate ( t1_t2N );
        return true;

}

bool MCMCDA::Propose_Switch()
{

        if ( proposal_graph.start_nodes.size() < 2 ) {
                return false;
        }

        std::vector<int> time_vec;
        // the range from which we can propose track switches
        // fill vec and shuffle. run through
        for ( int i =  proposal_graph.sliding_window.size()-PROPOSAL_WINDOW_SIZE ; i <= proposal_graph.sliding_window.size(); i++ ) {
                time_vec.push_back ( i );
        }

        //shuffle the vector

        std::random_shuffle ( time_vec.begin(),time_vec.end() );


        for ( auto & r: time_vec ) {

                vector<Node*> active_tracks = get_Tracks_At_T ( r );

                if ( active_tracks.size() >= 2 ) {



                        std::uniform_real_distribution<> dis2 ( 0,active_tracks.size()-1 );
                        int index_t1 = dis2 ( *gen );
                        Node * t1 = active_tracks[index_t1];

                        vector_erase ( active_tracks,t1 );

                        std::uniform_real_distribution<> dis2 ( 0,active_tracks.size()-1 );
                        int index_t2 = dis2 ( *gen );
                        Node * t2 = active_tracks[index_t2];
                        // Now we've acquired both tracks, attempt to switch
                        // get the next pt in the track
                        if ( Switch ( t1,t2 ) == true ) {
                                return true;
                        }
                }
        }

        return false;

}


//ix the Ld function. D = frame # * max speed increase.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            return true
