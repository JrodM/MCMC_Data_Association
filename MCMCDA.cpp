#include "MCMCDA.h"


MCMCDA::MCMCDA()
{
        /*for ( int i = 0 ; i<proposal_list.size(); i++ ) { // also there is out of bounds access (usage of <=) in your code
                ( this->*proposal_list[i] ) (); // or (*this.*callist[i])();
        }
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
        */
        proposal_list.push_back ( &MCMCDA::Birth_Move );
        proposal_list.push_back ( &MCMCDA::Death_Move );
        proposal_list.push_back ( &MCMCDA::Update_Move );
        proposal_list.push_back ( &MCMCDA::Extension_Move );
        proposal_list.push_back ( &MCMCDA::Reduction_Move );
        proposal_list.push_back ( &MCMCDA::Switch_Move );
        proposal_list.push_back ( &MCMCDA::Merge_Move );
        proposal_list.push_back ( &MCMCDA::Split_Move );

        gen = new std::mt19937 ( rd() );

}

MCMCDA::~MCMCDA()
{
        free ( gen );
}

// obviously add to the current window
/*void MCMCDA::add_Location ( int x, int y )
{
        // proposal_graph.sliding_window[proposal_graph.sliding_window.size()-1].pushback ( new TNode ( x,y ) );
}*/

void MCMCDA::track_Start_Search()
{
        proposal_graph.start_nodes.clear();

        for ( auto & fra : proposal_graph.sliding_window ) {
                for ( auto & pt : fra.frame ) {
                        if ( pt->active_out ) {
                                bool is_start = true;;

                                for ( auto & x : pt->in_edges ) {
                                        if ( x->active ==true ) {
                                                is_start = false;
                                                break;

                                        }
                                }


                                if ( is_start ) {
                                        proposal_graph.start_nodes.push_back ( pt );
                                        pt->start_of_path = true;
                                }
                        }
                }
        }

}

void MCMCDA::drawEntityPaths ( Mat &image )
{
        cout<<"drawing Paths "<< MAP_estimate.size() <<endl;

        for ( vector<vector<TNode>>::iterator i = MAP_estimate.begin(); i!= MAP_estimate.end(); i++ ) {
                for ( vector<TNode>::iterator j = ( *i ).begin(); j+1 != ( *i ).end(); j++ ) {
                        line ( image, Point ( ( *j ).location.x, ( *j ).location.y ), Point ( ( * ( j+1 ) ).location.x, ( * ( j+1 ) ).location.y ), SCALAR_GREEN, 2 );

                }

        }
}
//is the node active
bool MCMCDA::Is_Active ( TNode *n )
{


        for ( auto & x : n->in_edges ) {
                if ( x->active ==true ) {
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
vector<Edge * > MCMCDA::Inactive_TNodes ( TNode * n )
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

        /*if ( e->source->start_of_path == true ) {
                e->source->start_of_path = false;

                vector_erase ( proposal_graph.start_nodes,e->source );
                cout<<"destroyed the start track"<<endl<<proposal_graph.start_nodes.size() <<endl;
        }*/

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
        e->proposed = true;
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

        /*if ( e->target->start_of_path == true ) {
                e->target->start_of_path = false;
                e->source->start_of_path = true;
                vector_erase ( proposal_graph.start_nodes,e->target );
        }*/

        proposal_graph.proposal_edge_list.push_back ( e );
        cout<<"Proposing "<< e->source->frame->time<< "<=From | To=> " << e->target->frame->time<< " " << e->source->active_out<<endl;
}

// get the active tracks at time t
vector<TNode*> MCMCDA::get_Tracks_At_T ( int t )
{

        vector<TNode*> pts;

        for ( auto & pt: proposal_graph.sliding_window[t].frame ) {
                if ( Is_Active ( pt ) ) {
                        pts.push_back ( pt );
                }
        }

        return pts;
}


//finds an edge between two points/nodes
//n1 comes first temporally then two
Edge * MCMCDA:: nodes_2_Edge ( TNode * n1, TNode * n2 )
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
bool MCMCDA::Extend ( TNode * n )
{
        std::uniform_real_distribution<> dis ( 0.0,1.0 );
        int track_len = 0;
        vector<Edge * > valid_edges = Inactive_TNodes ( n );

        // if size is only equal to none
        if ( valid_edges.size() ==0 ) {
                return false;
        }

        while ( valid_edges.size() > 0 && dis ( *gen ) >  gamma ) {



                // if our only option is is one that one advanced forward to reassign.
                if ( valid_edges.size() ==1 && valid_edges[0]->active == true ) {

                        valid_edges = Inactive_TNodes ( valid_edges[0]->target );

                } else {

                        Edge * e = 0;


                        std::uniform_real_distribution<> dis2 ( 0,valid_edges.size()-1 );

                        e = valid_edges[dis2 ( *gen )];

                        Propose_Activate ( e );
                        valid_edges = Inactive_TNodes ( e->target );
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
        cout<<"Birth Move "<<endl;
        //
        // the back end of our graph is were we are sampling from
        std::uniform_real_distribution<> dis ( proposal_graph.sliding_window.size()-PROPOSAL_WINDOW_SIZE,proposal_graph.sliding_window.size()-1 );
// the time position to birth a new track
        int r = dis ( *gen );

        vector<TNode*> frame = proposal_graph.sliding_window[r].frame;

        vector<TNode *> in_active;

        for ( auto & x: proposal_graph.sliding_window[r].frame ) {

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

                return true;
        }

        return false;
}

bool MCMCDA::Death_Move()
{

        cout<<"Death Move "<<endl;
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

int MCMCDA::track_Length ( TNode * n )
{
        int i =1;

        while ( n->active_out ) {
                n = n ->active_out->target;
                i++;
        }

        return i;
}


// for our update move we choose a track and if it doesnt have a point within our proposal window return false
// else choose uniform at random the points after too extend
bool MCMCDA:: Update_Move()
{
        cout<<"Update Move "<<endl;
        if ( proposal_graph.start_nodes.size() == 0 ) {
                return false;
        }

        std::uniform_real_distribution<> dis2 ( 0,proposal_graph.start_nodes.size()-1 );


        int r = dis2 ( *gen );// start node
        TNode * n = proposal_graph.start_nodes[r];
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
vector<TNode*> MCMCDA:: extendable_Tracks()
{
        //return
        vector<TNode*> r_this;

        for ( auto & n: proposal_graph.start_nodes ) {
                while ( n->active_out ) {
                        n = n->active_out->target;
                }

                // this is cheapy because it still loops thru whole vector
                vector<Edge *> m = Inactive_TNodes ( n );
                if ( m.size() >0 ) {
                        r_this.push_back ( n );
                }

        }

        return r_this;
}

bool MCMCDA::Extension_Move()
{
        cout<<"Extension Move "<<endl;
        vector<TNode*> acceptable_tracks = extendable_Tracks();

        if ( acceptable_tracks.size() == 0 ) {
                return false;
        }

        std::uniform_real_distribution<> dis ( 0,acceptable_tracks.size()-1 );
        int r = dis ( *gen );
        // chose an radom track and a random poin

        TNode * n = acceptable_tracks[r];


        return Extend ( n );

}


bool MCMCDA::Reduction_Move()
{
        cout<<"Reduction Move "<<endl;
        vector<TNode*> first_mutable_node;

        for ( auto & n: proposal_graph.start_nodes ) {
                while ( n->frame->time < proposal_graph.sliding_window.size()-PROPOSAL_WINDOW_SIZE ) {
                        if ( n->active_out == 0 ) {
                                n =0;
                                break;
                        }
                        cout<< "Seg: Reduction: "<<n->active_out->target<<endl;
                        n = n->active_out->target;
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
        TNode * n = first_mutable_node[r];

        cout<<"Length acquiting"<<endl;
        int len = track_Length ( n );

        int active_in_len = 0;
        // find the depth of the nodes behind it. We need  length of 3 to reduce
        for ( auto & ed : n->in_edges ) {
                if ( ed->active ==true ) {
                        Edge * eds = ed;
                        active_in_len = 1;
                        for ( auto & e2 : eds->source->in_edges ) {
                                if ( e2->active ==true ) {
                                        active_in_len = 2;
                                        break;
                                }
                        }

                        break;
                }
        }

        if ( len + active_in_len <3 ) {
                return false;
        }

        std::uniform_real_distribution<> dis2 ( 0 ,len-1 );
        r = dis2 ( *gen );
        cout<<"DONE Length acquiting "<<len<<" "<<r<<endl;

        TNode * temp = n;
        while ( r ) {
                temp = n;
                n = n->active_out->target;
                r--;
        }
        cout<<"Done searching"<< endl;
        Propose_Deactivate ( temp->active_out );
        cout<<endl<<"END Reduction Move "<<endl;
        return true;
}


// attempt to switch tracks attach t1 to t2 +1 and vice versa
bool MCMCDA::Switch ( TNode* t1, TNode * t2 )
{
        Edge *  t1_NE = t1->active_out;
        Edge *  t2_NE = t2->active_out;

        if ( t1_NE ==0 || t2_NE ==0 ) {
                return false;
        }
        TNode *  t1_N = t1->active_out->target;
        TNode *  t2_N = t2->active_out->target;

        Edge *  t1_t2N = nodes_2_Edge ( t1,t2_N );
        Edge *  t2_t1N = nodes_2_Edge ( t2,t1_N );

        if ( t1_t2N ==0 || t2_t1N ==0 ) {
                return false;
        }

        Propose_Activate ( t1_t2N );
        Propose_Activate ( t1_t2N );
        return true;

}

bool MCMCDA::Switch_Move()
{
        cout<<"Switch Move "<<endl;
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

                vector<TNode*> active_tracks = get_Tracks_At_T ( r );

                if ( active_tracks.size() >= 2 ) {



                        std::uniform_real_distribution<> dis2 ( 0,active_tracks.size()-1 );
                        int index_t1 = dis2 ( *gen );
                        TNode * t1 = active_tracks[index_t1];

                        vector_erase ( active_tracks,t1 );

                        std::uniform_real_distribution<> dis1 ( 0,active_tracks.size()-1 );
                        int index_t2 = dis1 ( *gen );
                        TNode * t2 = active_tracks[index_t2];
                        // Now we've acquired both tracks, attempt to switch
                        // get the next pt in the track
                        if ( Switch ( t1,t2 ) == true ) {
                                return true;
                        }
                }
        }

        return false;

}

// return a list of the end of the tracks who cotain edges that lead to the start of a track.
// the vector of edges accounts for more then one possible track merge from the same track
// algorithm O N ^3
vector<tuple<TNode*,vector<Edge *>>> MCMCDA::mergable_Vectors()
{
        TNode* n = 0;
        vector<Edge *> edge_list;
        vector<tuple<TNode*,vector<Edge *>>> r_this;
        for ( auto & n_temp: proposal_graph.start_nodes ) {

                //  int i=0;
                n = n_temp;
                while ( n->active_out ) {
                        // i++;
                        n = n->active_out->target;
                }
                edge_list.clear();
                for ( auto & e: n->out_edges ) {
                        for ( auto & start_n: proposal_graph.start_nodes ) {
                                if ( e->target == start_n && start_n->frame->time < proposal_graph.sliding_window.size()-PROPOSAL_WINDOW_SIZE ) {
                                        edge_list.push_back ( e );
                                }
                        }
                }
                // now we have our edge list if its greater then zero keep track of it.
                if ( edge_list.size() != 0 ) {
                        r_this.push_back ( make_tuple ( n,edge_list ) );
                }
        }
        return r_this;
}

//rememeber to set start of path to false;
bool MCMCDA::Merge_Move()
{
        cout<<"Sampling Merge Move "<<endl;
        vector<std::tuple<TNode*,vector<Edge *>>> options = mergable_Vectors();

        if ( options.size() == 0 ) {
                return false;
        }

        std::uniform_real_distribution<> dis ( 0,options.size()-1 );
        int r = dis ( *gen );
        TNode *n1 = get<0> ( options[r] );
        vector<Edge *> e = get<1> ( options[r] );

        std::uniform_real_distribution<> dis2 ( 0,e.size()-1 );
        r = dis2 ( *gen );
        Edge * proposal_e = e[r];

        Propose_Activate ( proposal_e );

        return true;
        // taken care of now when we propose proposal_e->target->start_of_path =false;

}

bool MCMCDA::Split_Move()
{
        cout<<"Sampling Split Move "<<endl;
        // find all of  the tracks that meet the criteria for a split move
        // use tuple so we dont have to go find length again
        vector<std::tuple<TNode*,int>> split_tracks;
        for ( auto & n: proposal_graph.start_nodes ) {
                int len = track_Length ( n );
                if ( len >3 ) {
                        split_tracks.push_back ( std::make_tuple ( n,len ) );
                }
        }

        if ( split_tracks.size() == 0 ) {
                return false;
        }

        std::uniform_real_distribution<> dis2 ( 0, split_tracks.size() );
        int r = dis2 ( *gen );
        TNode * n = std::get<0> ( split_tracks[r] );
        int len = std::get<1> ( split_tracks[r] );

        std::uniform_real_distribution<> dis ( 2,len-2 );

        r = dis ( *gen );
        TNode * temp;
        while ( r ) {
                temp = n;
                n = n->active_out->target;
                r--;

        }

        n->start_of_path = true;
        // temp->active_out= 0;
        temp->active_out->proposed = true;
        temp->active_out->active = false;
        temp->saved_out =   temp->active_out;
        temp->active_out = 0;
        proposal_graph.proposal_edge_list.push_back ( temp->active_out );
        return true;

}
//ix the Ld function. D = frame # * max speed increase.


void MCMCDA::Accept_Proposal()
{
        cout<<endl<<endl<<"ACCEPTING PROPOSAL"<<endl<<endl;

        for ( auto & e : proposal_graph.proposal_edge_list ) {
                e->proposed = false;
        }

        proposal_graph.proposal_edge_list.clear();

}

void MCMCDA::Reject_Proposal()
{

        for ( auto & e : proposal_graph.proposal_edge_list ) {
                // if active
                if ( e->active ==true ) {

                        // e->source -> start_of_path =false;
                        e->source->active_out = e->source->saved_out;
                        e->source->saved_out =0;

                } else {
                        // if the first node is the start of the track destroy it
                        // if it destroys the track
                        /* if ( e->source->start_of_path == true ) {
                                 vector_erase ( proposal_graph.start_nodes,e->source );
                         }

                         // if rejecting from a merge move
                         if ( track_Length ( e->source ) >1 ) {
                                 e->target->start_of_path = true;
                                 proposal_graph.start_nodes.push_back ( e->target );
                         }*/

                        e->source->active_out = e;

                }


                e->active = !e->active;

        }

        proposal_graph.proposal_edge_list.clear();

}

void MCMCDA::Sampler()
{
        vector<TNode> m;
        float previous_prob = 0;
        vector<vector<TNode>>previous_config;
        // if theres no observations then we should assign a probabilty of zero.
        // another function that should be added in is if there arent any nodes we have to force a birth move.

        track_Start_Search();

        if ( proposal_graph.total_observations == 0 ) {

                // previous_config.clear();
                MAP_estimate.clear();
                MAP_prob = 0;
                //previous_prob = 0.0001f;

                cout<<"NOTHING TO VIEW"<<endl;
                return;
        }

        // this is for intializing
        if ( proposal_graph.total_observations<2 ) {
                return;
        } else  if ( proposal_graph.start_nodes.size() == 0 ) {

                for ( int i = 0 ; i < PROPOSAL_WINDOW_SIZE-1; i++ ) {
                        if ( Birth_Move() ) {
                                cout<<"Succesful Birth"<<endl;
                                // Accept_Proposal();
                                break;

                        }

                        Reject_Proposal();

                        //if we could find anything quit
                        if ( i == PROPOSAL_WINDOW_SIZE-2 ) {
                                return;
                        }
                }


                previous_config.clear();
                MAP_estimate.clear();

                MAP_prob = previous_prob = proposal_graph.Posterior();

                track_Start_Search();

                for ( auto & n : proposal_graph.start_nodes ) {
                        TNode * trvs = n;

                        m.clear();

                        while ( trvs != 0 ) {
                                // cout<<"#1.1 "<< trvs->active_out<<endl;
                                m .push_back ( TNode ( trvs ) );
                                //cout<<"#1.2 "<< trvs->active_out->target<<endl;
                                if ( trvs->active_out == 0 ) {
                                        break;
                                }

                                trvs = trvs->active_out->target;

                        }

                        cout<<"#2"<<endl;
                        //cvWaitKey(100);
                        previous_config.push_back ( m );
                }

                MAP_estimate = previous_config;
                Accept_Proposal();
                cout<<"#3"<<endl;
        } else {

                // this is loading the previous optimal state as well as the memory / immutable part of the graph
                previous_config = MAP_estimate;
                previous_config = MAP_prob;

                //start the graph over
                Edge * erase = 0;
                for ( auto & n: proposal_graph.start_nodes ) {
                        erase = n->active_out;
                        while ( erase->target->active_out ) {
                                Propose_Deactivate ( erase );
                                erase = erase ->target->active_out;
                        }
                        Propose_Deactivate ( erase );
                }


                /* Now reintialize based of the MAP estimate
		 
		 Down low we need a case to update the MAP
		 */

                }

                /*else if
                *
                * Load initial state as MAP if there is one.
                *
                * This is handled at the end of the Sampler. We have our MAP set to clear out. At
                * the end of the sampler we then set our graph back to our MAP for the next sampler call.
                *
                * If we are this far use an else then we can assume we have a preious config (since startnodes >0)
                * and user our map prob as our previous prob and our
                */

                std::uniform_real_distribution<> dis ( 0, proposal_list.size()-1 );
                std::uniform_real_distribution<> dis2 ( 0.0, 1.0 );
                // at the end of our loop we have to set W to the max configurations
                for ( int i = 0 ; i < 1000; i++ ) {

                        track_Start_Search();

                        int r = dis ( *gen );

                        // call a proposal function aka a move
                        bool sucess = ( this->*proposal_list[r] ) ();
                        cout<<"Sampling: "<<sucess<<endl;
                        // if the move is valid
                        if ( sucess ) {

                                track_Start_Search();
                                float current_config_prob = proposal_graph.Posterior();


                                if ( MAP_prob == 0.0 ) {
                                        // accept the configuration since we dont have one to compare against
                                        MAP_estimate.clear();
                                        for ( auto & n : proposal_graph.start_nodes ) {

                                                TNode * trvs = n;

                                                m.clear();
                                                while ( trvs != 0 ) {
                                                        // cout<<"#1.1 "<< trvs->active_out<<endl;
                                                        m .push_back ( TNode ( trvs ) );
                                                        //cout<<"#1.2 "<< trvs->active_out->target<<endl;
                                                        if ( trvs->active_out == 0 ) {
                                                                break;
                                                        }

                                                        trvs = trvs->active_out->target;

                                                }
                                                MAP_estimate.push_back ( m );
                                        }
                                        Accept_Proposal();
                                } else {
                                        //mcmc simulated annnealing with a constant temp value
                                        if ( dis2 ( *gen ) < std::min ( 1.0f, ( current_config_prob/MAP_prob ) ) ) {

                                                // accept the configuration since we dont have one to compare against
                                                MAP_estimate.clear();
                                                MAP_prob = current_config_prob;

                                                for ( auto & n : proposal_graph.start_nodes ) {
                                                        TNode * trvs = n;

                                                        m.clear();

                                                        while ( trvs != 0 ) {
                                                                // cout<<"#1.1 "<< trvs->active_out<<endl;
                                                                m .push_back ( TNode ( trvs ) );
                                                                //cout<<"#1.2 "<< trvs->active_out->target<<endl;
                                                                if ( trvs->active_out == 0 ) {
                                                                        break;
                                                                }

                                                                trvs = trvs->active_out->target;

                                                        }
                                                        MAP_estimate.push_back ( m );
                                                }
                                                //MAP_estimate.push_back ( m );
                                                Accept_Proposal();
                                        } else {
                                                cout<<"NO"<<endl;
                                                Reject_Proposal();
                                        }
                                }

                        }


                }





        }

        /* next step is to fix move and MAP */
