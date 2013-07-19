// =====================================================================================
//
//       Filename:  ARC_Pair.cpp
//
//    Description:  Class for storing match data.
//
//        Version:  1.0
//        Created:  07/19/2013 02:51:25 PM
//       Revision:  none
//       Compiler:  gcc
//
//         Author:  Martin Miller (), miller7@illinois.edu
//   Organization:  Aerospace Robotics and Control Lab
//
// =====================================================================================

#include "ARC_Pair.hpp"

//--------------------------------------------------------------------------------------
//       Class:  ARC_Pair
//      Method:  ARC_Pair :: insert_sigma
// Description:  Inserts element in decreasing order. Returns index.
//--------------------------------------------------------------------------------------
    int
ARC_Pair::insert_sigma (  double s )
{
    if( sigmas.empty() )
    {
        sigmas.push_front( s );
        sigmas.pop_back();
        return 0;
    }
    size_t i = 0;
    list<double>::iterator it=sigmas.begin();
    while( it!=sigmas.end() )
    {
        if( s>*it )
        {
            sigmas.insert( it, s );
            sigmas.pop_back() ;
            return i;
        }
        ++i;
        ++it;
    }
    return -1 ;
}		// -----  end of method ARC_Pair::insert_sigma  ----- 

//--------------------------------------------------------------------------------------
//       Class:  ARC_Pair
//      Method:  ARC_Pair :: add_pair
// Description:  Adds pair of points and sigma to respective lists maintaining
// index correspondence. Returns index.
//--------------------------------------------------------------------------------------
    int
ARC_Pair::add_pair (  Point2f src,  Point2f ref,  double nsigma )
{
    int i;
    // Check if indices are consistent
    if( points.source.size()!=points.reflection.size()
            || points.source.size()!=sigmas.size() )
    {
        cerr << "\nERROR : ARC_Pair index failure." << endl;
        exit (EXIT_FAILURE);
    }
    i = insert_sigma( nsigma );
    cout << "insert sigma: " << i << endl;
    list<Point2f>::iterator sit=points.source.begin();
    list<Point2f>::iterator rit=points.reflection.begin();
    int j = 0;
    while( j++<i )
    {
        ++sit;
        ++rit;
    }
    points.source.insert( sit, src );
    points.source.pop_back();
    points.reflection.insert( rit, ref );
    points.reflection.pop_back();

    return i;
}		// -----  end of method ARC_Pair::add_pair  ----- 


    void
ARC_Pair::print_all ( )
{
    list<double>::iterator sit=sigmas.begin();
    list<Point2f>::iterator srcit=points.source.begin();
    list<Point2f>::iterator refit=points.reflection.begin();

    while( sit!=sigmas.end() )
    {
        cout << *srcit << spc
             << *refit << spc
             << *sit << endl;
        ++sit;
        ++srcit;
        ++refit;
    }
    return ;
}		// -----  end of method ARC_Pair::print_all  ----- 

