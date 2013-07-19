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
//      Method:  ARC_Pair
// Description:  constructor
//--------------------------------------------------------------------------------------
ARC_Pair::ARC_Pair ( Point2f src, Point2f ref, double ns)
{
    points.source = src;
    points.reflection = ref;
    nsigma = ns;
}  // -----  end of method ARC_Pair::ARC_Pair  (constructor)  ----- 

    ostream &
operator << ( ostream &os, const ARC_Pair &obj )
{
    os << obj.points.source << spc
       << obj.points.reflection << spc
       << obj.nsigma ;
    return os;
}		// -----  end of function operator <<  ----- 
