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
ARC_Pair::ARC_Pair ( Rect first, Rect second, double ns)
{
    if( first.y<second.y )
    {
        roi.source = first;
        roi.reflection = second;
    }
    else
    {
        roi.source = second;
        roi.reflection = first;
    }
    nsigma = ns;
}  // -----  end of method ARC_Pair::ARC_Pair  (constructor)  ----- 

    ostream &
operator << ( ostream &os, const ARC_Pair &obj )
{
    os << obj.roi.source.tl() << spc
       << (Point) obj.roi.source.size() << spc
       << obj.roi.reflection.tl() << spc
       << (Point) obj.roi.reflection.size() << spc
       << obj.nsigma ;
    return os;
}		// -----  end of function operator <<  ----- 
