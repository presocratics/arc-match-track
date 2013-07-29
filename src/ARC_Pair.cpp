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

    bool
ARC_Pair::set_reflection ( Mat img, Rect r, Size s )
{
    roi.reflection = convert_to_point( r, img, s );
    return ( roi.reflection!=Point( -1, -1 ) );
}		/* -----  end of method ARC_Pair::set_reflection  ----- */

//--------------------------------------------------------------------------------------
//       Class:  ARC_Pair
//      Method:  ARC_Pair :: convert_to_point
// Description:  Returns the coordinate of a feature in the center of a rect bounded in
// the center by Size s.
//--------------------------------------------------------------------------------------
    Point
ARC_Pair::convert_to_point ( Rect r, Mat& img, Size s )
{
    vector<Point> lv;
    Rect little;
    Mat gray;
    Mat mask;

    little = r;
    little += 0.5*Point( r.size()-s );
    little -= r.size() - s;

    cvtColor( img, gray, CV_BGR2GRAY );
    mask = Mat::zeros( img.size(), CV_8UC1 );
    rectangle( mask, little, 255, CV_FILLED );

    goodFeaturesToTrack( gray, lv, 1, 0.01, 10, mask, 3, 0, 0.04);

    return ( lv.size()>0 ) ? lv[0] : Point( -1, -1 ) ;
}		// -----  end of method ARC_Pair::convert_to_point  ----- 

//--------------------------------------------------------------------------------------
//       Class:  ARC_Pair
//      Method:  ARC_Pair
// Description:  constructor
//--------------------------------------------------------------------------------------
ARC_Pair::ARC_Pair ( Rect first, Rect second, double ns, Mat img, bool* error )
{
    age=0;
    nNoMatch=0;
    Point f, s;
    Size inner_size( 10, 10 );
    f = convert_to_point( first, img, inner_size );
    s = convert_to_point( second, img, inner_size );
    if( f==Point(-1, -1) || s==Point(-1, -1) )
    {
        *error = true;
        return;
    }
    if( f.y<s.y )
    {
        roi.source = f;
        roi.reflection = s;
    }
    else
    {
        roi.source = s;
        roi.reflection = f;
    }
    nsigma = ns;
    id = ++num;
    *error = false;
}  // -----  end of method ARC_Pair::ARC_Pair  (constructor)  ----- 

ARC_Pair::ARC_Pair ( Point f, Rect second, double ns, Mat img, bool* error )
{
    age=0;
    nNoMatch=0;
    Point s;
    Size inner_size( 10, 10 );
    s = convert_to_point( second, img, inner_size );
    if( s==Point(-1, -1) )
    {
        *error = true;
        return;
    }
    if( f.y<s.y )
    {
        roi.source = f;
        roi.reflection = s;
    }
    else
    {
        roi.source = s;
        roi.reflection = f;
    }
    nsigma = ns;
    id = ++num;
    *error = false;
}  // -----  end of method ARC_Pair::ARC_Pair  (constructor)  ----- 

    ostream &
operator << ( ostream &os, const ARC_Pair &obj )
{
    os << obj.id << spc
       << obj.roi.source << spc
       << obj.roi.reflection << spc
       << obj.nsigma << spc
       << obj.nNoMatch << spc
       << obj.age ;
    return os;
}		// -----  end of function operator <<  ----- 

int ARC_Pair::num=0;
