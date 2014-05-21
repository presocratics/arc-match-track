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
ARC_Pair::set_reflection ( cv::Mat img, cv::Rect r, cv::Size s )
{
    roi.reflection = convert_to_point( r, img, s );
    if( roi.reflection==cv::Point( -1, -1 ) )
        roi.reflection=r.tl()-0.5*cv::Point(s);
    return true;
}		/* -----  end of method ARC_Pair::set_reflection  ----- */

//--------------------------------------------------------------------------------------
//       Class:  ARC_Pair
//      Method:  ARC_Pair :: convert_to_point
// Description:  Returns the coordinate of a feature in the center of a rect bounded in
// the center by Size s.
//--------------------------------------------------------------------------------------
    cv::Point
ARC_Pair::convert_to_point ( cv::Rect r, cv::Mat& img, cv::Size s )
{
    std::vector<cv::Point> lv;
    cv::Rect little;
    cv::Mat gray;
    cv::Mat mask;

    little = r;
    little += 0.5*cv::Point( r.size()-s );
    little -= r.size() - s;

    cvtColor( img, gray, CV_BGR2GRAY );
    mask = cv::Mat::zeros( img.size(), CV_8UC1 );
    rectangle( mask, little, 255, CV_FILLED );

    goodFeaturesToTrack( gray, lv, 1, 0.01, 10, mask, 3, 0, 0.04);

    return ( lv.size()>0 ) ? lv[0] : cv::Point( -1, -1 ) ;
}		// -----  end of method ARC_Pair::convert_to_point  ----- 

//--------------------------------------------------------------------------------------
//       Class:  ARC_Pair
//      Method:  ARC_Pair
// Description:  constructor
//--------------------------------------------------------------------------------------
ARC_Pair::ARC_Pair ( cv::Rect first, cv::Rect second, double ns, cv::Mat img, bool* error )
{
    age=0;
    nNoMatch=0;
    cv::Point f, s;
    cv::Size inner_size( 10, 10 );
    f = convert_to_point( first, img, inner_size );
    s = convert_to_point( second, img, inner_size );
    if( f==cv::Point(-1, -1) || s==cv::Point(-1, -1) )
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
    last_good=roi;
    nsigma = ns;
    id = ++num;
    *error = false;
}  // -----  end of method ARC_Pair::ARC_Pair  (constructor)  ----- 

ARC_Pair::ARC_Pair ( cv::Point f, cv::Rect second, double ns, cv::Mat img, bool* error )
{
    age=0;
    nNoMatch=0;
    cv::Point s;
    cv::Size inner_size( 10, 10 );
    s = convert_to_point( second, img, inner_size );
    if( s==cv::Point(-1, -1) )
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
    last_good=roi;
    nsigma = ns;
    id = ++num;
    *error = false;
}  // -----  end of method ARC_Pair::ARC_Pair  (constructor)  ----- 

    std::ostream &
operator << ( std::ostream &os, const ARC_Pair &obj )
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
