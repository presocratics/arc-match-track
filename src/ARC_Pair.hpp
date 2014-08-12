#ifndef  ARC_PAIR_H
#define ARC_PAIR_H

#include <iostream>
#include <string>
#include <vector>
#include <list>
#include <fstream>
#include <cstdlib>

#include <cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "config.hpp"


struct ARC_Point_Pair {
    ARC_Point_Pair() : source(-2,-2), reflection(-2,-2) {};
    ARC_Point_Pair(cv::Point s, cv::Point r) : source(s), reflection(r) {};
    cv::Point source, reflection;
};				/* ----------  end of struct ARC_Point_Pair  ---------- */

typedef struct ARC_Point_Pair ARC_Point_Pair;
// =====================================================================================
//        Class:  ARC_Pair
//  Description:  Stores match information.
// =====================================================================================
class ARC_Pair
{
    public:
        // ====================  LIFECYCLE     ======================================= 
        ARC_Pair () : nsigma(0), nNoMatch(0), id(++num), age(0), slot(0) {};
        ARC_Pair ( const cv::Rect& first, const cv::Rect& second, double ns, const cv::Mat& img, bool* error );   // constructor 
        ARC_Pair ( const cv::Point& f, const cv::Rect& second, double ns, const cv::Mat& img, bool* error ); // constructor
        ARC_Pair ( const cv::Point& f, const cv::Point& s, double ns ) : 
            roi(f,s), last_good(f,s), nsigma(ns), nNoMatch(0), id(++num), age(0), slot(0) {};

        // ====================  ACCESSORS     ======================================= 

        // ====================  MUTATORS      ======================================= 
        bool set_reflection( cv::Mat img, cv::Rect r, cv::Size s );

        // ====================  OPERATORS     ======================================= 

        friend std::ostream & operator << ( std::ostream &os, const ARC_Pair &obj );

        friend int operator <( const ARC_Pair& first, const ARC_Pair& second )
        {
            return ( first.nsigma < second.nsigma );
        }		

        friend int operator >( const ARC_Pair& first, const ARC_Pair& second )
        {
            return ( first.nsigma < second.nsigma );
        }		
        // ====================  DATA MEMBERS  ======================================= 
        ARC_Point_Pair roi, last_good;

        double nsigma;                    // Number of std above mean.
        unsigned int nNoMatch;
        unsigned int id;
        unsigned int age;
        unsigned int slot;

    protected:
        // ====================  METHODS       ======================================= 

        // ====================  DATA MEMBERS  ======================================= 

    private:
        static int num;
        // ====================  METHODS       ======================================= 
        cv::Point convert_to_point ( const cv::Rect& r, const cv::Mat& img, const cv::Size& s );

        // ====================  DATA MEMBERS  ======================================= 

}; // -----  end of class ARC_Pair  ----- 


#endif     /* -----  ARC_PAIR_H  ----- */
