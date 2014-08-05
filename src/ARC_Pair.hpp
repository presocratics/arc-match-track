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
#include "ARC_Point_Pair.hpp"
#include "config.hpp"


// =====================================================================================
//        Class:  ARC_Pair
//  Description:  Stores match information.
// =====================================================================================
class ARC_Pair
{
    public:
        // ====================  LIFECYCLE     ======================================= 
        ARC_Pair ()
        {
            nsigma=0;
            id=++num;
            nNoMatch=0;
            age=0;
        }
        ARC_Pair ( const cv::Rect& first, const cv::Rect& second, const double ns, 
                const cv::Mat& img, bool* error );   // constructor 
        ARC_Pair ( const cv::Point& first, const cv::Point& second, const double ns );
        ARC_Pair ( const cv::Point& f, const cv::Rect& second, const double ns,
                const cv::Mat& img, bool* error ); // constructor

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
        static int num;
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
        // ====================  METHODS       ======================================= 
        cv::Point convert_to_point ( const cv::Rect& r, const cv::Mat& img, const cv::Size& s );

        // ====================  DATA MEMBERS  ======================================= 

}; // -----  end of class ARC_Pair  ----- 


#endif     /* -----  ARC_PAIR_H  ----- */
