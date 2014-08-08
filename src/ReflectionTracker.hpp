//Author: Simon Peter speter3@illinois.edu
#ifndef REFLECTION_TRACKER_H
#define REFLECTION_TRACKER_H
//#include </usr/include/opencv2/opencv.hpp>
//#include </usr/include/opencv/highgui.h>
#include <opencv2/highgui/highgui_c.h>
#include "ARC_Pair.hpp"

cv::Mat get_masked_frame ( cv::Rect roi, double slope, cv::Mat* frame, cv::Mat* mask );
void createTemplatesFromVector(cv::Mat image, cv::Size patchSize, std::vector<cv::Point> *points,
        std::list<ARC_Pair> *outlist);
cv::Rect findBestMatchLocation( const cv::Mat& image, const cv::Rect& source_rect, 
        double* nsigma, const cv::Mat& mask );
void findReflections( const cv::Mat& image, cv::Size patchSize, double slope, 
        std::list<ARC_Pair> *outlist);

//The following functions are meant to be called on by a user, the previous ones
//are only used by getReflections()

//GIVEN AN IMAGE AND A PATCHSIZE, AND SLOPE INFORMATION, PUTS A SEQUENCE OF REAL
//OBJECTS AND THEIR REFLECTED REGIONS IN outvector AS ARC_Pair's
int getReflections( const cv::Mat& frame, const cv::Size& patchSize,
        std::list<ARC_Pair>& outlist, const std::vector<cv::Point2f>& gft );

int getReflectionsPYR(cv::Mat &image, cv::Size outerPatchSize, cv::Size innerPatchSize, 
        double slope, double theta, std::list<ARC_Pair> &outlist);

cv::Mat maskImage ( cv::Mat image, std::vector<cv::Point>& snake, cv::Scalar c );

struct outside_theta {
    outside_theta( double m, double d ){
        theta =m;
        dev = d;
    }
    bool operator() (const ARC_Pair& value ) 
    { 
        cv::Point2f del = value.roi.reflection-value.roi.source;
        double match_theta = atan2( del.y, del.x );
        return( fabs(theta-match_theta)>dev );
    }
    private:
    double theta;
    double dev;
};
struct overlap {
    overlap( cv::Size p ): patchSize(p){}
    bool operator() (const ARC_Pair& value ) 
    { 
        cv::Rect a, b;
        a = cv::Rect( value.roi.source - 0.5*cv::Point( patchSize ), patchSize );
        b = cv::Rect( value.roi.reflection - 0.5*cv::Point( patchSize ), patchSize );
        return ( (a&b).area()>0.3*patchSize.area() );
    }
    cv::Size patchSize;
};
struct below_threshold {
    below_threshold( double t ): threshold(t){}
    bool operator() (const ARC_Pair& value ) { return( value.nsigma<threshold ); }
    private:
    double threshold;
};

struct longer_than {
    longer_than( double l): length(l){}
    bool operator() (const ARC_Pair& value ) 
    { 
        cv::Point del = value.roi.source-value.roi.reflection;
        return (del.x*del.x+del.y*del.y)>length*length;
    }
    private:
    double length;
};
#endif /*REFLECTION_TRACKER_H*/
