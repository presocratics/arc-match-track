//Author: Simon Peter speter3@illinois.edu
#ifndef REFLECTION_TRACKER_H
#define REFLECTION_TRACKER_H
//#include </usr/include/opencv2/opencv.hpp>
//#include </usr/include/opencv/highgui.h>
#include <opencv2/highgui/highgui_c.h>
#include "ARC_Pair.hpp"

Mat get_masked_frame ( Rect roi, double slope, Mat* frame, Mat* mask );
bool rematch ( Mat frame, Size patchSize, ARC_Pair& pair, double slope );
void createTemplatesFromVector(Mat image, Size patchSize, vector<Point> *points, list<ARC_Pair> *outlist);
Rect findBestMatchLocation(double slope, Mat image,  Rect source_rect, double* nsigma, Mat mask );
void findReflections(Mat image, Size patchSize, double slope, list<ARC_Pair> *outlist);

//The following functions are meant to be called on by a user, the previous ones are only used by getReflections()

//GIVEN AN IMAGE AND A PATCHSIZE, AND SLOPE INFORMATION, PUTS A SEQUENCE OF REAL OBJECTS AND THEIR REFLECTED REGIONS IN outvector AS ARC_Pair's
int getReflections(Mat frame, Size patchSize, int numOfFeatures, double slope, list<ARC_Pair> &outlist);
//DISPLAYS THE RESULTS OF getReflections()
void displayReflectionMatches(Mat image, Size patchSize, double slope, double theta, list<ARC_Pair> *outlist);
//GIVEN slope INFORMATION,A source MAT AND A tmplte RECT, IT RETURNS A RECT OF THE REFLECTION
Rect findOneReflection(double slope,Mat source, Rect tmplte);
//GIVEN A source MAT, patchSize, slope INFORMATION, AND A BOOLEAN FLAG, IT TRIES TO RETURN A NEW GOOD FEATURE AND IT'S REFLECTION
ARC_Pair getOneReflectionPair(Mat image, int patchSize, double slope, bool *regionFound);
int getReflectionsPYR(Mat &image, Size outerPatchSize, Size innerPatchSize, double slope, double theta, list<ARC_Pair> &outlist);

void get_shorline_margin ( cv::Mat src, cv::Mat& dst, int iter );
void find_water ( cv::Mat src, cv::Mat& dst );
cv::Mat maskImage ( cv::Mat image, std::vector<cv::Point>& snake, cv::Scalar c );

struct outside_theta {
    outside_theta( double m, double d ){
        theta =m;
        dev = d;
    }
    bool operator() (const ARC_Pair& value ) 
    { 
        Point del = value.roi.reflection-value.roi.source;
        double match_theta = atan2( del.y, del.x );
        return( abs(theta-match_theta)>dev );
    }
    private:
    double theta;
    double dev;
};
struct overlap {
    overlap( Size p ): patchSize(p){}
    bool operator() (const ARC_Pair& value ) 
    { 
        Rect a, b;
        a = Rect( value.roi.source - 0.5*Point( patchSize ), patchSize );
        b = Rect( value.roi.reflection - 0.5*Point( patchSize ), patchSize );
        return ( (a&b).area()>0.3*patchSize.area() );
    }
    Size patchSize;
};
struct below_threshold {
    below_threshold( double t ): threshold(t){}
    bool operator() (const ARC_Pair& value ) { return( value.nsigma<threshold ); }
    private:
    double threshold;
};

struct within_shore {
    within_shore(Mat src) {
        cv::Mat water_mask;
        find_water(src,water_mask);
        cv::Mat edges;
        if( water_mask.size()!=cv::Size(0,0) )
            get_shorline_margin(water_mask,edges,64);
        shoreMask = edges.clone();
    }   
    bool operator() (const ARC_Pair& value)
    {   
        if(shoreMask.size()!=cv::Size(0,0))
        {   
            int sourcePixel = shoreMask.at<uchar>(value.roi.source.y,value.roi.source.x);
            int reflPixel = shoreMask.at<uchar>(value.roi.reflection.y,value.roi.reflection.x);
            //return true if either the source or reflection is outside the shoreline margin - that ARC_Pair will be thrown out
            return sourcePixel==0 || reflPixel==0;
        }   
        //if no shoreline was detected, return false so that ARC_Pairs aren't all thrown out
        return false;
    }   
    private:
    cv::Mat shoreMask;
};


		
#endif /*REFLECTION_TRACKER_H*/
