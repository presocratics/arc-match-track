//Author: Simon Peter speter3@illinois.edu
#ifndef REFLECTION_TRACKER_H
#define REFLECTION_TRACKER_H
//#include </usr/include/opencv2/opencv.hpp>
//#include </usr/include/opencv/highgui.h>
#include <opencv2/highgui/highgui_c.h>
#include "ARC_Pair.hpp"

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
struct outside_theta {
    outside_theta( double m ): theta(m){}
    bool operator() (const ARC_Pair& value ) 
    { 
        Point del = value.roi.reflection.tl()-value.roi.source.tl();
        //double match_theta = (del.x==0) ? M_PI/2 : atan2(del.y/del.x);
        double match_theta = atan2( del.y, del.x );
        //cout << "Match theta: " << match_theta << endl;
        //cout << "Diff theta: " << abs(match_theta-theta) << endl;
        return( abs(theta-match_theta)>0.10 );
        //return( abs(theta-match_theta)>0.30 );
        //return( abs(theta-match_theta)>0.60 );
    }
    private:
    double theta;
};
struct overlap {
    //overlap( double t ): threshold(t){}
    bool operator() (const ARC_Pair& value ) { return( (value.roi.source&value.roi.reflection).area()>0 ); }
};
struct below_threshold {
    below_threshold( double t ): threshold(t){}
    bool operator() (const ARC_Pair& value ) { return( value.nsigma<threshold ); }
    private:
    double threshold;
};
#endif /*REFLECTION_TRACKER_H*/
