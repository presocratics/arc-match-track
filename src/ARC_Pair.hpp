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
#include "ARC_Match.hpp"
#include "config.hpp"

using namespace cv;
using namespace std;


// =====================================================================================
//        Class:  ARC_Pair
//  Description:  Stores match information.
// =====================================================================================
class ARC_Pair
{
    public:
        // ====================  LIFECYCLE     ======================================= 
        ARC_Pair ( );
        ARC_Pair ( Point2f src, Point2f ref, double ns );                             // constructor 

        // ====================  ACCESSORS     ======================================= 
        // ====================  MUTATORS      ======================================= 

        // ====================  OPERATORS     ======================================= 

        friend ostream & operator << ( ostream &os, const ARC_Pair &obj );

        friend int operator <( const ARC_Pair& first, const ARC_Pair& second )
        {
            return ( first.nsigma < second.nsigma );
        }		

        friend int operator >( const ARC_Pair& first, const ARC_Pair& second )
        {
            return ( first.nsigma < second.nsigma );
        }		
        // ====================  DATA MEMBERS  ======================================= 
        struct {
            Point2f source;
            Point2f reflection;
        } points;
        double nsigma;                    // Number of std above mean.

    protected:
        // ====================  METHODS       ======================================= 

        // ====================  DATA MEMBERS  ======================================= 

    private:
        // ====================  METHODS       ======================================= 

        // ====================  DATA MEMBERS  ======================================= 

}; // -----  end of class ARC_Pair  ----- 

/*
 * =====================================================================================
 *        struct:  ARC_Pair
 *  Description:  Stores match information for a source-reflection pair.
 * =====================================================================================
typedef struct ARC_Pair_t
{
    unsigned int iter_count, no_match;
    struct {
        unsigned int match;
        unsigned int track;
    } direction;
    struct {
        Rect source;
        Rect reflection;
    } roi;
    double slope;
    struct {
        vector<KeyPoint> reflection;
        vector<KeyPoint> source;
    } keypoints;
    vector<DMatch> matches;
} ARC_Pair; 
 */



#endif     /* -----  ARC_PAIR_H  ----- */
