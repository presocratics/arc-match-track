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
        //ARC_Pair ();                             // constructor 

        // ====================  ACCESSORS     ======================================= 
        void print_all();
        unsigned int get_max_size()
        {
            return max_size;
        }

        unsigned int get_current_size()
        {
            return points.source.size();
        }

        double get_max_sigma()
        {
            return ( sigmas.empty() ) ? 0 : sigmas.front();
        }

        double get_min_sigma()
        {
            return ( sigmas.empty() ) ? 0 : sigmas.back();
        }
        // ====================  MUTATORS      ======================================= 
        void set_max_size( unsigned int ms )
        {
            max_size = ms;
            points.source.resize( ms );
            points.reflection.resize( ms );
            sigmas.resize( ms );
        }

        int add_pair( Point2f src, Point2f ref, double nsigma );

        // ====================  OPERATORS     ======================================= 

    protected:
        // ====================  METHODS       ======================================= 

        // ====================  DATA MEMBERS  ======================================= 

    private:
        // ====================  METHODS       ======================================= 
        int insert_sigma( double s );

        // ====================  DATA MEMBERS  ======================================= 
        unsigned int max_size;                  // If specified lists will not exceed size.
        struct {
            list<Point2f> source;
            list<Point2f> reflection;
        } points;
        list<double> sigmas;                    // Number of std above mean.

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
