#ifndef  ARC_PAIR_H
#define ARC_PAIR_H

#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <cstdlib>

#include <cv.h>
#include "opencv2/core/core.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <ARC_Segment.hpp>
#include <ARC_Match.hpp>
#include <config.hpp>

using namespace cv;
using namespace std;


/*
 * =====================================================================================
 *        struct:  ARC_Pair
 *  Description:  
 * =====================================================================================
 */
typedef struct ARC_Pair_t
{
    unsigned int iter_count;
    struct {
        unsigned int match;
        unsigned int track;
    } direction;
    Rect roi;
    double slope;
    struct {
        vector<KeyPoint> reflection;
        vector<KeyPoint> source;
    } keypoints;
    //PPC good_points; // reference origin of scene.
    vector<DMatch> matches;
    unsigned int staleness;
} ARC_Pair; 



#endif     /* -----  ARC_PAIR_H  ----- */
