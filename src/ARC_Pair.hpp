#ifndef  ARC_PAIR_H
#define ARC_PAIR_H

#include <iostream>
#include <string>
#include <vector>
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
using std::vector;


/*
 * =====================================================================================
 *        struct:  ARC_Pair
 *  Description:  Stores match information for a source-reflection pair.
 * =====================================================================================
 */
typedef struct ARC_Pair_t
{
    unsigned int iter_count;
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



#endif     /* -----  ARC_PAIR_H  ----- */
