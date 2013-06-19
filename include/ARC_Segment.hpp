#ifndef ARC_SEGMENT_H
#define ARC_SEGMENT_H


#include <iostream>
#include <vector>
#include <cv.h>
#include "opencv2/core/core.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/nonfree/features2d.hpp"

using namespace std;
using namespace cv;

typedef struct segment_t {
    // For Matcher
    Rect roi;
    Mat img, drawn_matches, descriptors;
    unsigned int match_count;
    vector<KeyPoint> keypoints;
    vector<DMatch> matches; 
    vector<vector<DMatch> > kmatches; 
    vector<DMatch> good_matches; 
    // For tracking
    Mat mask;
    vector<Point2f> points[2];
    vector<uchar> status;
    vector<float> error;
} segment;				// ----------  end of struct segment  ---------- 


#endif /* ARC_SEGMENT_H */

