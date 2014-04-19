#ifndef MAIN_H
#define MAIN_H
#include <iostream>
#include <ostream>
#include <string>
#include <vector>
#include <list>
#include <fstream>
#include <cstdlib>
#include <cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "ARC_Pair.hpp"
#include "ARC_Write.hpp"
#include "config.hpp"
#include "ReflectionTracker.hpp"
#include "ARC_IMU.hpp"
#include "ARC_Point_Pair.hpp"

#define _USE_MATH_DEFINES
// A Matrix
/* June Data */
/*
const double fx = 492.42317;
const double fy = 489.98014;
const double cx = 313.71144;
const double cy = 261.27213;
*/
/* Fall Data */
const double fx = 492.42317;
const double fy = 489.98014;
const double cx = 313.71144;
const double cy = 261.27213;
const Matx33d A( fx, 0, cx,
                  0, fy, cy,
                  0,  0,  1 );

typedef struct PPC_t {                          // Point Pair Conversion
    vector<Point2f> source;
    vector<Point2f> reflection;
} PPC ;

typedef struct arguments_t {
    unsigned int refresh_count;                 // Number of iterations before rematching.
    int debug;
    int verbosity;
    int show_match;
    int show_track;
    double theta_dev;
    int num_regions;
    Size patch_size;
    string video_filename;
    string text_filename;
    bool blur;
    bool features_before_track;
    int good_features_to_track;
    double eig;
    double std;
    double max_dist;

    void arguments();                           // Function to initialize defaults;
} arguments;

void change_good_features_to_track( int slider, void* gft );
void change_frame_number( int slider, void* fn );
void change_patch_size( int slider, void* ps );
void change_num_regions( int slider, void* nr );
void change_theta_dev( int slider, void* sd );

void slope_endpoints ( double theta, Point2f* ol );

void update_regions ( Mat& frame, list<ARC_Pair>* pairs,
        unsigned int nregions, Size patch_size, double slope, 
        double theta, double eig );

void draw_match_by_hand( Mat out_img, Mat* scene, 
        Mat* object, Rect sroi, Rect rroi,
        vector<Point2f>& source_points, 
        vector<Point2f>& reflection_points);

void help( string program_name );

Mat mask_scene ( Rect roi, Mat& frame );

void get_image_list(string filename, vector<string>* il);

bool get_regions(string filename, vector<ARC_Pair>* regions);

bool get_arguments(int argc, char** argv, arguments* a);

bool track (Mat gray, Mat prev_gray, list<ARC_Pair>* pairs );
void pairs_to_points ( Mat gray, list<ARC_Pair>* pairs, 
        vector<Point2f>* src, vector<Point2f>* ref,
        bool fbt );

#endif /* MAIN_H */
