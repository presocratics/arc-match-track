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

#define _USE_MATH_DEFINES
// A Matrix
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
    double match_ratio;
    int slope_dev;
    int num_regions;
    Size patch_size;
    float radius;
    unsigned int min_match_points;
    string video_filename;
    string text_filename;
    bool isRatio, isSym, isRansac, blur;

    void arguments();                           // Function to initialize defaults;
} arguments;

void change_frame_number( int slider, void* fn );
void change_patch_size( int slider, void* ps );
void change_num_regions( int slider, void* nr );
void change_slope_dev( int slider, void* sd );
void change_match_ratio( int slider, void* mr );
void slope_endpoints ( double theta, Point2f* ol );
bool slope_filter ( Point2f src_pt, Point2f ref_pt, double slope, int max_dev );

void update_regions ( Mat& frame, list<ARC_Pair>* pairs,
        unsigned int nregions, Size patch_size, double slope, double theta );
Rect update_roi ( Rect roi, vector<Point2f> pts );

void prune_keypoints ( vector<KeyPoint>* train_kpt, vector<KeyPoint>* query_kpt,
        vector<DMatch>& matches );

inline Point2f reflect_point ( Point2f point, Rect roi );

void keypoints_to_goodpoints ( vector<KeyPoint>& kpt_train, vector<KeyPoint>& kpt_query,
        vector<Point2f>* good_train, vector<Point2f>* good_query, 
        vector<DMatch>& matches, Rect roi, unsigned int direction );

Mat process_object ( Mat* frame, Rect roi, Mat* mask, bool blur );

void good_points_to_keypoints( vector<Point2f> train_pts, vector<KeyPoint>* train_kpt,
        vector<Point2f> query_pts, vector<KeyPoint>* query_kpt,
        vector<DMatch>* matches, double theta );

void draw_match_by_hand( Mat out_img, Mat* scene, 
        Mat* object, Rect sroi, Rect rroi,
        vector<Point2f>& source_points, 
        vector<Point2f>& reflection_points);

void transpose( vector<Point2f>& pts, Rect roi );

void extract_good_points( vector<Point2f>& in_train_pts, vector<Point2f>* out_train_pts,
        vector<Point2f>& in_query_pts, vector<Point2f>* out_query_pts,
        vector<DMatch>& matches );

void draw_match_by_hand( Mat* in_img, Mat* out_img, 
        Mat* object_img, Rect roi,
        vector<Point2f>* good_source_points,
        vector<Point2f>* good_reflection_points);

void help( string program_name );

void get_image_list(string filename, vector<string>* il);

bool get_regions(string filename, vector<ARC_Pair>* regions);

bool get_arguments(int argc, char** argv, arguments* a);

Mat get_masked_frame ( Rect roi, double slope, unsigned int dir, Mat* frame, Mat* mask );

bool track (Mat gray, Mat prev_gray, list<ARC_Pair>* pairs );
void pairs_to_points ( list<ARC_Pair>* pairs, vector<Point2f>* src, vector<Point2f>* ref );

#endif /* MAIN_H */
