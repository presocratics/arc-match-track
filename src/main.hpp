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
const cv::Matx33d A( fx, 0, cx,
                  0, fy, cy,
                  0,  0,  1 );

typedef struct PPC_t {                          // Point Pair Conversion
    std::vector<cv::Point2f> source;
    std::vector<cv::Point2f> reflection;
} PPC ;

typedef struct arguments_t {
    unsigned int refresh_count;                 // Number of iterations before rematching.
    int debug;
    int verbosity;
    int show_match;
    int show_track;
    double theta_dev;
    int num_regions;
    cv::Size patch_size;
    std::string video_filename;
    std::string text_filename;
    int blur;
    bool features_before_track;
    int good_features_to_track;
    double eig;
    double std;
    double max_dist;
    int start_frame;

    void arguments();                           // Function to initialize defaults;
} arguments;

void change_good_features_to_track( int slider, void* gft );
void change_frame_number( int slider, void* fn );
void change_patch_size( int slider, void* ps );
void change_num_regions( int slider, void* nr );
void change_theta_dev( int slider, void* sd );

void slope_endpoints ( double theta, cv::Point2f* ol );

void update_regions ( cv::Mat& frame, std::list<ARC_Pair>* pairs,
        unsigned int nregions, cv::Size patch_size, double slope, 
        double theta, double eig );

void draw_match_by_hand( cv::Mat out_img, cv::Mat* scene, 
        cv::Mat* object, cv::Rect sroi, cv::Rect rroi,
        std::vector<cv::Point2f>& source_points, 
        std::vector<cv::Point2f>& reflection_points);

void help( std::string program_name );

cv::Mat mask_scene ( cv::Rect roi, cv::Mat& frame );

void get_image_list(std::string filename, std::vector<std::string>* il);

bool get_regions(std::string filename, std::vector<ARC_Pair>* regions);

bool get_arguments(int argc, char** argv, arguments* a);

bool track (cv::Mat gray, cv::Mat prev_gray, std::list<ARC_Pair>* pairs );
void pairs_to_points ( cv::Mat gray, std::list<ARC_Pair>* pairs, 
        std::vector<cv::Point2f>* src, std::vector<cv::Point2f>* ref,
        bool fbt );

#endif /* MAIN_H */
