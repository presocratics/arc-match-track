#ifndef MAIN_H
#define MAIN_H
#include <ourerr.hpp>
#include <Quaternion.hpp>
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

#define _USE_MATH_DEFINES

struct configuration {
    cv::Matx33d k;
    double kc[5];
    Quaternion camIMU;
};
typedef struct configuration Configuration;

typedef struct PPC_t {                          // Point Pair Conversion
    std::vector<cv::Point2f> source;
    std::vector<cv::Point2f> reflection;
} PPC ;

typedef struct arguments_t {
    unsigned int refresh_count;                 // Number of iterations before rematching.
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
    unsigned int gft_min;

    void arguments();                           // Function to initialize defaults;
} arguments;

int Pipe ( int pipefd[2] );
pid_t Fork ( );
void change_good_features_to_track( int slider, void* gft );
void change_frame_number( int slider, void* fn );
void change_patch_size( int slider, void* ps );
void change_num_regions( int slider, void* nr );
void change_theta_dev( int slider, void* sd );

void slope_endpoints ( double angle, cv::Point2f* ol );

void update_regions ( const cv::Mat& frame, std::list<ARC_Pair>& pairs,
        const cv::Size& patch_size, std::vector<cv::Point2f>& GFT, int N );

void help( std::string program_name );

void get_image_list(std::string filename, std::vector<std::string>* il);

bool get_arguments(int argc, char** argv, arguments* a);

bool track ( const cv::Mat& gray, cv::Mat& prev_gray, std::list<ARC_Pair>& pairs );
void pairs_to_points ( const cv::Mat& gray, const std::list<ARC_Pair>& pairs, 
        std::vector<cv::Point2f>* src, std::vector<cv::Point2f>* ref,
        bool fbt );
void flow_gft ( const cv::Mat& gray, const cv::Mat& prev_gray, const std::vector<cv::Point2f>& pts, 
        std::vector<cv::Point2f>& npts );

#endif /* MAIN_H */
