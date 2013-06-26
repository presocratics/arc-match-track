/*
 * =====================================================================================
 *
 *       Filename:  main.cpp
 *
 *    Description:  Test main for Reflection Tracker
 *
 *        Version:  1.0
 *        Created:  06/26/2013 11:27:16 AM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Simon Peter
 *   Organization:  
 *
 * =====================================================================================
 */


#include <iostream>
#include <cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <string>
#include "ARC_Pair.hpp"
#include "config.hpp"
#include "ReflectionTracker.hpp"

using namespace std;
using namespace cv;

int main(int argc,char **argv){


    vector<ARC_Pair> outvector;
    Mat image = imread(argv[1],CV_LOAD_IMAGE_UNCHANGED);
    Mat source = image.clone(); 
    Mat reflection=image.clone();
    getReflections(image,50,outvector);
    Rect tmp = outvector[1].roi.source;
    cout<<"Outvector size: "<<outvector.size()<<endl;
    cout<<"Source roi at "<<tmp.tl()<<" of size "<<(Point) tmp.size()<<endl;
    source = source(outvector[1].roi.source);
    reflection = reflection(outvector[1].roi.reflection);
    cout<<"Outvector size: "<<outvector.size()<<endl;
    namedWindow("outvector[1]",CV_WINDOW_AUTOSIZE);
    imshow("outvector[1]",source);
    namedWindow("outvector reflection", CV_WINDOW_AUTOSIZE);
    imshow("outvector reflection",reflection);
    waitKey(0);
    return 0;
}

