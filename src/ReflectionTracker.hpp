//Author: Simon Peter speter3@illinois.edu
#ifndef REFLECTION_TRACKER_H
#define REFLECTION_TRACKER_H
#include </usr/include/opencv2/opencv.hpp>
#include </usr/include/opencv/highgui.h>
#include <opencv2/highgui/highgui_c.h>
#include "ARC_Pair.hpp"

void createTemplatesFromVector(Mat image, int patchSize, vector<Point> *points, vector<Mat> *templates, vector<ARC_Pair> *outvector);
Point findBestMatchLocation(Mat image, Mat sourceTemplate, int TLCornerTemplate);
void findReflections(Mat image, int patchSize, vector<Point> *points, vector<Mat> *templates, vector<Rect> *reflections, vector<ARC_Pair> *outvector);
void runSymmetryTest(Mat frame, int patchSize, vector<Point> *points, vector<Rect> *reflections, vector<Rect> *originalMatches, vector<ARC_Pair> *outvector);
void identifyRealObjects(vector<ARC_Pair> *outvector);
int getReflections(Mat frame, int patchSize, int numOfFeatures, vector<ARC_Pair> &outvector);
void displayReflectionMatches(Mat image, int patchSize, vector<ARC_Pair> *outvector);
Rect findOneReflection(Mat source, Mat tmplte, int leftBound);
Rect findOneReflection(Mat source, Rect tmplte);
ARC_Pair getOneReflectionPair(Mat image, int patchSize, bool *regionFound);
void getDifferentReflectionPairs(Mat image, int patchSize, int desiredSize,vector<ARC_Pair> *outvector);
#endif /*REFLECTION_TRACKER_H*/
