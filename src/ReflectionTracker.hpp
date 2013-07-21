//Author: Simon Peter speter3@illinois.edu
#ifndef REFLECTION_TRACKER_H
#define REFLECTION_TRACKER_H
#include </usr/include/opencv2/opencv.hpp>
#include </usr/include/opencv/highgui.h>
#include <opencv2/highgui/highgui_c.h>
#include "ARC_Pair.hpp"

void createTemplatesFromVector(Mat image, Size patchSize, vector<Point> *points, list<ARC_Pair> *outlist);
Point findBestMatchLocation(double slope, Mat image,  Rect source_rect, double* nsigma );
void findReflections(Mat image, Size patchSize, double slope, list<ARC_Pair> *outlist);
void runSymmetryTest(Mat frame, int patchSize, double slope, vector<Point> *points, vector<Rect> *reflections, vector<Rect> *originalMatches, vector<ARC_Pair> *outvector);
void identifyRealObjects(vector<ARC_Pair> *outvector);

//The following functions are meant to be called on by a user, the previous ones are only used by getReflections()

//GIVEN AN IMAGE AND A PATCHSIZE, AND SLOPE INFORMATION, PUTS A SEQUENCE OF REAL OBJECTS AND THEIR REFLECTED REGIONS IN outvector AS ARC_Pair's
int getReflections(Mat frame, Size patchSize, int numOfFeatures, double slope, list<ARC_Pair> &outlist);
//DISPLAYS THE RESULTS OF getReflections()
void displayReflectionMatches(Mat image, Size patchSize, double slope, list<ARC_Pair> *outlist);
//GIVEN slope INFORMATION,A source MAT AND A tmplte RECT, IT RETURNS A RECT OF THE REFLECTION
Rect findOneReflection(double slope,Mat source, Rect tmplte);
//GIVEN A source MAT, patchSize, slope INFORMATION, AND A BOOLEAN FLAG, IT TRIES TO RETURN A NEW GOOD FEATURE AND IT'S REFLECTION
ARC_Pair getOneReflectionPair(Mat image, int patchSize, double slope, bool *regionFound);
int getReflectionsPYR(Mat &image, int outerPatchSize, int innerPatchSize, double slope, vector<ARC_Pair> &outvector);
#endif /*REFLECTION_TRACKER_H*/
