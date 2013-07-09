//Author: Simon Peter speter3@illinois.edu
#ifndef REFLECTION_TRACKER_H
#define REFLECTION_TRACKER_H
#include "ARC_Pair.hpp"
#include "config.hpp"
//#include </usr/include/opencv2/opencv.hpp>
#include "ARC_Pair.hpp"

void createTemplatesFromVector(Mat image, int patchSize, vector<Point> *points, vector<Mat> *templates, vector<ARC_Pair> *outvector);
Point findBestMatchLocation(Mat image, Mat sourceTemplate, int TLCornerTemplate);
void findReflections(Mat image, int patchSize, vector<Point> *points, vector<Mat> *templates, vector<Rect> *reflections, vector<ARC_Pair> *outvector);
void runSymmetryTest(Mat frame, int patchSize, vector<Point> *points, vector<Rect> *reflections, vector<Rect> *originalMatches, vector<ARC_Pair> *outvector);
void identifyRealObjects(vector<ARC_Pair> *outvector);

//The following functions are meant to be called on by a user, the previous ones are only used by getReflections()

//GIVEN AN IMAGE AND A PATCHSIZE, PUTS A SEQUENCE OF REAL OBJECTS AND THEIR REFLECTED REGIONS IN outvector AS ARC_Pair's
int getReflections(Mat frame, int patchSize, int numOfFeatures, vector<ARC_Pair> &outvector);
//DISPLAYS THE RESULTS OF getReflections()
void displayReflectionMatches(Mat image, int patchSize, vector<ARC_Pair> *outvector);
//GIVEN A source MAT, A tmplte MAT, AND THE leftBound OF THE tmplte, IT RETURNS A RECT OF THE REFLECTION
Rect findOneReflection(Mat source, Mat tmplte, int leftBound);
//GIVEN A source MAT AND A tmplte RECT, IT RETURNS A RECT OF THE REFLECTION
Rect findOneReflection(Mat source, Rect tmplte);
//GIVEN A source MAT, patchSize, AND A BOOLEAN FLAG, IT TRIES TO RETURN A NEW GOOD FEATURE AND IT'S REFLECTION
ARC_Pair getOneReflectionPair(Mat image, int patchSize, bool *regionFound);
#endif /*REFLECTION_TRACKER_H*/
