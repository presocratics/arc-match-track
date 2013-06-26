#ifndef REFLECTION_TRACKER_H
#define REFLECTION_TRACKER_H
#include </usr/include/opencv2/opencv.hpp>
#include </usr/include/opencv/highgui.h>
#include <opencv2/highgui/highgui_c.h>
void getReflections(Mat frame, int patchSize, vector<ARC_Pair> &outvector);
#endif /*REFLECTION_TRACKER_H*/
