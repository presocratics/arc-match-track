//Track the reflections of templates across water
//Author: Simon Peter 
//speter3@illinois.edu

//To modify the the vertical strip around the template used as a mask, change 
//'searchMargin' in findBestMatchLocation() To modify the number of features 
//goodFeaturesToTrack should find, change 'numOfFeatures' in getReflections()

#include <cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <string>
#include "config.hpp"
#include "ARC_Pair.hpp"
#include "ReflectionTracker.hpp"

bool displayRegions;
bool displayWindows;


//RETURNS THE TOP-LEFT CORNER OF THE REFLECTION OF sourceTemplate ON image
cv::Rect findBestMatchLocation( const cv::Mat& image, const cv::Rect& source_rect, 
        double* nsigma, const cv::Mat& mask )
{
    cv::Mat image_gray;
    cvtColor( image, image_gray, CV_RGB2GRAY, 1 );
    cv::Mat image_copy = image_gray.clone();
    
    // Create template.
    cv::Mat image_template_copy = image_gray.clone();
    cv::Mat sourceTemplate = image_template_copy( source_rect );
    flip( sourceTemplate, sourceTemplate, 0 );

    // Creates results matrix where the top left corner of the 
    // template is slid across each pixel of the source
    int result_cols = image.cols-sourceTemplate.cols+1;
    int result_rows = image.rows-sourceTemplate.rows+1;
    cv::Mat result;
    result.create( result_cols,result_rows, CV_32FC1 );

    // Mask image to match only in selected ROI.
    if( !mask.empty() )
    {
        cv::Mat tmp;
		image_copy.copyTo( tmp, mask );
        image_copy = tmp;
    }

    //0:CV_TM_SQDIFF
    //1:CV_TM_SQDIFF_NORMED
    //2:CV_TM_CORR
    //3:CV_TM_CCOR_NORMED
    //4:CV_TM_CCOEFF
    //5:CV_TM_CCOEFF_NORMED <----Most succesful at finding reflections
    
    int match_method = CV_TM_CCOEFF_NORMED; // 4 seemed good for stddev thresholding.

    //matchTemplate( masked_scene, sourceTemplate, result, match_method );
    matchTemplate( image_gray, sourceTemplate, result, match_method );

    double minVal, maxVal; 
    cv::Point minLoc, maxLoc, matchLoc;
    minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat() );
     
    if( match_method == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED )
    {
        matchLoc = minLoc;	
    }
    else 
    {
        matchLoc = maxLoc;
    }
    cv::Scalar mean, stddev;
    meanStdDev( result, mean, stddev, cv::Mat() );
    *nsigma = ( maxVal-mean[0] )/ stddev[0];
    // matchLoc is the location of the top left corner of the reflection
    // that matchTemplate found

    return cv::Rect( matchLoc, source_rect.size() );
}

// GIVEN AN IMAGE, SLOPE INFORMATION, AND A PATCHSIZE, PUTS A SEQUENCE OF
// REAL OBJECTS AND THEIR REFLECTED REGIONS IN outvector AS ARC_Pair's
int getReflections( const cv::Mat& frame, const cv::Size& patchSize, 
        std::list<ARC_Pair>& outlist, const std::vector<cv::Point2f>& gft )
{
    cv::Mat sourceCopy;
	if( !frame.data ) 
    {
        std::cout << "Image couldn't be loaded\n";
        exit( EXIT_FAILURE );
    }

	frame.copyTo( sourceCopy );
	cvtColor( sourceCopy, sourceCopy, CV_RGB2GRAY, 1 );
    cv::equalizeHist( sourceCopy, sourceCopy );

    std::vector<cv::Point> points; 

    cv::Rect frame_rect( cv::Point(0, 0), frame.size() );
    for( std::vector<cv::Point2f>::const_iterator it=gft.begin();
            it!=gft.end(); ++it )
    {
        cv::Mat sourceCopy2 = frame.clone();
        cv::Rect a, b;
        double nsigma;
        // Get Rect A
        a = cv::Rect( *it-( .5*cv::Point2f( patchSize ) ), patchSize ) & frame_rect;
        if( a.width==0 || a.height==0 ) continue;
        // Get Rect B and nsigma
		b = findBestMatchLocation( sourceCopy2, a, &nsigma, cv::Mat() );
        // Create ARC_Pair
        bool err;
        ARC_Pair pair( *it, b, nsigma, sourceCopy2, &err );
        // Add to list
        if( !err ) outlist.push_back( pair );
    }
	
	return outlist.size();
}

// === FUNCTION ======================================================================
// Name: get_masked_frame
// Description: Masks the frame based on slope and roi. Mask returned by pointer.
// =====================================================================================
cv::Mat get_masked_frame ( cv::Rect roi, double slope, cv::Mat* frame, cv::Mat* mask )
{
    cv::Point corners[1][4];
    //Set the frame
    *mask=cv::Mat::zeros( frame->size(), CV_8UC1 );
    cv::Mat masked_frame;
    if( slope==0 )
    {
        // TODO: Could use direction handling here.
        corners[0][0] = roi.br();
        corners[0][1] = cv::Point( frame->cols, roi.y+roi.height );
        corners[0][2] = corners[0][1]-cv::Point( 0, roi.height );
        corners[0][3] = corners[0][0]-cv::Point( 0, roi.height );
    }
    else if( isinf( slope ) )
    {
        {
            corners[0][0] = cv::Point( roi.x, frame->rows );
            corners[0][1] = cv::Point( roi.x, roi.y+roi.height);
        }
        {
            corners[0][0] = roi.tl();
            corners[0][1] = cv::Point( roi.x, 0 );
        }
        corners[0][2] = corners[0][1]+cv::Point( roi.width, 0);
        corners[0][3] = corners[0][0]+cv::Point( roi.width, 0 );
    }
    else
    {
        corners[0][0].x = ( int ) ( (frame->rows + slope*roi.x-roi.y)/slope );
        corners[0][0].y = frame->rows;
        corners[0][1] = cv::Point( ( int )( (-roi.y + slope * roi.x ) / slope ), 0 );
        corners[0][2] = corners[0][1] + cv::Point(roi.width, 0);
        corners[0][3] = corners[0][0] + cv::Point(roi.width, 0);
    }

    // This is weird, but follows OpenCV docs.
    const cv::Point* ppt[1] = { corners[0] };
    const int npt[] = { 4 };

    fillPoly( *mask, ppt, npt, 1, 255 );
    frame->copyTo(masked_frame, *mask);
    return masked_frame;
}	// ----- end of function get_masked_frame ----- 

    cv::Mat
maskImage ( cv::Mat image, std::vector<cv::Point>& snake, cv::Scalar c )
{
    cv::Mat mask, masked_contour ;
    std::vector<std::vector<cv::Point> > contours;

    contours.push_back( snake );
    // mask the image with the contour.
    mask= cv::Mat::zeros(image.size(), CV_8UC1 );
    cv::drawContours(mask, contours, -1, cv::Scalar(255,255,255), CV_FILLED );
    masked_contour = cv::Mat( image.size(), CV_8UC3 );
    if( c!=cv::Scalar(-1,-1,-1,-1) )
    {
        masked_contour.setTo(c);
    }
    image.copyTo(masked_contour, mask);
    return masked_contour;
}		/* -----  end of function maskImage  ----- */
