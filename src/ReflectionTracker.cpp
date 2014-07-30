//Track the reflections of templates across water
//Author: Simon Peter 
//speter3@illinois.edu

//To modify the the vertical strip around the template used as a mask, change 'searchMargin' in findBestMatchLocation()
//To modify the number of features goodFeaturesToTrack should find, change 'numOfFeatures' in getReflections()

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
cv::Rect findBestMatchLocation( cv::Mat image, cv::Rect source_rect, 
        double* nsigma, cv::Mat mask )
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


    cv::Mat water_mask, edges;
    cv::Mat search_image;
    search_image = image_copy;

    //matchTemplate( masked_scene, sourceTemplate, result, match_method );
    matchTemplate( image_gray, sourceTemplate, result, match_method );

    double minVal, maxVal; 
    cv::Point minLoc, maxLoc, matchLoc;
    /*
    //find_water(image,water_mask);
    if( water_mask.size()!=cv::Size(0,0) )
        get_shorline_margin(water_mask,edges,64);
        */
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
int getReflections( cv::Mat frame, cv::Size patchSize, 
        std::list<ARC_Pair>* outlist, std::vector<cv::Point2f>& gft )
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

    //vector<Rect> originalMatches;
    cv::Mat water_mask, edges;


    // Goes through any ARC_Pairs already in the outvector and creates a mask to
    // prevent rematching those regions

    cv::Mat water_copy = frame.clone();
    /*
    //find_water(water_copy,water_mask);
    if( water_mask.size()!=cv::Size(0,0) )
        get_shorline_margin(water_mask,edges,64);
        */

    cv::Rect frame_rect( cv::Point(0, 0), frame.size() );
    for( std::vector<cv::Point2f>::iterator it=gft.begin();
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
        //Mat mask;
        //get_masked_frame( a, slope, &sourceCopy2, &mask );
		//b = findBestMatchLocation( slope, sourceCopy2, a, &nsigma, mask );
        // Create ARC_Pair
        bool err;
        ARC_Pair pair( *it, b, nsigma, sourceCopy2, &err );
        // Add to list
        if( !err ) outlist->push_back( pair );
    }
	
	return outlist->size();
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

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  get_shorline_mask
 *  Description:  finds the shorline edge from a water mask. Returns the edges with some
 *  margin that increases with iter.
 * =====================================================================================
 */
    void
get_shorline_margin ( cv::Mat src, cv::Mat& dst, int iter )
{
    cv::Rect fr(0,0,640,480);
    cv::Canny(src, dst, 100, 200, 3);
    cv::line(dst, fr.tl(), fr.tl() + cv::Point(0,fr.height), 0, 10 );
    cv::line(dst, fr.tl() + cv::Point(0,fr.height), fr.br(), 0, 10 );
    cv::dilate(dst, dst, cv::Mat(), cv::Point(-1,-1), iter );
    return;
}		/* -----  end of function get_shorline_mask  ----- */
/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  find_water
 *  Description:  Quick and dirty approach to finding water in a the src image.
 *  Returns a mask of the water in dst. If no water found, dst=cv::Mat()
 *  =====================================================================================
 */
    void
find_water ( cv::Mat src, cv::Mat& dst )
{
    cv::Point bottom_left;
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    std::vector<cv::Point> water;

    bottom_left = cv::Point( 0, src.size().height );

    // Find water areas
    cv::dilate( src, src, cv::Mat(), cv::Point(-1,-1), 2 );
    cv::erode( src, src, cv::Mat(), cv::Point(-1,-1), 4 );
    cv::cvtColor( src, src, CV_RGB2GRAY );
    
    cv::adaptiveThreshold( src, src, 255,
            CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY, 3, 0 );
    cv::blur( src, src, cv::Size(100, 100) );
    cv::medianBlur( src, src, 5 );
    cv::threshold( src, src, 50, 255, CV_THRESH_BINARY_INV );
    cv::dilate( src, src, cv::Mat(), cv::Point(-1,-1), 1 );
    cv::erode( src, src, cv::Mat(), cv::Point(-1,-1), 1 );

    // Select the contour that is in the bottom left
    findContours( src, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE );
    for( std::vector<std::vector<cv::Point> >::iterator c=contours.begin();
            c!=contours.end(); ++c )
    {
        if( cv::pointPolygonTest( *c, bottom_left, true )>=-10 )
        {
            water=*c;
            break;
        }
    }

    if( water.size()==0 ) 
    {
        std::cerr << "No water." <<std::endl;
        dst = cv::Mat();
    }
    else
    {
        dst = maskImage( src, water, cv::Scalar(255,255,255) );
    }
    return;
}		/* -----  end of function find_water  ----- */

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

    void
getShorelinePairs( cv::Mat frame, cv::Size patchSize, int numOfFeatures, double eig,
        std::list<ARC_Pair> &outlist )
{
    cv::Mat sourceCopy;
    cv::Mat water_mask;
    std::vector<cv::Point> points; 
    cv::Rect frame_rect;
    cv::Mat edges;
    bool err;
    cv::Mat mask = cv::Mat::ones(frame.size(),CV_8UC1)*255;

    cvtColor( frame, sourceCopy, CV_RGB2GRAY, 1 );
    frame_rect = cv::Rect( cv::Point(0, 0), frame.size() );

    //find_water(frame,water_mask);
    if( water_mask.size()!=cv::Size(0,0) )
        get_shorline_margin(water_mask,edges,32);
    goodFeaturesToTrack( sourceCopy, points, numOfFeatures, eig,
            5, edges, 3, 0, 0.04);
    for( std::vector<cv::Point>::iterator it=points.begin();
            it!=points.end(); ++it )
    {
        cv::Rect b;
        b = cv::Rect( *it-( .5*cv::Point( patchSize ) ), patchSize ) & frame_rect;
        ARC_Pair pair( *it, b, 1, frame, &err );
        if( !err ) outlist.push_back( pair );
    }
    return;
}
