/* waterline2.cpp
 * Martin Miller
 * 02/27/2014
 * ARC
 * Code to identify waterline in image
 * Usage: waterline <list of images>
 * where the list of images is a file with paths to each image
 */

#include	<stdlib.h>
#include <cv.h>
#include <highgui.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>


void find_water ( cv::Mat& src, cv::Mat& dst );
void getImageList( std::string filename,  std::vector<std::string>* il );
void maximum_rgb ( cv::Mat& src, cv::Mat& dst );
cv::Mat maskImage ( cv::Mat image, std::vector<cv::Point>& snake, cv::Scalar c );

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  find_water
 *  Description:  Quick and dirty approach to finding water in a the src image.
 *  Returns a mask of the water in dst. If no water found, dst=cv::Mat()
 *  =====================================================================================
 */
    void
find_water ( cv::Mat& src, cv::Mat& dst )
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

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  getImageList
 *  Description:  Reads in list of image filenames.
 * =====================================================================================
 */
void getImageList( std::string filename,  std::vector<std::string>* il )
{

    std::string    ifs_file_name = filename;         /* input  file name */
    std::ifstream  ifs;                              /* create ifstream object */

    ifs.open (  ifs_file_name.c_str( ) );         /* open ifstream */
    if ( !ifs ) {
        std::cerr << "\nERROR : failed to open input  file " << ifs_file_name << std::endl;
        exit ( EXIT_FAILURE );
    }
    std::string line;
    while(  getline( ifs,  line,  '\n') )
    {
        il->push_back( line );
    }
    ifs.close ( );                                 /* close ifstream */
}

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  maximum_rgb
 *  Description:  Sets maximum channel at each pixel to 255, zeros the others. 
 * =====================================================================================
 */
    void
maximum_rgb ( cv::Mat& src, cv::Mat& dst )
{
    cv::Mat tmp = src.clone();
    for( int i=0; i<src.rows; ++i )
    {
        for( int j=0; j<src.cols; ++j )
        {
            cv::Vec3b pixel;
            pixel=src.at<cv::Vec3b>(i,j);
            if( pixel[0]>pixel[1] && pixel[0]>pixel[2] )
            {
                pixel[0]=255;
                pixel[1]=pixel[2]=0;
            }
            else if( pixel[1]>pixel[2] )
            {
                pixel[1]=255;
                pixel[0]=pixel[2]=0;
            }
            else
            {
                pixel[2]=255;
                pixel[0]=pixel[1]=0;
            }
            tmp.at<cv::Vec3b>(i,j)=pixel;
        }
    }
    dst=tmp;
    return;
}		/* -----  end of function maximum_rgb  ----- */

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

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  main
 *  Description:  
 * =====================================================================================
 */
    int
main ( int argc, char *argv[] )
{
    std::vector<std::string> images;
    std::string listname;
    cv::VideoWriter vidout;
    cv::Mat firstFrame;

    if( argc!=2 )
    {
        std::cout << "Usage: " << argv[0] << " <list of images>" << std::endl;
        exit( EXIT_FAILURE );
    }
    listname = argv[1];
    getImageList( listname, &images );

    firstFrame = cv::imread( images[0], CV_LOAD_IMAGE_UNCHANGED );

    vidout.open( std::string("out.avi"), CV_FOURCC('F','M','P','4'), 20.0, firstFrame.size(), true);
    if( !vidout.isOpened() )
    {
        std::cerr << "Could not open video file out.avi" << std::endl;
        exit( EXIT_FAILURE );
    }

    for( std::vector<std::string>::iterator img_name=images.begin();
            img_name!=images.end(); ++img_name  )
    {
        cv::Mat imgColor = cv::imread( *img_name, CV_LOAD_IMAGE_UNCHANGED );
        cv::Mat img= cv::imread( *img_name, CV_LOAD_IMAGE_UNCHANGED );
        cv::Mat img2( imgColor.size(), CV_8UC3 );
        cv::Mat water_mask;//=cv::Mat::zeros( imgColor.size(), CV_8UC1 );
        find_water( img, water_mask );
        if( water_mask.size()==cv::Size(0,0) ) continue;


        cv::Mat edges;
        cv::Canny(water_mask, edges, 100, 200, 3);
        cv::dilate(edges, edges, cv::Mat(), cv::Point(-1,-1),64);

        cv::cvtColor( edges, img2, CV_GRAY2RGB );
        cv::addWeighted( img2, 0.3, imgColor, 0.7, 0, img2 );

        cv::imshow("Vid", img2 );
        //cv::imshow("VidColor", imgColor );
        vidout << img2;
        cv::waitKey(1);
    }
    return EXIT_SUCCESS;
}				/* ----------  end of function main  ---------- */
