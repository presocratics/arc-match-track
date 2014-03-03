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
    cv::Point bottom_left;

    if( argc!=2 )
    {
        std::cout << "Usage: " << argv[0] << " <list of images>" << std::endl;
        exit( EXIT_FAILURE );
    }
    listname = argv[1];
    getImageList( listname, &images );

    firstFrame = cv::imread( images[0], CV_LOAD_IMAGE_UNCHANGED );
    bottom_left = cv::Point( 0, firstFrame.size().height );

    vidout.open( std::string("out.avi"), CV_FOURCC('F','M','P','4'), 20.0, firstFrame.size(), true);
    if( !vidout.isOpened() )
    {
        std::cerr << "Could not open video file out.avi" << std::endl;
        exit( EXIT_FAILURE );
    }

    for( std::vector<std::string>::iterator img_name=images.begin();
            img_name!=images.end(); ++img_name  )
    {
        std::vector<std::vector<cv::Point> > contours;
        std::vector<cv::Vec4i> hierarchy;
        // Initialize some Mats
        cv::Mat water_mask;
        cv::Mat imgColor = cv::imread( *img_name, CV_LOAD_IMAGE_UNCHANGED );
        //cv::Mat img = cv::imread( *img_name, CV_LOAD_IMAGE_GRAYSCALE );
        cv::Mat img = cv::imread( *img_name, CV_LOAD_IMAGE_UNCHANGED );
        cv::Mat img2( imgColor.size(), CV_8UC3 );

        cv::dilate( img, img, cv::Mat(), cv::Point(-1,-1), 2 );
        cv::erode( img, img, cv::Mat(), cv::Point(-1,-1), 4 );
        
        cv::cvtColor( img, img, CV_RGB2GRAY );


        
        // Find water areas
        //cv::medianBlur( img, img, 5 );
        cv::adaptiveThreshold( img, img, 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY, 3, 0 );
        cv::blur( img, img, cv::Size(100, 100) );
        cv::medianBlur( img, img, 5 );
        cv::threshold( img, img, 50, 255, CV_THRESH_BINARY_INV );
        cv::dilate( img, img, cv::Mat(), cv::Point(-1,-1), 1 );
        cv::erode( img, img, cv::Mat(), cv::Point(-1,-1), 1 );

        // Select the contour that is in the bottom left
        std::vector<cv::Point> water;
        findContours( img, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE );
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
            continue;   
        }
        water_mask = maskImage( img, water, cv::Scalar(255,255,255) );

        cv::cvtColor( water_mask, img2, CV_GRAY2RGB );
        cv::addWeighted( img2, 0.3, imgColor, 0.7, 0, img2 );
        //maximum_rgb( img2, img2 );

        cv::imshow("Vid", img2 );
        //cv::imshow("VidColor", imgColor );
        vidout << img2;
        cv::waitKey(3);
    }
    return EXIT_SUCCESS;
}				/* ----------  end of function main  ---------- */
