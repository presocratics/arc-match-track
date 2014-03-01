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
        // Initialize some Mats
        cv::Mat imgColor = cv::imread( *img_name, CV_LOAD_IMAGE_UNCHANGED );
        cv::Mat img = cv::imread( *img_name, CV_LOAD_IMAGE_GRAYSCALE );
        cv::Mat img2( imgColor.size(), CV_8UC3 );

        // Blur some stuff
        cv::medianBlur( img, img, 5 );
        cv::adaptiveThreshold( img, img, 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY, 3, 0 );
        cv::blur( img, img, cv::Size(100, 100) );
        cv::medianBlur( img, img, 5 );
        cv::threshold( img, img, 50, 255, CV_THRESH_BINARY_INV );

        cv::cvtColor( img, img2, CV_GRAY2RGB );
        cv::addWeighted( img2, 0.3, imgColor, 0.7, 0, img2 );


        cv::imshow("Vid", img2 );
        vidout << img2;
        cv::waitKey(3);
    }
    return EXIT_SUCCESS;
}				/* ----------  end of function main  ---------- */
