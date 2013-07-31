/*
 * =====================================================================================
 *
 *       Filename:  ARC_Segment.cpp
 *
 *    Description:  SURF and KLT matching from the basis of the segment unit. A
 *    segment is a container for storing data about a reflection and its match.
 *
 *        Version:  1.0
 *        Created:  06/04/2013 08:59:15 AM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Martin Miller (), miller7@illinois.edu
 *   Organization:  University of Illinois at Urbana-Champaign
 *
 * =====================================================================================
 */
#include "main.hpp"
#include "config.hpp"
#include "ARC_IMU.hpp"
//#define DEBUG_IMU
using namespace std;


/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  get_imu_list
 *  Description:  
 * =====================================================================================
 */
    void
get_imu_list ( string filename, vector<Point3f>* il )
{

    string    ifs_file_name = filename;                 /* input  file name */
    ifstream  ifs;                                /* create ifstream object */

    ifs.open ( ifs_file_name.c_str() );           /* open ifstream */
    if (!ifs) {
        cerr << "\nERROR : failed to open input  file " << ifs_file_name << endl;
        exit (EXIT_FAILURE);
    }
    string line;
    while( getline( ifs, line, '\n' ) )
    {
        double x, y, z;
        string fn;
        stringstream l(line);
        l >> fn >> x >> y >> z;
        il->push_back( Point3f( x, y, z ) );
    }
    ifs.close ();                                 /* close ifstream */
    return ;
}		/* -----  end of function get_imu_list  ----- */

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  get_image_list
 *  Description:  Get a list of image files for the video stream.
 * =====================================================================================
 */
void get_image_list(string filename, vector<string>* il)
{
 
    string    ifs_file_name = filename;         /* input  file name */
    ifstream  ifs;                              /* create ifstream object */

    ifs.open ( ifs_file_name.c_str() );         /* open ifstream */
    if (!ifs) {
        cerr << "\nERROR : failed to open input  file " << ifs_file_name << endl;
        exit (EXIT_FAILURE);
    }
    string line;
    while( getline(ifs, line, '\n') )
    {
        il->push_back(line);
    }
    ifs.close ();                                 /* close ifstream */
}
#ifndef  DEBUG_IMU
void change_frame_number( int slider, void* fn )
{
    unsigned int* fn_typed = (unsigned int *) fn;
    *fn_typed = ( unsigned int )slider;
}

void change_patch_size( int slider, void* ps )
{
    Size* ps_typed = (Size *) ps;
    *ps_typed = Size( slider, slider );
}

void change_num_regions( int slider, void* nr )
{
    int* nr_typed = (int *) nr;
    *nr_typed = slider;
}

void change_theta_dev( int slider, void* sd )
{
    double* sd_typed = (double *) sd;
    *sd_typed = slider/100.0;
}


// ===  FUNCTION  ======================================================================
//         Name:  slope_endpoints
//  Description:  
// =====================================================================================
void slope_endpoints ( double slope, Point2f* ol )
{
    double x = 480/slope+320;
    Point2f t(320, 0);
    Point2f b(x, 480);
    ol[0] = t;
    ol[1] = b;
    return ;
}		// -----  end of function slope_endpoints  ----- 

// ===  FUNCTION  ======================================================================
//         Name:  update_regions
//  Description:  Removes low quality regions and add new regions.
// =====================================================================================
void update_regions ( Mat& frame, list<ARC_Pair>* pairs,
        unsigned int nregions, Size patch_size, double slope, double theta )
{
    //pairs->clear();
    //cout << "Num regions: " << nregions << endl;
    //cout << "Patch size: " << Point(patch_size) << endl;
    // Get new regions.
    //if( pairs->size()<nregions )
    if( 1 )
    {
        getReflections( frame, patch_size, nregions, slope, *pairs );
        //getReflectionsPYR( frame, patch_size, Size( 10, 10 ), slope, theta, *pairs );
    }
        
    return ;
}		// -----  end of function update_regions  ----- 

// ===  FUNCTION  ======================================================================
//         Name:  draw_match_by_hand
//  Description:  
// =====================================================================================
void draw_match_by_hand( Mat* out_img, Mat* scene, 
        Mat* object, Rect sroi, Rect rroi, 
        vector<Point2f>& source_points, 
        vector<Point2f>& reflection_points)
{
    if( source_points.size()!=reflection_points.size() )
    {
        cerr << "source_points.size()!=reflection_points.size()" << endl; 
        exit( EXIT_FAILURE );
    }

    // draw lines
    // TODO: Random colors. Should be stored at region level.
    for( size_t i=0; i<source_points.size() ; ++i )
    {
        circle( *out_img, source_points[i], 3, Scalar(0, 255, 0) );
        circle( *out_img, reflection_points[i], 3, Scalar(0, 255, 0) );
        line( *out_img, source_points[i], reflection_points[i], Scalar(0, 255, 0), 1, CV_AA );
    }
    rectangle( *out_img, sroi, Scalar( 50, 100, 150 ), 1 );
    rectangle( *out_img, rroi, Scalar( 50, 100, 150 ), 1 );
    return ;
}		// -----  end of function draw_match_by_hand  ----- 


// ===  FUNCTION  ======================================================================
//         Name:  help()
//  Description:  Display options.
// =====================================================================================
void help( string program_name )
{
    cout 
         << "Usage: " << program_name << spc << "<list of image files> <list of regions> [options]"
         << endl
         << "OPTIONS" << endl
         << ARC_ARG_FEATURES_BEFORE_TRACK << tab 
         << "Run goodFeaturesToTrack in each frame before tracking." 
         << " Default: " << ARC_DEFAULT_FEATURES_BEFORE_TRACK << endl
         << ARG_SHOW_MATCHES << tab << "Show matches." << endl
         << ARG_SHOW_TRACKING << tab << "Show tracking (default)." << endl
         << ARG_VID_FILE << spc << "<filename>" << tab << "Set video output file." << endl
         << ARG_TXT_FILE << spc << "<filename>" << tab << "Set text output file." << endl
         << ARG_BLUR << tab << "Median blur scene for tracking." << endl
         << ARG_NO_BLUR << tab << "No median blur scene for tracking." << endl

         << ARC_ARG_THETA_DEV << spc << "[0-90]" << tab << "Set Max deviation of match slope from IMU slope." << spc
         << "Default: " << ARC_DEFAULT_THETA_DEV << endl

         << ARC_ARG_PATCH_SIZE << spc << "[0-150]" << tab << "Set region patch size." << spc
         << "Default: " << ARC_DEFAULT_PATCH_SIZE<<"x"<<ARC_DEFAULT_PATCH_SIZE << endl

         << ARC_ARG_NUM_REGIONS << spc << "[0-50]" << tab << "Set desired number of regions to track." << spc
         << "Actual number tracked may be less." << spc
         << "Default: " << ARC_DEFAULT_NUM_REGIONS << endl

         << ARG_REFRESH_COUNT << spc << "<N>" << tab << "Number of iterations before rematching. "
         << "Default: " << DEFAULT_REFRESH_COUNT << endl

         << ARG_VERBOSE << tab << "Verbose output." << endl
         << ARG_VERY_VERBOSE << tab << "Very verbose output." << endl
         << ARG_VERY_VERY_VERBOSE << tab << "Very very verbose output." << endl
         << ARG_DEBUG_MODE << tab << "Show debugging output." << endl
         ;

    return ;
}		// -----  end of function help()  ----- 

// ===  FUNCTION  ======================================================================
//         Name:  pairs_to_points
//  Description:  Writes centers of ARC_Pair ROIs to source and reflection point vectors.
// =====================================================================================
void pairs_to_points ( Mat gray, list<ARC_Pair>* pairs, 
        vector<Point2f>* src, vector<Point2f>* ref,
        bool fbt )
{
    for( list<ARC_Pair>::iterator it=pairs->begin();
            it!=pairs->end(); ++it )
    {
        Point2f s, r;
        s = it->roi.source;
        r = it->roi.reflection;
        if( fbt )
        {
            Mat masks, maskr;
            vector<Point2f> vs, vr;
            Point2f shift( 5, 5 );
            Size sz( 10, 10 );
            Rect rs, rr;

            rs = Rect( s-shift, sz );
            rr = Rect( r-shift, sz );

            masks = Mat::zeros( gray.size(), CV_8UC1 );
            maskr = Mat::zeros( gray.size(), CV_8UC1 );
            rectangle( masks, rs, 255, CV_FILLED );
            rectangle( maskr, rr, 255, CV_FILLED );

            goodFeaturesToTrack( gray, vs, 1, 0.01, 10, masks, 3, 0, 0.04);
            goodFeaturesToTrack( gray, vr, 1, 0.01, 10, maskr, 3, 0, 0.04);
            s = ( vs.size()>0 ) ? vs[0] : s;
            r = ( vr.size()>0 ) ? vr[0] : r;
        }
        src->push_back( s );
        ref->push_back( r );
    }
    return ;
}		// -----  end of function pairs_to_points  ----- 

// ===  FUNCTION  ======================================================================
//         Name:  track
//  Description:  Track matched points.
// =====================================================================================
bool track( Mat gray, Mat prev_gray, list<ARC_Pair>* pairs )
{
    TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 0.03);
    Size sub_pix_win_size(10,10);
    Size win_size(31,31);
    PPC points, new_points;

    if( prev_gray.empty() )
        gray.copyTo(prev_gray);
    pairs_to_points( gray, pairs, &points.source, &points.reflection, true );

    // Do the tracking.
    size_t i;
    if( points.source.size()!=0 )
    {
        vector<uchar> source_status, reflection_status;
        vector<float> source_error, reflection_error;

        calcOpticalFlowPyrLK( prev_gray, gray, points.source, new_points.source,
                source_status, source_error, win_size, 3, termcrit, 0, 0.001 );

        calcOpticalFlowPyrLK(prev_gray, gray, points.reflection,
                new_points.reflection, reflection_status, 
                reflection_error, win_size, 3, termcrit, 0, 0.001);
        
        // Set lost points to (-1, -1), so we know they are lost.
        list<ARC_Pair>::iterator it=pairs->begin();
        for( i=0; i<(new_points.source.size()); i++ )
        {
            Point sdel, rdel;
            //double smag, rmag;
            if( !source_status[i] || !reflection_status[i] ) 
            {
                it = pairs->erase( it );
                continue;
            }
            // Get difference between old and move.
            sdel = new_points.source[i] - points.source[i];
            rdel = new_points.reflection[i] - points.reflection[i];
            //cout << it->id << spc << sdel.y-rdel.y << endl;
            if( abs( sdel.y-rdel.y ) > 2 )
            {
                //cout << "Large SDEL-RDEL diff" << endl;
                it->nNoMatch=1;
                ++it;
                continue;
            }
            
            // Shift by difference.
            it->roi.source += sdel;
            it->roi.reflection += rdel;
            ++it;
        }
    }

    return ( new_points.source.size()>0 && new_points.reflection.size()>0 ) ? true : false;
}		// -----  end of method ARC_Pair::track  ----- 

// ===  FUNCTION  ======================================================================
//         Name:  mask_scene
//  Description:  Masks the frame to the given roi.
// =====================================================================================
Mat mask_scene ( Rect roi, Mat& frame )
{
    Mat masked_frame;
    Mat mask = Mat::zeros( frame.size(), CV_8UC1 );
    rectangle( mask, roi, 255, CV_FILLED );
    frame.copyTo(masked_frame, mask);

    return masked_frame;
}		// -----  end of function mask_scene  ----- 


/*
 * arguments constructor
 */
void arguments::arguments()
{
    features_before_track = ARC_DEFAULT_FEATURES_BEFORE_TRACK;
    blur = ARC_DEFAULT_BLUR;
    refresh_count = DEFAULT_REFRESH_COUNT;
    theta_dev = ARC_DEFAULT_THETA_DEV;
    num_regions = ARC_DEFAULT_NUM_REGIONS;
    patch_size = Size( ARC_DEFAULT_PATCH_SIZE, ARC_DEFAULT_PATCH_SIZE );
    debug = NO_DEBUG;
    verbosity = NOT_VERBOSE;
    show_match = NO_SHOW_MATCHES;
    show_track = SHOW_TRACKING;
    video_filename = DEFAULT_VID_FILENAME;
    text_filename = DEFAULT_TXT_FILENAME;
    return;
}


/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  get_regions
 *  Description:  Read in a list of regions and check for errors in format.
 *  File Format:  <x> <y> <width> <height> [direction] [slope]
 * =====================================================================================
 */
/*
bool get_regions(string filename, vector<ARC_Pair>* regions)
{
    bool status=true;
    string    ifs_file_name = filename;                 // input  file name
    ifstream  ifs;                                // create ifstream object 

    ifs.open ( ifs_file_name.c_str() );           // open ifstream 
    if (!ifs) {
        cerr << "\nERROR : failed to open input  file " << ifs_file_name << endl;
        exit (EXIT_FAILURE);
    }
    string line;
    while( getline(ifs, line, '\n') )
    {
        double slope=INFINITY;
        unsigned int direction=UP;
        Point2i loc(-1,-1);
        Size dim(-1,-1);

        stringstream l(line);

        l >> loc.x >> loc.y >> dim.width >> dim.height >> direction >> slope;
        if( loc.x>-1 && loc.y>-1 && dim.width>0 && dim.height>0 )
        {
            Rect roi(loc,dim);
            ARC_Pair region;
            if( direction==DOWN )
                region.roi.source = roi;
            else
                region.roi.reflection = roi;
            regions->push_back(region);
        }
        else
        {
            cerr << "Invalid region at " << loc << " of size " << (Point) dim << endl;
            status=false;
        }
    }
    ifs.close ();                                 // close ifstream 
    return status;
}
*/

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  get_arguments
 *  Description:  Parses command line arguments.
 * =====================================================================================
 */

bool get_arguments ( int argc, char** argv, arguments* a)
{
    if( argc==1 ) return false;
    for ( int i = 3; i < argc; i += 1 ) 
    {
        if( !strcmp(argv[i], ARC_ARG_FEATURES_BEFORE_TRACK ) ) a->features_before_track = true;
        if( !strcmp(argv[i], ARG_BLUR) ) a->blur = true;
        if( !strcmp(argv[i], ARG_DEBUG_MODE) ) a->debug=DEBUG;
        if( !strcmp(argv[i], ARG_VERBOSE) ) a->verbosity=VERBOSE;
        if( !strcmp(argv[i], ARG_VERY_VERBOSE) ) a->verbosity=VERY_VERBOSE;
        if( !strcmp(argv[i], ARG_VERY_VERY_VERBOSE) ) a->verbosity=VERY_VERY_VERBOSE;
        if( !strcmp(argv[i], ARG_VERY_VERY_VERBOSE) ) a->verbosity=VERY_VERY_VERBOSE;

        if( !strcmp(argv[i], ARG_SHOW_MATCHES) ) a->show_match=SHOW_MATCHES;

        if( !strcmp(argv[i], ARG_SHOW_TRACKING) ) a->show_track=SHOW_TRACKING;
        if( !strcmp(argv[i], ARC_ARG_THETA_DEV) ) 
        {
            a->theta_dev=atof(argv[++i]);
            continue;
        }
        if( !strcmp(argv[i], ARC_ARG_NUM_REGIONS) ) 
        {
            a->num_regions=atof(argv[++i]);
            continue;
        }
        if( !strcmp(argv[i], ARC_ARG_PATCH_SIZE) ) 
        {
            int dim = atoi( argv[++i] );
            a->patch_size=Size( dim, dim );
            continue;
        }
        if( !strcmp(argv[i], ARG_VID_FILE) ) 
        {
            a->video_filename=argv[++i];
            continue;
        }
        if( !strcmp(argv[i], ARG_TXT_FILE) ) 
        {
            a->text_filename=argv[++i];
            continue;
        }
        if( !strcmp(argv[i], ARG_REFRESH_COUNT) ) 
        {
            a->refresh_count=atoi(argv[++i]);
            continue;
        }
    }
    return true;
}		/* -----  end of function get_arguments  ----- */


int main(int argc, char** argv)
{
    vector<string> image_list;                  // Video frames for tracking.
    vector<Point3f> imu_list;                  // IMU data for slope.
    list<ARC_Pair> pairs;                   // Container for selected reflections and matches.

    // Parse Arguments
    arguments a;
    a.arguments();                              // TODO should init automatically.
    namedWindow( DEFAULT_WINDOW_NAME, CV_WINDOW_AUTOSIZE );

    if( !get_arguments(argc, argv, &a) )        // Parse command line args.
    {
        help( argv[0] );
        exit( EXIT_FAILURE );
    }

    if( a.debug==DEBUG )
    {
        string blur_status = ( a.blur ) ? "true" : "false" ;
        cout
            << "ARGUMENTS" << endl
            << "Refresh Count:" << tab << a.refresh_count << endl
            << "Debug:" << tab << a.debug << endl
            << "Verbosity:" << tab << a.verbosity << endl
            << "Show Match:" << tab<< a.show_match << endl
            << "Show Track:" << tab << a.show_track << endl
            << "Blur: " << tab << blur_status << endl
            << "Video Filename:" << tab << a.video_filename <<
            endl;
    }
    get_image_list( argv[1], &image_list );     // Reads in the image list.
    get_imu_list( argv[2], &imu_list );
    //get_regions( argv[2], &regions );           // Reads in the region list.

    // Create GUI objects
    unsigned int i = 0;                               // Image index
    int td = a.theta_dev * 100;
    createTrackbar( "theta_dev", DEFAULT_WINDOW_NAME, &td, 
            100, change_theta_dev, &a.theta_dev );
    createTrackbar( "num_regions", DEFAULT_WINDOW_NAME, &a.num_regions, 
            50, change_num_regions, &a.num_regions );
    createTrackbar( "patch_size", DEFAULT_WINDOW_NAME, &a.patch_size.width, 
            150, change_patch_size, &a.patch_size );
    createTrackbar( "frame_number", DEFAULT_WINDOW_NAME, (int*) &i, 
            image_list.size(), change_frame_number, &i );

    // Init text file
    //ARC_Write writer( a.text_filename );

    // Init Video
    Mat first_frame=imread( image_list[0], CV_LOAD_IMAGE_COLOR );
    VideoWriter vidout;
    vidout.open( a.video_filename, CV_FOURCC('F','M','P','4'), 
            20.0, first_frame.size(), true );
    if( !vidout.isOpened() )
    {
        cerr << "Could not open video file: " << a.video_filename << endl;
        exit( EXIT_FAILURE );
    }

    // Init ARC_IMU for rotation matrix.
    ARC_IMU imu;
    imu.set_A( A );
    //Begin image loop.
    Point2f mid_pt( 320, 240 );
    Mat cur_frame, gray, prev_gray;
    double alpha = .125;
    double theta = -1;
    while( i<image_list.size() )
    {
        setTrackbarPos( "frame_number", DEFAULT_WINDOW_NAME, (int) i );
        if( a.verbosity>=VERBOSE ) cout << "Frame: " << image_list[i] << endl;
        Matx33d rotation_matrix = imu.calc_rotation_matrix( imu_list[i] );
        if( theta==-1 )
            theta = imu.get_rotation_angle( rotation_matrix );
        else
            theta = (1-alpha)*theta+alpha*imu.get_rotation_angle( rotation_matrix );
        double slope = imu.theta_to_slope( theta );

        cur_frame=imread( image_list[i], CV_LOAD_IMAGE_COLOR );           // open image 
        if ( !cur_frame.data ) {
            cerr << "\nERROR : failed to open input file " << image_list[i] << endl;
            exit (EXIT_FAILURE);
        }
        cvtColor(cur_frame, gray, CV_BGR2GRAY);
        if( a.blur )
        {
            //medianBlur( gray, gray, 7 );        // TODO: Should be parameter
            //medianBlur( gray, gray, 5 );        // TODO: Should be parameter
            medianBlur( gray, gray, 3 );        // TODO: Should be parameter
        }
        Mat drawn_matches;
        cur_frame.copyTo(drawn_matches);

        // Filter pairs.
        // Update regions.
        if( i%5==0 && pairs.size()<(unsigned int)a.num_regions )
            update_regions( cur_frame, &pairs, 12, a.patch_size, slope, theta );
        pairs.remove_if( below_threshold( 3 ) );
        pairs.remove_if( outside_theta( theta, a.theta_dev ) );
        pairs.remove_if( overlap( a.patch_size ) );
        // track.
        track( gray, prev_gray, &pairs );
        //writer.write_matches( image_list[i], r->keypoints.source, r->keypoints.reflection,
         //       r->matches, r->roi.source );
        //draw_match_by_hand( &drawn_matches, &cur_frame,
         //       &flipped, r->roi.source , r->roi.reflection,
         //       good_points.source, good_points.reflection );
        Scalar red (0,0,255);
        Scalar black(0,0,0);
        list<ARC_Pair>::iterator it=pairs.begin();
        while( it!=pairs.end() )
        {
            if( it->nNoMatch>0 )
            {
                if( !rematch( cur_frame, a.patch_size, *it, slope ) )
                {
                    if( it->nNoMatch++>10 )
                        it=pairs.erase( it );
                    continue;
                }
                it->nNoMatch=0;
                ++it->age;
                ++it;
                continue;
            }
            Point s, r, t;
            s = it->roi.source;
            r = it->roi.reflection;
            t = Point( 5, 5 );
            circle( drawn_matches, s, 3, red );
            stringstream sid;
            sid << it->id;

            putText( drawn_matches, ( sid.str() ).c_str(), s-t, FONT_HERSHEY_SIMPLEX, .3, 100 );
            circle( drawn_matches, r, 3, black );
            line( drawn_matches, s, r, black, 1, CV_AA, 0 );
            cout << image_list[i] << spc
                 << *it
                 << endl;
            ++it->age;
            ++it;
        }
        //Point2f src_pt( 320, 80 );

        //cout << imu.get_rotation_angle( src_pt, rotation_matrix ) <<endl;
        Point2f ol[2];
        slope_endpoints( slope, ol );
        line( drawn_matches, ol[0], ol[1], 200, 1, CV_AA, 0 );
        swap(prev_gray, gray);
        imshow( DEFAULT_WINDOW_NAME, drawn_matches );
        waitKey(5);
        vidout << drawn_matches;
        ++i;
    }
	return 0;
}
#else      // -----  not DEBUG_IMU  ----- 


// ===  FUNCTION  ======================================================================
//         Name:  main
//  Description:  Function for testing ARC_IMU
// =====================================================================================
int main ( int argc, char *argv[] )
{
    vector<string> image_list;
    vector<Point3f> imu_list;
    //int num=atoi(argv[4]);
    //int num=240;
    get_image_list( argv[1], &image_list );     // Reads in the image list.
    get_imu_list( argv[2], &imu_list );

    ARC_IMU imu;
    imu.set_A( A );
    // Init Video
    Mat first_frame=imread( image_list[0], CV_LOAD_IMAGE_COLOR );
    VideoWriter vidout;
    vidout.open( "mov.avi", CV_FOURCC('F','M','P','4'), 
            20.0, first_frame.size(), true );
    if( !vidout.isOpened() )
    {
        cerr << "Could not open video file: " << "mov.avi" << endl;
        exit( EXIT_FAILURE );
    }
    for( size_t i=0; i<image_list.size(); ++i )
    {
        list<ARC_Pair> outlist;
        Matx33d rotation_matrix = imu.calc_rotation_matrix( imu_list[i] );
        double theta = imu.get_rotation_angle( rotation_matrix );
        double slope = imu.theta_to_slope( theta );
        //double slope = atof(argv[3]);

        //Mat img = imread ( "../test/test.jpg", CV_LOAD_IMAGE_COLOR );           // open image 
        Mat img = imread ( image_list[i], CV_LOAD_IMAGE_COLOR );           // open image 
        Size outerPatchSize( 50, 50 );
        Size innerPatchSize( 10, 10 );


        getReflections( img, outerPatchSize, 10, slope, outlist);
        outlist.sort();
        outlist.remove_if( outside_theta(theta) );
        outlist.remove_if( below_threshold( 3 ) ) ;
        outlist.remove_if( overlap() );
        Scalar red (0,0,255);
        Scalar black(0,0,0);
        for( list<ARC_Pair>::iterator it=outlist.begin();
                it!=outlist.end(); ++it )
        {
            Point s,r;
            s = it->roi.source;
            r = it->roi.reflection;

            circle( img, s, 3, red );
            circle( img, r, 3, black );
            line( img, s, r, black, 1, CV_AA, 0 );
        }
            
        cout << "Slope: " << slope << endl;
        cout << "Theta: " << theta << endl;
        //getReflectionsPYR( img, outerPatchSize, innerPatchSize, slope, theta, outlist );
        //displayReflectionMatches( img, outerPatchSize, slope, theta, &outlist );
        vidout << img;
    }
    return EXIT_SUCCESS;
}				// ----------  end of function main  ---------- 
#endif     // -----  not DEBUG_IMU  ----- 

