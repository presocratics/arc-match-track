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

void change_radius( int slider, void* r )
{
    float* r_typed = (float *) r;
    *r_typed = slider/100.0;
}

void change_match_ratio( int slider, void* mr )
{
    double* mr_typed = (double *) mr;
    *mr_typed = slider/100.0;
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
//         Name:  slope_filter
//  Description:  Checks if regions don't deviate too far from slope.
// =====================================================================================
bool slope_filter ( Point2f src_pt, Point2f ref_pt, double imu_theta, int max_dev )
{
    //Point2f del = pair.roi.reflection.tl() - pair.roi.source.tl();
    //float roi_slope = del.y/del.x;
    // Create and set A matrix.
    cout << "Slope dev: " << max_dev << endl;
    imu_theta = imu_theta * 180/M_PI;           // Convert to degrees.
    double match_theta = (atan2( ref_pt.y-src_pt.y, ref_pt.x-src_pt.x ) * 180/M_PI);
    
    cout << "Diff: " << abs(match_theta - imu_theta ) << endl;
    return ( abs(match_theta-imu_theta)<max_dev );
}		// -----  end of function slope_filter  ----- 

// ===  FUNCTION  ======================================================================
//         Name:  update_regions
//  Description:  Removes low quality regions and add new regions.
// =====================================================================================
void update_regions ( Mat& frame, list<ARC_Pair>* pairs,
        unsigned int nregions, Size patch_size, double slope, double theta )
{
    //pairs->clear();
    cout << "Num regions: " << nregions << endl;
    cout << "Patch size: " << Point(patch_size) << endl;
    // Get new regions.
    //if( pairs->size()<nregions )
    if( 1 )
    {
        getReflections( frame, patch_size, nregions, slope, *pairs );
        //getReflectionsPYR( frame, patch_size, Size( 10, 10 ), slope, theta, *pairs );
    }
    pairs->remove_if( below_threshold( 3 ) );
    pairs->remove_if( outside_theta( theta ) );
    pairs->remove_if( overlap() );
        
    return ;
}		// -----  end of function update_regions  ----- 

// ===  FUNCTION  ======================================================================
//         Name:  update_roi
//  Description:  Returns a new ROI centerd on the center of mass of the input
//  set of points. If the center of mass is NaN, then the input ROI is
//  returned.
//  =====================================================================================
Rect update_roi ( Rect roi, vector<Point2f> pts )
{
    Point2f scm;
    //TODO Moments doesn't work...
    if( pts.size()>0 )
        scm = pts[0];
    else
    {
        Moments mome = moments( pts, false );
        scm = Point2f( mome.m10/mome.m00, mome.m01/mome.m00 );
        if( isnan( scm.x) || isnan( scm.y ) )
        {
            cerr << "ERROR: Failed to update roi at " << roi.tl() << endl;
            return roi;
        }
    }
    Point2f tl;
    tl.x =scm.x - roi.width/2;
    tl.y =scm.y - roi.height/2;
    return Rect( tl, roi.size() );
}		// -----  end of function update_roi  ----- 

// ===  FUNCTION  ======================================================================
//         Name:  prune_keypoints
//  Description:  Removes all keypoints that do not have an associated match.
// =====================================================================================
void prune_keypoints ( vector<KeyPoint>* train_kpt, vector<KeyPoint>* query_kpt,
        vector<DMatch>& matches )
{
    vector<KeyPoint> new_train_kpt, new_query_kpt;
    for( vector<DMatch>::iterator it=matches.begin() ;
            it!=matches.end(); ++it )
    {
        KeyPoint tkp, qkp;
        tkp = (*train_kpt)[ it->trainIdx ];
        qkp = (*query_kpt)[ it->queryIdx ];
        new_train_kpt.push_back( tkp );
        new_query_kpt.push_back( qkp );
    }
    (*train_kpt) = new_train_kpt;
    (*query_kpt) = new_query_kpt;
    return ;
}		// -----  end of function prune_keypoints  ----- 

// ===  FUNCTION  ======================================================================
//         Name:  reflect_point
//  Description:  Reflects point over the middle row of a roi.
// =====================================================================================
inline Point2f reflect_point ( Point2f point, Rect roi )
{
    // flip the points over the middle row
    float num_rows;
    int middle_row;
    float new_row;

    num_rows = roi.height;                      // Typecast int->float for next step.
    middle_row = ceil( num_rows/2 );
    new_row = 2*middle_row - point.y;           // Calculate new row assignment.
    point.y = new_row;
    return point ;
}		// -----  end of function reflect_point  ----- 

// ===  FUNCTION  ======================================================================
//         Name:  keypoints_to_goodpoints
//  Description:  Converts a set of keypoints to a set of good points. A vector
//  of points is extracted from the keypoints. Then the subvector of
//  good_points is extracted, the points are flipped across the roi and then
//  transposed to the scene's origin.
//  =====================================================================================
void keypoints_to_goodpoints ( vector<KeyPoint>& kpt_train, vector<KeyPoint>& kpt_query,
        vector<Point2f>* good_train, vector<Point2f>* good_query, 
        vector<DMatch>& matches, Rect roi, unsigned int direction )
{
    Point2f transform = roi.tl();
    // Get the points
    vector<Point2f> pt_train, pt_query;
    // Extract all points from the KeyPoints.
    KeyPoint::convert( kpt_train, pt_train ); 
    KeyPoint::convert( kpt_query, pt_query ); 

    // Push only the points that have a match onto new set of points.
    for( vector<DMatch>::iterator it=matches.begin() ;
            it!=matches.end(); ++it )
    {
        Point2f new_train, new_query;
        if( direction==DOWN )
        {
            new_train = reflect_point( pt_train[ it->trainIdx ], roi ) + transform ;
            new_query = pt_query[ it->queryIdx ] ;
        }
        else
        {
            new_train = pt_train[ it->trainIdx ] ;
            new_query = reflect_point( pt_query[ it->queryIdx ], roi ) + transform ;
        }
        good_train->push_back( new_train );
        good_query->push_back( new_query );
    }
    return ;
}		// -----  end of function keypoints_to_goodpoints  ----- 

// ===  FUNCTION  ======================================================================
//         Name:  good_points_to_keypoints
//  Description:  Update KeyPoint vector with new locations. Removes the
//  transform, reflects back to the flipped image, then updates point locations.
//  =====================================================================================
void good_points_to_keypoints( vector<Point2f> train_pts, vector<KeyPoint>* train_kpt,
        vector<Point2f> query_pts, vector<KeyPoint>* query_kpt,
        vector<DMatch>* matches, Rect roi, unsigned int direction,
        double theta, int max_dev )
{
    Point2f transform = -roi.tl();
    vector<Point2f> trans_train, trans_query;
    vector<DMatch>::iterator it=matches->begin();
    size_t i=0;
    while( it!=matches->end() )
    {
        Point2f new_train, new_query;
        if( train_pts[i]==Point2f(-1, -1) 
                || query_pts[i]==Point2f(-1, -1)
                || !slope_filter( train_pts[i], query_pts[i], theta, max_dev ))
        {
            // Tracking point lost, remove from matches.
            ++i;
            it = matches->erase(it);
            continue; 
        }
        if( direction==DOWN )
        {
            new_train = reflect_point( train_pts[i] + transform, roi );
            new_query = query_pts[i];
        }
        else
        {
            new_train = train_pts[i];
            new_query = reflect_point( query_pts[i] + transform, roi );
        }
        (*train_kpt)[ it->trainIdx ].pt = new_train;
        (*query_kpt)[ it->queryIdx ].pt = new_query;
        ++i;
        ++it;
    }
    return ;
}		// -----  end of function good_points_to_keypoints  ----- 

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
//         Name:  transpose
//  Description:  Transposes a set of points to an ROI.
// =====================================================================================
void transpose( vector<Point2f>& pts, Rect roi )
{
    Point2f c = Point2f( roi.x, roi.y );
    for( size_t i=0; i<pts.size(); ++i ) pts[i] += c;
    return ;
}		// -----  end of function transpose  ----- 

// ===  FUNCTION  ======================================================================
//         Name:  extract_good_points
//  Description:  Uses matches to put matching points into new vectors, indices are aligned.
// =====================================================================================
void extract_good_points( vector<Point2f>& in_train_pts, vector<Point2f>* out_train_pts,
        vector<Point2f>& in_query_pts, vector<Point2f>* out_query_pts,
        vector<DMatch>& matches )
{
    for( vector<DMatch>::iterator it=matches.begin() ;
            it!=matches.end(); ++it )
    {
        out_train_pts->push_back( in_train_pts[it->trainIdx] );
        out_query_pts->push_back( in_query_pts[it->queryIdx] );
    }
    return ;
}		// -----  end of function extract_good_points  ----- 

// ===  FUNCTION  ======================================================================
//         Name:  process_object
//  Description:  Crops frame to roi, ensuring frame bounds are not exceeded.
//  Returns the flipped crop. Mask pointer input is the mask.
//  TODO: Mask may not be necessary.
//  =====================================================================================
Mat process_object ( Mat* frame, Rect roi, Mat* mask )
{
    // Flip the roi
    Mat object, flipped;
    Rect scene_rect( Point( 0, 0 ), frame->size() );
    object = (*frame)(roi & scene_rect);
    flip(object, flipped, 0);

    return flipped;
    //return masked_frame;
}		// -----  end of function process_object  ----- 

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
         << ARG_SHOW_MATCHES << tab << "Show matches." << endl
         << ARG_SHOW_TRACKING << tab << "Show tracking (default)." << endl
         << ARG_VID_FILE << spc << "<filename>" << tab << "Set video output file." << endl
         << ARG_TXT_FILE << spc << "<filename>" << tab << "Set text output file." << endl
         << ARG_RATIO_OFF << tab << "Disable ratio test." << endl
         << ARG_SYMTEST_OFF << tab << "Disable symmetry test (DOES NOT WORK)." << endl
         << ARG_RANSAC_OFF << tab << "Disable ransac test (default)." << endl
         << ARG_RANSAC_ON << tab << "Enable ransac test." << endl
         << ARG_BLUR << tab << "Median blur scene for tracking." << endl
         << ARG_NO_BLUR << tab << "No median blur scene for tracking." << endl

         << ARG_MATCH_RATIO << spc << "(0-1)" << tab << "Set ratio for knn matching test." << spc
         << "Default: " << DEFAULT_MATCH_RATIO << endl

         << ARC_ARG_THETA_DEV << spc << "[0-90]" << tab << "Set Max deviation of match slope from IMU slope." << spc
         << "Default: " << ARC_DEFAULT_THETA_DEV << endl

         << ARC_ARG_PATCH_SIZE << spc << "[0-150]" << tab << "Set region patch size." << spc
         << "Default: " << ARC_DEFAULT_PATCH_SIZE<<"x"<<ARC_DEFAULT_PATCH_SIZE << endl

         << ARC_ARG_RADIUS << spc << "[0-1]" << tab << "Set match radius threshold." << spc
         << "Default: " << ARC_DEFAULT_RADIUS << endl

         << ARC_ARG_NUM_REGIONS << spc << "[0-50]" << tab << "Set desired number of regions to track." << spc
         << "Actual number tracked may be less." << spc
         << "Default: " << ARC_DEFAULT_NUM_REGIONS << endl

         << ARG_MIN_MATCH_POINTS << spc << "<N>" << tab << "Minimum match points after "
         << "symmetry test. Default: " << DEFAULT_MIN_MATCH_POINTS << endl
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
void pairs_to_points ( Mat gray, list<ARC_Pair>* pairs, vector<Point2f>* src, vector<Point2f>* ref )
{
    for( list<ARC_Pair>::iterator it=pairs->begin();
            it!=pairs->end(); ++it )
    {
        Point2f s, r;
        Rect little_s, little_r;
        vector<Point> sv, rv;
        little_r = it->roi.reflection;
        little_s = it->roi.source;
        // TODO: dynamic sizing.
        little_r += Point( 20, 20 );
        little_r -= Size(40, 40);
        little_s += Point( 20, 20 );
        little_s -= Size(40, 40);

        Mat mask_s, mask_r;
        mask_s = Mat::zeros( gray.size(), CV_8UC1 );
        mask_r = Mat::zeros( gray.size(), CV_8UC1 );
        rectangle( mask_s, little_s, 255, CV_FILLED );
        rectangle( mask_r, little_r, 255, CV_FILLED );
        goodFeaturesToTrack( gray, sv, 1, 0.01, 10, mask_s, 3, 0, 0.04);
        goodFeaturesToTrack( gray, rv, 1, 0.01, 10, mask_r, 3, 0, 0.04);
        if( rv.size()>0 && sv.size()>0 )
        {
            s=sv[0];
            r=rv[0];
        //    circle( img, s, 3, red );
         //   circle( img, r, 3, black );
          //  line( img, s, r, black, 1, 8, 0 );
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
    pairs_to_points( gray, pairs, &points.source, &points.reflection );
    for( size_t i=0; i<points.source.size(); ++i )
        cout << points.source[i] << endl;
    for( size_t i=0; i<points.reflection.size(); ++i )
        cout << points.reflection[i] << endl;


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
            if( !source_status[i] || !reflection_status[i] ) 
            {
                it = pairs->erase( it );
                continue;
            }
            // Get difference between old and move.
            sdel = new_points.source[i] - points.source[i];
            rdel = new_points.reflection[i] - points.reflection[i];
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
// ===  FUNCTION  ======================================================================
//         Name:  get_masked_frame
//  Description:  Masks the frame based on slope and roi. Mask returned by pointer.
// =====================================================================================
Mat get_masked_frame ( Rect roi, double slope, unsigned int dir, Mat* frame, Mat* mask )
{
    Point corners[1][4];
    //Set the frame
    *mask=Mat::zeros( frame->size(), CV_8UC1 );
    Mat masked_frame;
    if( slope==0 )
    {
        // TODO: Could use direction handling here.
        corners[0][0] = roi.br();
        corners[0][1] = Point( frame->cols, roi.y+roi.height );
        corners[0][2] = corners[0][1]-Point( 0, roi.height );
        corners[0][3] = corners[0][0]-Point( 0, roi.height );
    }
    else if( isinf( slope ) )
    {
        if( dir==DOWN )
        {
            corners[0][0] = Point( roi.x, frame->rows );
            corners[0][1] = Point( roi.x, roi.y+roi.height);
        }
        else
        {
            corners[0][0] = roi.tl();
            corners[0][1] = Point( roi.x, 0 );
        }
        corners[0][2] = corners[0][1]+Point( roi.width, 0);
        corners[0][3] = corners[0][0]+Point( roi.width, 0 );
    }
    else
    {
        if( dir==DOWN )
        {
            corners[0][0].x = ( int ) ( (frame->rows + slope*roi.x-roi.y)/slope );
            corners[0][0].y = frame->rows;
            corners[0][1] = roi.tl()+Point(0,roi.height);
        }
        else
        {
            corners[0][0] = Point( ( int )( (-roi.y + slope * roi.x ) / slope ), 0 );
            corners[0][1] = roi.tl();
        }
        corners[0][2] = corners[0][1] + Point(roi.width, 0);
        corners[0][3] = corners[0][0] + Point(roi.width, 0);
    }

    // This is weird, but follows OpenCV docs.
    const Point* ppt[1] = { corners[0] };
    const int npt[] = { 4 };

    fillPoly( *mask, ppt, npt, 1, 255 );
    frame->copyTo(masked_frame, *mask);
    return masked_frame;
}		// -----  end of function get_masked_frame  ----- 

/*
 * arguments constructor
 */
void arguments::arguments()
{
    blur = ARC_DEFAULT_BLUR;
    isSym = true;
    isRatio = true;
    isRansac = ARC_DEFAULT_RANSAC;
    refresh_count = DEFAULT_REFRESH_COUNT;
    min_match_points=DEFAULT_MIN_MATCH_POINTS;
    match_ratio = DEFAULT_MATCH_RATIO;
    theta_dev = ARC_DEFAULT_THETA_DEV;
    num_regions = ARC_DEFAULT_NUM_REGIONS;
    patch_size = Size( ARC_DEFAULT_PATCH_SIZE, ARC_DEFAULT_PATCH_SIZE );
    radius = ARC_DEFAULT_RADIUS;
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
        if( !strcmp(argv[i], ARG_BLUR) ) a->blur = true;
        if( !strcmp(argv[i], ARG_RATIO_OFF) ) a->isRatio = false;
        if( !strcmp(argv[i], ARG_SYMTEST_OFF) ) a->isSym = false;
        if( !strcmp(argv[i], ARG_RANSAC_OFF) ) a->isRansac = false;
        if( !strcmp(argv[i], ARG_DEBUG_MODE) ) a->debug=DEBUG;
        if( !strcmp(argv[i], ARG_VERBOSE) ) a->verbosity=VERBOSE;
        if( !strcmp(argv[i], ARG_VERY_VERBOSE) ) a->verbosity=VERY_VERBOSE;
        if( !strcmp(argv[i], ARG_VERY_VERY_VERBOSE) ) a->verbosity=VERY_VERY_VERBOSE;
        if( !strcmp(argv[i], ARG_VERY_VERY_VERBOSE) ) a->verbosity=VERY_VERY_VERBOSE;

        if( !strcmp(argv[i], ARG_SHOW_MATCHES) ) a->show_match=SHOW_MATCHES;

        if( !strcmp(argv[i], ARG_SHOW_TRACKING) ) a->show_track=SHOW_TRACKING;
        if( !strcmp(argv[i], ARG_MIN_MATCH_POINTS) ) 
        {
            a->min_match_points=atoi(argv[++i]);
            continue;
        }
        if( !strcmp(argv[i], ARC_ARG_RADIUS) ) 
        {
            a->radius=atof(argv[++i]);
            continue;
        }
        if( !strcmp(argv[i], ARG_MATCH_RATIO) ) 
        {
            a->match_ratio=atof(argv[++i]);
            continue;
        }
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
            << "Match Ratio:" << tab << a.match_ratio << endl
            << "Min Match Points:" << tab << a.min_match_points << endl
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
    while( i<image_list.size() )
    {
        setTrackbarPos( "frame_number", DEFAULT_WINDOW_NAME, (int) i );
        if( a.verbosity>=VERBOSE ) cout << "Frame: " << image_list[i] << endl;
        Matx33d rotation_matrix = imu.calc_rotation_matrix( imu_list[i] );
        double theta = imu.get_rotation_angle( rotation_matrix );
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
        if( i%50==0 )
            update_regions( cur_frame, &pairs, a.num_regions, a.patch_size, slope, theta );
        if( 0 )
        //if( i%200==0 )
        {
            // Update reflection ROIs.
            cout << "Update reflection ROIs not implemented." << endl;
        }
        else
        {
            // track.
            track( gray, prev_gray, &pairs );
            //writer.write_matches( image_list[i], r->keypoints.source, r->keypoints.reflection,
             //       r->matches, r->roi.source );
            //draw_match_by_hand( &drawn_matches, &cur_frame,
             //       &flipped, r->roi.source , r->roi.reflection,
             //       good_points.source, good_points.reflection );
        } 
        Scalar red (0,0,255);
        Scalar black(0,0,0);
        for( list<ARC_Pair>::iterator it=pairs.begin();
                it!=pairs.end(); ++it )
        {
            Point s,r;
            s = Point( it->roi.source.tl()+.5*Point(it->roi.source.size()) );
            r = Point( it->roi.reflection.tl()+.5*Point(it->roi.reflection.size()) );
            circle( drawn_matches, s, 3, red );
            circle( drawn_matches, r, 3, black );
            line( drawn_matches, s, r, black, 1, 8, 0 );
            cout << *it << endl;
        }
        //Point2f src_pt( 320, 80 );

        //cout << imu.get_rotation_angle( src_pt, rotation_matrix ) <<endl;
        Point2f ol[2];
        slope_endpoints( slope, ol );
        line( drawn_matches, ol[0], ol[1], 200, 3 );
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
            Rect little_r;
            vector<Point> rv;
            little_r = it->roi.reflection;
            little_r += Point( 20, 20 );
            little_r -= Size(40, 40);
            Mat gray;
            Mat mask;
            cvtColor(img, gray, CV_BGR2GRAY);
            mask = Mat::zeros( img.size(), CV_8UC1 );
            rectangle( mask, little_r, 255, CV_FILLED );
            goodFeaturesToTrack( gray, rv, 1, 0.01, 10, mask, 3, 0, 0.04);
            s = Point( it->roi.source.tl()+.5*Point(outerPatchSize) );
            //r = Point( it->roi.reflection.tl()+.5*Point(outerPatchSize) );
            if( rv.size()>0 )
            {
                r=rv[0];
                circle( img, s, 3, red );
                circle( img, r, 3, black );
                line( img, s, r, black, 1, 8, 0 );
            }
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

