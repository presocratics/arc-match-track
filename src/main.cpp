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
using namespace std;


// ===  FUNCTION  ======================================================================
//         Name:  update_roi
//  Description:  Returns a new ROI centerd on the center of mass of the input
//  set of points. If the center of mass is NaN, then the input ROI is
//  returned.
//  =====================================================================================
Rect update_roi ( Rect roi, vector<Point2f> pts )
{
    Point2f scm;
    if( pts.size()==1 )
        scm = pts[0];
    else
    {
        Moments mome = moments( pts, false );
        scm = Point2f( mome.m10/mome.m00, mome.m01/mome.m00 );
        if( isnan( scm.x) || isnan( scm.y ) )
            return roi;
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
void prune_keypoints ( vector<KeyPoint>* train_kpt, vector<KeyPoint>* query_kpt, vector<DMatch>& matches )
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
        vector<DMatch>* matches, Rect roi, unsigned int direction )
{
    Point2f transform = -roi.tl();
    vector<Point2f> trans_train, trans_query;
    vector<DMatch>::iterator it=matches->begin();
    size_t i=0;
    while( it!=matches->end() )
    {
        Point2f new_train, new_query;
        if( train_pts[i]==Point2f(-1, -1) || query_pts[i]==Point2f(-1, -1) )
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
        Mat* object, Rect roi, 
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
    rectangle( *out_img, roi, Scalar( 50, 100, 150 ), 1 );
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
         << ARG_RANSAC_OFF << tab << "Disable ransac test." << endl
         << ARG_BLUR << tab << "Median blur scene for tracking." << endl

         << ARG_MATCH_RATIO << spc << "(0-1)" << tab << "Set ratio for knn matching test. "
         << "Default: " << DEFAULT_MATCH_RATIO << endl

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
//         Name:  track
//  Description:  Track matched points.
// =====================================================================================
bool track( Mat gray, Mat prev_gray, ARC_Pair* r )
{
    TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 0.03);
    Size sub_pix_win_size(10,10);
    Size win_size(31,31);
    vector<uchar> object_status, scene_status;
    vector<float> object_error, scene_error;
    PPC good_points, new_points;

    if( prev_gray.empty() )
        gray.copyTo(prev_gray);

    if( r->direction.track==DOWN )
    {
        keypoints_to_goodpoints( r->keypoints.source, r->keypoints.reflection,
                &good_points.source, &good_points.reflection,
                r->matches, r->roi.source, r->direction.match );
    }
    else
    {
        keypoints_to_goodpoints( r->keypoints.reflection, r->keypoints.source,
                &good_points.reflection, &good_points.source,
                r->matches, r->roi.reflection, r->direction.match );
    }
    // Do the tracking.
    size_t i;
    if( good_points.source.size()!=0 )
    {
        calcOpticalFlowPyrLK( prev_gray, gray, good_points.source, new_points.source,
                object_status, object_error, win_size, 3, termcrit, 0, 0.001 );
        // Set lost points to (-1, -1), so we know they are lost.
        for( i=0; i<(new_points.source.size()); i++ )
            if( !object_status[i] ) new_points.source[i]=Point2f(-1,-1); 
    }

    if( good_points.reflection.size()!=0 )
    {
        calcOpticalFlowPyrLK(prev_gray, gray, good_points.reflection,
                new_points.reflection, scene_status, 
                scene_error, win_size, 3, termcrit, 0, 0.001);
        for( i=0; i<new_points.reflection.size(); i++ )
            if( !scene_status[i] ) new_points.reflection[i] = Point2f(-1, -1);
    }
    // Update KeyPoints.
    if( r->direction.track==DOWN )
    {
        // Update ROI.
        if( new_points.source.size()>0 )
            r->roi.source = update_roi( r->roi.source, good_points.source );

        good_points_to_keypoints( new_points.source, &(r->keypoints.source),
            new_points.reflection, &(r->keypoints.reflection), &(r->matches), r->roi.source, 
            r->direction.match );
    }
    else
    {
        good_points_to_keypoints( new_points.reflection, &(r->keypoints.reflection),
            new_points.source, &(r->keypoints.source), &(r->matches), r->roi.reflection,
            r->direction.match );
    }

    return ( new_points.source.size()>0 ) ? true : false;
}		// -----  end of method ARC_Pair::track  ----- 

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
    blur = false;
    isSym = true;
    isRatio = true;
    isRansac = true;
    refresh_count = DEFAULT_REFRESH_COUNT;
    min_match_points=DEFAULT_MIN_MATCH_POINTS;
    match_ratio = DEFAULT_MATCH_RATIO;
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

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  get_regions
 *  Description:  Read in a list of regions and check for errors in format.
 *  File Format:  <x> <y> <width> <height> [direction] [slope]
 * =====================================================================================
 */
bool get_regions(string filename, vector<ARC_Pair>* regions)
{
    bool status=true;
    string    ifs_file_name = filename;                 /* input  file name */
    ifstream  ifs;                                /* create ifstream object */

    ifs.open ( ifs_file_name.c_str() );           /* open ifstream */
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
            region.direction.track = region.direction.match = direction;
            region.slope = -slope;
            region.iter_count = 0;
            regions->push_back(region);
        }
        else
        {
            cerr << "Invalid region at " << loc << " of size " << (Point) dim << endl;
            status=false;
        }
    }
    ifs.close ();                                 /* close ifstream */
    return status;
}

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
        if( !strcmp(argv[i], ARG_MATCH_RATIO) ) 
        {
            a->match_ratio=atof(argv[++i]);
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
    vector<ARC_Pair> regions;                   // Container for selected reflections and matches.

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
    get_regions( argv[2], &regions );           // Reads in the region list.

    // Init text file
    ARC_Write writer( a.text_filename );

    // Init Video
    Mat first_frame=imread( image_list[0], CV_LOAD_IMAGE_COLOR );
    VideoWriter vidout;
    vidout.open( a.video_filename, CV_FOURCC('F','M','P','4'), 
            15.0, first_frame.size(), true );
    if( !vidout.isOpened() )
    {
        cerr << "Could not open video file: " << a.video_filename << endl;
        exit( EXIT_FAILURE );
    }

    // initialize matching object.
    Ptr<FeatureDetector> pfd = new SurfFeatureDetector( 1 );
    Ptr<DescriptorExtractor> pde = new SurfDescriptorExtractor();
    Ptr<DescriptorMatcher> pdm = new FlannBasedMatcher();
    
    //Ptr<FeatureDetector> pfd = new SiftFeatureDetector( );
    //Ptr<DescriptorExtractor> pde = new SiftDescriptorExtractor();
    //Ptr<DescriptorMatcher> pdm = new BFMatcher( NORM_L1, false );

    double confidence = 0.99f;                  // Confidence ratio for RANSAC.
    double distance = 3.5;                      // Max distance from epipolar lines.
    bool rf = true;                             // Match refining in RANSAC.

    ARC_Match m;

    m.set_confidence( confidence );
    m.set_distance( distance );
    m.set_ratio( a.match_ratio );
    m.set_refineF( rf );
    m.set_min_points( a.min_match_points );
    m.set_ratio_test( a.isRatio );
    m.set_symmetry_test( a.isSym );
    m.set_ransac_test( a.isRansac );
    m.set_verbosity( a.verbosity );

    m.set_feature_detector( pfd );
    m.set_descriptor_extractor( pde );
    m.set_descriptor_matcher( pdm );

    //Begin image loop.
    Mat cur_frame, gray, prev_gray;
    for( size_t i=0; i<image_list.size(); i++ )
    {
        if( a.verbosity>=VERBOSE ) cout << "Frame: " << image_list[i] << endl;

        cur_frame=imread ( image_list[i], CV_LOAD_IMAGE_COLOR );           // open image 
        if ( !cur_frame.data ) {
            cerr << "\nERROR : failed to open input file " << image_list[i] << endl;
            exit (EXIT_FAILURE);
        }
        cvtColor(cur_frame, gray, CV_BGR2GRAY);
        if( a.blur )
            medianBlur( gray, gray, 7 );
        Mat drawn_matches;
        cur_frame.copyTo(drawn_matches);

        // Begin region loop.
        for( vector<ARC_Pair>::iterator r=regions.begin(); r!=regions.end(); ++r )
        {
            Mat object_mask, scene_mask;
            // Prepare object for matching.
            Mat flipped;
            if( r->direction.match==DOWN )
                flipped = process_object( &cur_frame, r->roi.source, &object_mask );
            else
                flipped = process_object( &cur_frame, r->roi.reflection, &object_mask );

            if( r->iter_count%a.refresh_count==0 ) // Refresh match.
            {
                // Remove all keypoints that don't have matches.
                // TODO: Handles direction.
                prune_keypoints( &(r->keypoints.source), &(r->keypoints.reflection),
                        r->matches );
                if( r->keypoints.source.size()<2 && a.verbosity>=VERY_VERBOSE )
                    cout << "main: Too few keypoints." << endl;

                // mask the search region
                Mat masked_frame;
                if( r->direction.match==DOWN )
                {
                    masked_frame = get_masked_frame( r->roi.source, r->slope,
                            r->direction.match, &cur_frame, &scene_mask );
                }
                else
                {
                    masked_frame = get_masked_frame( r->roi.reflection, r->slope,
                            r->direction.match, &cur_frame, &scene_mask );
                }

                // Run matcher
                bool isMatch;
                if( r->direction.match==DOWN)
                {
                    if( a.debug==DEBUG ) cout << "Enter match direction down." << endl;
                    isMatch = m.match( masked_frame, flipped, 
                                        scene_mask, object_mask, r->matches,
                                        r->keypoints.reflection, r->keypoints.source );
                    if( a.debug==DEBUG ) cout << "Leave match direction down." << endl;
                }
                else
                {
                    if( a.debug==DEBUG ) cout << "Enter match direction up." << endl;
                    isMatch = m.match( masked_frame, flipped, 
                                        scene_mask, object_mask, r->matches,
                            r->keypoints.source, r->keypoints.reflection );
                    if( a.debug==DEBUG ) cout << "Leave match direction up." << endl;
                }
                if( isMatch )
                {
                    ++r->iter_count;
                    if( a.verbosity>=VERBOSE ) cout << "Match found." << endl;
                    if( a.show_match==SHOW_MATCHES )
                    {
                        imshow( DEFAULT_WINDOW_NAME, flipped );
                        waitKey( 0 );
                        imshow( DEFAULT_WINDOW_NAME, masked_frame );
                        waitKey( 0 );

                        Mat match_img;
                        if( r->direction.match==DOWN )
                        {
                        drawMatches( cur_frame, r->keypoints.reflection,
                              flipped, r->keypoints.source,
                              r->matches, match_img, Scalar::all(-1), Scalar::all(-1),
                              vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
                        }
                        else
                        {
                        drawMatches( cur_frame, r->keypoints.source,
                              flipped, r->keypoints.reflection,
                              r->matches, match_img, Scalar::all(-1), Scalar::all(-1),
                              vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
                        }
                        imshow( DEFAULT_WINDOW_NAME, match_img );
                        waitKey(0);

                    }
                    // When we track we'll remember which direction the match came from.
                    r->direction.track = r->direction.match; 
                    r->direction.match = DOWN;
                }
                else
                {
                    // TODO: any other action if no match found?
                    // Maybe we should remove a region if it goes N cycles without a match.
                    r->iter_count = 0 ;
                }
            }
            else                                // Perform tracking.
            {
                r->iter_count+=1;
                PPC good_points, old_good_points;

                if( a.debug==DEBUG )
                {
                    if( r->direction.track==DOWN )
                    {
                        // train, query, train, query, matches, roi
                        keypoints_to_goodpoints( r->keypoints.source, r->keypoints.reflection,
                                &old_good_points.source, &old_good_points.reflection,
                                r->matches, r->roi.source, r->direction.match );
                    }
                    else
                    {
                        keypoints_to_goodpoints( r->keypoints.reflection, r->keypoints.source,
                                &old_good_points.reflection, &old_good_points.source,
                                r->matches, r->roi.reflection, r->direction.match );
                    }
                }

                if( a.verbosity>=VERY_VERBOSE )
                {
                    if( r->direction.track==DOWN )
                        cout << "ROI at " << r->roi.source.tl() << (Point2f) r->roi.source.size() << endl;
                    else
                        cout << "ROI at " << r->roi.reflection.tl() << (Point2f) r->roi.reflection.size() << endl;
                }
                if ( !track(gray, prev_gray, &(*r) ) ) r->iter_count=0;
                if( a.verbosity>=VERY_VERBOSE )
                {
                    if( r->direction.track==DOWN )
                        cout << "New ROI at " << r->roi.source.tl() << (Point2f) r->roi.source.size() << endl;
                    else
                        cout << "New ROI at " << r->roi.reflection.tl() << (Point2f) r->roi.reflection.size() << endl;
                }

                if( r->direction.track==DOWN )
                {
                    // train, query, train, query, matches, roi
                    keypoints_to_goodpoints( r->keypoints.source, r->keypoints.reflection,
                            &good_points.source, &good_points.reflection,
                            r->matches, r->roi.source, r->direction.match );
                }
                else
                {
                    keypoints_to_goodpoints( r->keypoints.reflection, r->keypoints.source,
                            &good_points.reflection, &good_points.source,
                            r->matches, r->roi.reflection, r->direction.match );
                }
                if( a.debug==DEBUG )
                {
                    cout << "main: track:" << endl;
                    for( size_t i=0; i<old_good_points.source.size(); ++i )
                    {
                        cout << "SOURCE:" << endl;
                        cout << "Before: " << old_good_points.source[i] << tab
                             << "After: " << good_points.source[i] << endl;
                        cout << "REFLECTION:" << endl;
                        cout << "Before: " << old_good_points.reflection[i] << tab
                             << "After: " << good_points.reflection[i] << endl;
                    }
                }
                
                writer.write_matches( i, r->keypoints.source, r->keypoints.reflection,
                        r->matches, r->roi.source );
                draw_match_by_hand( &drawn_matches, &cur_frame,
                        &flipped, ( r->direction.track==DOWN ) ? r->roi.source : r->roi.reflection,
                        good_points.source, good_points.reflection );
            }
        }
        swap(prev_gray, gray);
        imshow( DEFAULT_WINDOW_NAME, drawn_matches );
        waitKey(5);
        vidout << drawn_matches;
    }
	return 0;
}
