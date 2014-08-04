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
#include <unistd.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/wait.h>
using std::cout;
using std::endl;
using std::cerr;

configuration conf;


/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  read_config
 *  Description:  
 * =====================================================================================
 */
    void
read_config ( FILE *fh, configuration *cfg )
{
    char *val, *line;

    val = line = (char *) calloc ( (size_t)(MAXLINE), sizeof(char) );
    
    if ( val==NULL ) { 
        fprintf ( stderr, "\ndynamic memory allocation failed\n" );
        exit (EXIT_FAILURE);
    }   

    while( fgets( val, MAXLINE, fh )!=NULL )
    {   
        char *key;
        key=strsep( &val, "=" );
        if( !strncmp(key, "cam", MAXLINE) )
        {   
            double fx, fy, cx, cy; 
            sscanf(val, "%lf,%lf,%lf,%lf", &fx, &fy, &cx, &cy);
            cfg->k(0,0)=fx;
            cfg->k(1,1)=fy;
            cfg->k(0,2)=cx;
            cfg->k(1,2)=cy;
            cfg->k(2,2)=1;
        }   
        else if( !strncmp(key, "camimu", MAXLINE) )
        {   
            cv::Vec4d qbw;
            sscanf( val, "%lf,%lf,%lf,%lf", &qbw[0], &qbw[1], &qbw[2], &qbw[3] );
            cfg->camIMU=Quaternion(qbw);
        }   
        else
        {   
            printf("Unknown option: %s\n", key);
            exit(EXIT_FAILURE);
        }   
    }   
    free (line);
    line    = NULL;
    return;
}		/* -----  end of function read_config  ----- */

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  get_angle
 *  Description:  Returns angle of rotation in camera plane in radians.
 * =====================================================================================
 */
    double
get_angle ( const Quaternion& qbw )
{
    const cv::Matx33d S (1,0,0,  // I-2*n*n.t
                         0,1,0,
                         0,0,-2);
    const cv::Vec3d x(320,240,1); // Use center of image.

    double angle;
    cv::Vec3d h, xs, xsr;  // Point, normalized, unit sphere, u.s. reflection
    cv::Matx33d Rcb, Rbw;  // Rotation matrices: camera-body, body-world
    cv::Vec3d diff;

    solve( conf.k, x, h );  // h=K.inv * x
    normalize( h, xs ); // Normalize h

    Rcb=conf.camIMU.rotation();
    Rbw=qbw.rotation();

    xsr = Rcb.t()*Rbw.t()*S*Rbw*Rcb*xs;

    diff=xs-xsr;
    angle = atan2(diff[1], diff[0]);

    return M_PI_2+angle;
}		/* -----  end of function get_angle  ----- */

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  get_imu_list
 *  Description:  
 * =====================================================================================
 */
    void
get_imu_list ( std::string filename, std::vector<cv::Point3f>* il )
{

    std::string    ifs_file_name = filename;                 /* input  file name */
    std::ifstream  ifs;                                /* create ifstream object */

    ifs.open ( ifs_file_name.c_str() );           /* open ifstream */
    if (!ifs) {
        std::cerr << "\nERROR : failed to open input  file " << ifs_file_name << std::endl;
        exit (EXIT_FAILURE);
    }
    std::string line;
    while( getline( ifs, line, '\n' ) )
    {
        double x, y, z;
        std::string fn;
        std::stringstream l(line);
        l >> fn >> x >> y >> z;
        il->push_back( cv::Point3f( x, y, z ) );
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
void get_image_list(std::string filename, std::vector<std::string>* il)
{
 
    std::string    ifs_file_name = filename;         /* input  file name */
    std::ifstream  ifs;                              /* create ifstream object */

    ifs.open ( ifs_file_name.c_str() );         /* open ifstream */
    if (!ifs) {
        std::cerr << "\nERROR : failed to open input  file " << ifs_file_name << std::endl;
        exit (EXIT_FAILURE);
    }
    std::string line;
    while( getline(ifs, line, '\n') )
    {
        il->push_back(line);
    }
    ifs.close ();                                 /* close ifstream */
}
void change_frame_number( int slider, void* fn )
{
    unsigned int* fn_typed = (unsigned int *) fn;
    *fn_typed = ( unsigned int )slider;
}

void change_good_features_to_track( int slider, void* gft )
{
    int* gft_typed = (int *) gft;
    *gft_typed = slider;
}

void change_patch_size( int slider, void* ps )
{
    int* ps_typed = (int *) ps;
    *ps_typed = slider;
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

void change_eig( int slider, void* eig )
{
    double* eig_typed = (double *) eig;
    *eig_typed = slider/100.0;
}

// ===  FUNCTION  ======================================================================
//         Name:  slope_endpoints
//  Description:  
// =====================================================================================
void slope_endpoints ( double angle, cv::Point2f* ol )
{
    double slope = tan(angle);

    ol[0] = cv::Point2d(320-240/slope,0);
    ol[1] = cv::Point2d(320+240/slope,480);

    return ;
}		// -----  end of function slope_endpoints  ----- 

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  Fork
 *  Description:  Wrapper for fork() function.
 * =====================================================================================
 */
    pid_t
Fork ( )
{
    pid_t p;
    if( (p=fork())==-1 )
    {
        perror("fork");
        exit(1);
    }
    return p;
}		/* -----  end of function Fork  ----- */


/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  Pipe
 *  Description:  Wrapper for pipe() function.
 * =====================================================================================
 */
    int
Pipe ( int pipefd[2] )
{
    int p;
    if( (p=pipe(pipefd))==-1 )
    {
        perror("pipe");
        exit(1);
    }
    return p;
}		/* -----  end of function Pipe  ----- */

// ===  FUNCTION  ======================================================================
//         Name:  update_regions
//  Description:  Removes low quality regions and add new regions.
// =====================================================================================
void update_regions ( cv::Mat& frame, std::list<ARC_Pair>* pairs,
        cv::Size patch_size, std::vector<cv::Point2f>& GFT, int N )
{
    std::list<ARC_Pair> temp[N];
    std::vector<cv::Point2f> features_to_match[N];
    pid_t pids[N];
    int pfds[N][2];
    for( int i=0; i<N; ++i )
    {
        Pipe( pfds[i] );
        if( GFT.size()>0 )
        {
            features_to_match[i].push_back( GFT.back() );
            GFT.pop_back();
        }
        else
        {
            cerr << "Minimum GFT too low." << endl;
        }
        if( (pids[i]=Fork())==0 )
        {
            close( pfds[i][0] );
            getReflections( frame, patch_size, temp+i, features_to_match[i] );
            ARC_Pair p = (temp+i)->front();
            write( pfds[i][1], &(p.roi.source), sizeof(cv::Point) );
            write( pfds[i][1], &(p.roi.reflection), sizeof(cv::Point) );
            write( pfds[i][1], &(p.nsigma), sizeof(double) );
            close( pfds[i][1] );
            _exit( 0 );
        }
    }
    for( int i=0; i<N; ++i )
    {
        int status;
        double ns;
        cv::Point src, ref;
        ARC_Pair *tmp=new ARC_Pair;
        close( pfds[i][1] );
        read( pfds[i][0], &src, sizeof(cv::Point) );
        read( pfds[i][0], &ref, sizeof(cv::Point) );
        read( pfds[i][0], &ns, sizeof(double) );
        tmp->nsigma=ns;
        tmp->roi.source=src;
        tmp->roi.reflection=ref;
        temp[i].push_back( *tmp );
        pairs->splice( pairs->end(), temp[i] );
        close( pfds[i][0] );
        waitpid( pids[i], &status, 0 );
        if( status!=0 )
        {
            perror("status");
            exit(1);
        }
    }
    
    //getReflections( frame, patch_size, temp, features_to_match );
    //getShorelinePairs( frame, patch_size, nregions, eig, temp );
    //Remove pair if not within detected shoreline margin
    //temp.remove_if( within_shore( frame.clone() ) );
    //pairs->splice( pairs->begin(), temp);
        
    return ;
}		// -----  end of function update_regions  ----- 

// ===  FUNCTION  ======================================================================
//         Name:  help()
//  Description:  Display options.
// =====================================================================================
void help( std::string program_name )
{
    cout 
         << "Usage: " << program_name << spc << "<list of image files> <frame gyro data> [options]"
         << std::endl
         << "OPTIONS" << std::endl

         << ARC_ARG_FEATURES_BEFORE_TRACK << tab 
         << "Run goodFeaturesToTrack in each frame before tracking." 
         << " Default: " << ARC_DEFAULT_FEATURES_BEFORE_TRACK << std::endl
         
         << ARG_SHOW_TRACKING << tab << "Show tracking (default)." << std::endl
         
         << ARG_VID_FILE << spc << "<filename>" << tab << "Set video output file." << std::endl

         << ARG_TXT_FILE << spc << "<filename>" << tab << "Set text output file." << std::endl

         << ARG_BLUR << tab << "[odd number]" << tab << "Median blur kernel size." << std::endl

         << ARC_ARG_THETA_DEV << spc << "[0-90]" << tab << "Set Max deviation of match slope from IMU slope." << spc
         << "Default: " << ARC_DEFAULT_THETA_DEV << std::endl

         << ARC_ARG_EIG << spc << "[0.01-1]" << tab << "Set eigenvalue threshold." 
         << spc << "Default: " << ARC_DEFAULT_EIG << std::endl

         << ARC_ARG_MAX_DIST << spc << "[N]" << tab << "Set max distance between source and reflection." 
         << spc << "Default: " << ARC_DEFAULT_MAX_DIST << std::endl

         << ARC_ARG_STD << spc << "[double]" << tab 
         << "Set number of standard deviations for acceptable matches." 
         << spc << "Default: " << ARC_DEFAULT_EIG << std::endl

         << ARC_ARG_START_FRAME << spc << "[int]" << tab 
         << "Set the frame number to start from."
         << spc << "Default: " << ARC_DEFAULT_START_FRAME << std::endl

         << ARC_ARG_PATCH_SIZE << spc << "[0-150]" << tab << "Set region patch size." << spc
         << "Default: " << ARC_DEFAULT_PATCH_SIZE<<"x"<<ARC_DEFAULT_PATCH_SIZE << std::endl

         << ARC_ARG_GFT_MIN << spc << "[N]" << tab << "Set minimum number of good features to track"
         << spc << "Default: " << ARC_DEFAULT_GFT_MIN << std::endl

         << ARC_ARG_NUM_GOOD_FEATURES_TO_TRACK << spc << "[0-50]" << tab 
         << "Set number of good features to track." << spc << "Default: " 
         << ARC_DEFAULT_NUM_GOOD_FEATURES_TO_TRACK << std::endl

         << ARC_ARG_NUM_REGIONS << spc << "[0-50]" << tab << "Set desired number of regions to track." << spc
         << "Actual number tracked may be less." << spc
         << "Default: " << ARC_DEFAULT_NUM_REGIONS << std::endl

         << ARG_REFRESH_COUNT << spc << "<N>" << tab << "Number of iterations before rematching. "
         << "Default: " << DEFAULT_REFRESH_COUNT << std::endl
         ;

    return ;
}		// -----  end of function help()  ----- 

// ===  FUNCTION  ======================================================================
//         Name:  pairs_to_points
//  Description:  Writes centers of ARC_Pair ROIs to source and reflection point vectors.
// =====================================================================================
void pairs_to_points ( cv::Mat gray, std::list<ARC_Pair>* pairs, 
        std::vector<cv::Point2f>* src, std::vector<cv::Point2f>* ref,
        bool fbt )
{
    for( std::list<ARC_Pair>::iterator it=pairs->begin();
            it!=pairs->end(); ++it )
    {
        cv::Point2f s, r;
        s = it->roi.source;
        r = it->roi.reflection;
        if( fbt )
        {
            cv::Mat masks, maskr;
            std::vector<cv::Point2f> vs, vr;
            cv::Point2f shift( 5, 5 );
            cv::Size sz( 10, 10 );
            cv::Rect rs, rr;

            rs = cv::Rect( s-shift, sz );
            rr = cv::Rect( r-shift, sz );

            masks = cv::Mat::zeros( gray.size(), CV_8UC1 );
            maskr = cv::Mat::zeros( gray.size(), CV_8UC1 );
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

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  flow_gft
 *  Description:  Performs flow on a set of good features to track. In place
 *  wrote of points->new points is allowed.
 * =====================================================================================
 */
    void
flow_gft ( cv::Mat gray, cv::Mat prev_gray, std::vector<cv::Point2f>& pts, 
        std::vector<cv::Point2f>& npts )
{
    if( pts.size()==0 || prev_gray.empty() ) return;

    std::vector<cv::Point2f> tmp,tmp2;
    cv::TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 0.03);
    cv::Size sub_pix_win_size(10,10);
    cv::Size win_size(31,31);
    std::vector<uchar> status;
    std::vector<float> error;

    calcOpticalFlowPyrLK( prev_gray, gray, pts, tmp,
            status, error, win_size, 3, termcrit, 0, 0.001 );
    for( size_t i=0; i<tmp.size(); ++i )
    {
        if( !status[i] ) continue;
        tmp2.push_back( tmp[i] );
    }
    npts = tmp2;
    return;
}		/* -----  end of function flow_gft  ----- */

// ===  FUNCTION  ======================================================================
//         Name:  track
//  Description:  Track matched points.
// =====================================================================================
bool track( cv::Mat gray, cv::Mat prev_gray, std::list<ARC_Pair>* pairs )
{
    cv::TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 0.03);
    cv::Size sub_pix_win_size(10,10);
    cv::Size win_size(31,31);
    PPC points, new_points;

    if( prev_gray.empty() )
        gray.copyTo(prev_gray);
    pairs_to_points( gray, pairs, &points.source, &points.reflection, false );

    // Do the tracking.
    size_t i;
    if( points.source.size()!=0 )
    {
        std::vector<uchar> source_status, reflection_status;
        std::vector<float> source_error, reflection_error;

        calcOpticalFlowPyrLK( prev_gray, gray, points.source, new_points.source,
                source_status, source_error, win_size, 3, termcrit, 0, 0.001 );

        calcOpticalFlowPyrLK(prev_gray, gray, points.reflection,
                new_points.reflection, reflection_status, 
                reflection_error, win_size, 3, termcrit, 0, 0.001);
        
        // Set lost points to (-1, -1), so we know they are lost.
        std::list<ARC_Pair>::iterator it=pairs->begin();
        for( i=0; i<(new_points.source.size()); i++ )
        {
            cv::Point sdel, rdel;
            //double smag, rmag;
            if( !source_status[i] || !reflection_status[i] ) 
            {
                it = pairs->erase( it );
                continue;
            }
            // Get difference between old and move.
            sdel = new_points.source[i] - points.source[i];
            rdel = new_points.reflection[i] - points.reflection[i];
            //cout << it->id << spc << sdel.y-rdel.y << std::endl;
            if( abs( sdel.y-rdel.y ) > 2 )
            {
                //cout << "Large SDEL-RDEL diff" << std::endl;
                it->nNoMatch=1;
                //++it;
                //continue;
            }
            
            // Shift by difference.
            it->roi.source = new_points.source[i];
            it->roi.reflection = new_points.reflection[i];
            ++it;
        }
    }

    return ( new_points.source.size()>0 && new_points.reflection.size()>0 ) ? true : false;
}		// -----  end of method ARC_Pair::track  ----- 

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
    patch_size = cv::Size( ARC_DEFAULT_PATCH_SIZE, ARC_DEFAULT_PATCH_SIZE );
    video_filename = DEFAULT_VID_FILENAME;
    text_filename = DEFAULT_TXT_FILENAME;
    good_features_to_track = ARC_DEFAULT_NUM_GOOD_FEATURES_TO_TRACK;
    eig = ARC_DEFAULT_EIG;
    std = ARC_DEFAULT_STD;
    max_dist = ARC_DEFAULT_MAX_DIST;
    start_frame = ARC_DEFAULT_START_FRAME;
    gft_min = ARC_DEFAULT_GFT_MIN;
    return;
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
        if( !strcmp(argv[i], ARC_ARG_FEATURES_BEFORE_TRACK ) ) a->features_before_track = true;

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
            a->patch_size=cv::Size( dim, dim );
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
        if( !strcmp(argv[i], ARC_ARG_NUM_GOOD_FEATURES_TO_TRACK) ) 
        {
            a->good_features_to_track=atof(argv[++i]);
            continue;
        }
        if( !strcmp(argv[i], ARC_ARG_EIG) ) 
        {
            a->eig=atof(argv[++i]);
            continue;
        }
        if( !strcmp(argv[i], ARC_ARG_STD) ) 
        {
            a->std=atof(argv[++i]);
            continue;
        }
        if( !strcmp(argv[i], ARC_ARG_MAX_DIST) ) 
        {
            a->max_dist=atof(argv[++i]);
            continue;
        }
        if( !strcmp(argv[i], ARC_ARG_START_FRAME) ) 
        {
            a->start_frame=atof(argv[++i]);
            continue;
        }
        if( !strcmp(argv[i], ARG_BLUR) ) 
        {
            a->blur=atof(argv[++i]);
            continue;
        }
        if( !strcmp(argv[i], ARC_ARG_GFT_MIN) ) 
        {
            a->gft_min=atof(argv[++i]);
            continue;
        }
    }
    return true;
}		/* -----  end of function get_arguments  ----- */



int main(int argc, char** argv)
{
    FILE	*img_fp;										/* input-file pointer */
    FILE	*qbw_fp;										/* input-file pointer */
    FILE    *conf_fp;
    std::list<ARC_Pair> pairs;                   // Container for selected reflections and matches.
    std::vector<cv::Point2f> GFT;
    char *image;
    double qbw[4];

    // Parse Arguments
    arguments a;
    a.arguments();                              // TODO should init automatically.
    cv::namedWindow( DEFAULT_WINDOW_NAME, CV_WINDOW_AUTOSIZE );

    if( !get_arguments(argc, argv, &a) )        // Parse command line args.
    {
        help( argv[0] );
        exit( EXIT_FAILURE );
    }

    if( (conf_fp=fopen(argv[3], "r"))==NULL )
        err_sys("fopen: config");
    read_config( conf_fp, &conf );
    
    char	*qbw_fp_file_name = argv[2];		/* input-file name    */

    qbw_fp	= fopen( qbw_fp_file_name, "r" );
    if ( qbw_fp == NULL ) {
        fprintf ( stderr, "couldn't open file '%s'; %s\n",
                qbw_fp_file_name, strerror(errno) );
        exit (EXIT_FAILURE);
    }
    
    char	*img_fp_file_name = argv[1];		/* input-file name    */
    img_fp	= fopen( img_fp_file_name, "r" );
    if ( img_fp == NULL ) {
        fprintf ( stderr, "couldn't open file '%s'; %s\n",
                img_fp_file_name, strerror(errno) );
        exit (EXIT_FAILURE);
    }
    
    image	= (char *) calloc ( (size_t)(1024), sizeof(char) );
    if ( image==NULL ) {
        fprintf ( stderr, "\ndynamic memory allocation failed\n" );
        exit (EXIT_FAILURE);
    }
    
    //get_regions( argv[2], &regions );           // Reads in the region list.

    // Create GUI objects
    unsigned int i = a.start_frame;                               // Image index
    int td = a.theta_dev * 100;
    int eig = a.eig * 100;
    cv::createTrackbar( "theta_dev", DEFAULT_WINDOW_NAME, &td, 
            100, change_theta_dev, &a.theta_dev );
    cv::createTrackbar( "eig", DEFAULT_WINDOW_NAME, &eig, 
            100, change_eig, &a.eig );
    cv::createTrackbar( "num_regions", DEFAULT_WINDOW_NAME, &a.num_regions, 
            50, change_num_regions, &a.num_regions );
    cv::createTrackbar( "patch_size_x", DEFAULT_WINDOW_NAME, &a.patch_size.width, 
            150, change_patch_size, &a.patch_size.width );
    cv::createTrackbar( "patch_size_y", DEFAULT_WINDOW_NAME, &a.patch_size.height, 
            150, change_patch_size, &a.patch_size.height );
    cv::createTrackbar( "good_features_to_track", DEFAULT_WINDOW_NAME, 
            (int*) &a.good_features_to_track, 100, change_good_features_to_track,
            &a.good_features_to_track );
    //cv::createTrackbar( "frame_number", DEFAULT_WINDOW_NAME, (int*) &i, 
     //       image_list.size(), change_frame_number, &i );

    // Init text file
    //ARC_Write writer( a.text_filename );

    // Init Video
    fscanf( img_fp, "%s", image );
    fscanf( qbw_fp, "%lf,%lf,%lf,%lf", qbw, qbw+1, qbw+2, qbw+3 );
    cv::Mat first_frame=cv::imread( image, CV_LOAD_IMAGE_COLOR );
    //cv::VideoWriter vidout;
    //vidout.open( a.video_filename, CV_FOURCC('F','M','P','4'), 
     //       20.0, first_frame.size(), true );
    //if( !vidout.isOpened() )
    //{
     //   std::cerr << "Could not open video file: " << a.video_filename << std::endl;
      //  exit( EXIT_FAILURE );
    //}

    // Init ARC_IMU for rotation matrix.
    ARC_IMU imu;
    imu.set_A( A );
    //Begin image loop.
    cv::Point2f mid_pt( 320, 240 );
    cv::Mat cur_frame, gray, prev_gray;
    while( fscanf( img_fp, "%s", image )!=EOF )
    {
        double angle;
        fscanf( qbw_fp, "%lf,%lf,%lf,%lf", qbw, qbw+1, qbw+2, qbw+3 );
        Quaternion quat( cv::Vec4d(qbw[0], qbw[1], qbw[2], qbw[3]) );
        angle = get_angle(quat);

        cv::Mat water_mask, edges;
        //cv::setTrackbarPos( "frame_number", DEFAULT_WINDOW_NAME, (int) i );

        cur_frame=cv::imread( image, CV_LOAD_IMAGE_UNCHANGED );           // open image 
        if ( !cur_frame.data ) {
            std::cerr << "\nERROR : failed to open input file " << image << std::endl;
            exit (EXIT_FAILURE);
        }
        cvtColor(cur_frame, gray, CV_BGR2GRAY);
        if( a.blur )
        {
            medianBlur( gray, gray, a.blur );        
        }
        cv::Mat drawn_matches;
        cur_frame.copyTo(drawn_matches);
        //find_water(drawn_matches.clone(), water_mask);

        // Filter pairs.
        // Flow GFT
        flow_gft( gray, prev_gray, GFT, GFT );
        if( GFT.size()<a.gft_min )
        {
            // Add more GFTs
            cv::Size shift( 20, 20 );
            cv::Mat mask = cv::Mat::ones(gray.size(),CV_8UC1)*255;
            for( std::list<ARC_Pair>::iterator it=pairs.begin();
                    it!=pairs.end(); ++it )
            {
                cv::Rect blockedRegionSource( it->roi.source-0.5*cv::Point(shift), shift );
                cv::Rect blockedRegionReflection( it->roi.reflection-0.5*cv::Point(shift), shift );
                rectangle( mask, blockedRegionSource, 0, CV_FILLED );
                rectangle( mask, blockedRegionReflection, 0, CV_FILLED );
            }
            goodFeaturesToTrack( gray, GFT, a.good_features_to_track, a.eig, 5, mask ); 
        }
        // Update regions.
        update_regions( cur_frame, &pairs, a.patch_size, GFT, 8 );
        pairs.remove_if( below_threshold( a.std ) ); // patch 50x50
        pairs.remove_if( outside_theta( M_PI_2, a.theta_dev ) );
        //pairs.remove_if( overlap( a.patch_size ) );
        pairs.remove_if( longer_than( a.max_dist ) );
		//pairs.remove_if( within_shore( cur_frame.clone() ) );//Remove pair if not within detected shoreline margin
        // track.
        track( gray, prev_gray, &pairs );
        cv::Scalar red (0,0,255);
        cv::Scalar black(0,0,0);
        std::list<ARC_Pair>::iterator it=pairs.begin();
        int num=0;
        while( it!=pairs.end() && num++<5 )
        {
            //if( it->age>5 )
            {
                cv::Point s, r, t;
                s = it->roi.source;
                r = it->roi.reflection;
                t = cv::Point( 5, 5 );
                circle( drawn_matches, s, 3, red );
                rectangle( drawn_matches, cv::Rect( s -0.5*cv::Point( a.patch_size ), a.patch_size ), red, 1 );
                std::stringstream sid;
                sid << it->id;

                cv::putText( drawn_matches, ( sid.str() ).c_str(), s-t, cv::FONT_HERSHEY_SIMPLEX, .3, 100 );
                circle( drawn_matches, r, 3, black );
                rectangle( drawn_matches, cv::Rect( r -0.5*cv::Point( a.patch_size ), a.patch_size ), black, 1 );
                if( it->nNoMatch>0 )
                {
                    cv::line( drawn_matches, s, r, black, 1, CV_AA, 0 );
                    it->nNoMatch=0;
                }
                else
                {
                    cv::line( drawn_matches, s, r, black, 1, CV_AA, 0 );
                }
                std::cout << *it
                     << std::endl;
            }
            ++it->age;
            ++it;
        }
        printf("\n");
        //Point2f src_pt( 320, 80 );

        //cout << imu.get_rotation_angle( src_pt, rotation_matrix ) <<std::endl;
        std::stringstream frame_number;
        frame_number << i;
        cv::Point2f ol[2];
        slope_endpoints( angle, ol );
        line( drawn_matches, ol[0], ol[1], 200, 1, CV_AA, 0 );
        cv::putText( drawn_matches, (frame_number.str()).c_str(), cv::Point(5,15) ,cv::FONT_HERSHEY_SIMPLEX, .5, 200 );
        swap(prev_gray, gray);
        cv::imshow( DEFAULT_WINDOW_NAME, drawn_matches );
        cv::waitKey(1);
        //vidout << drawn_matches;
        ++i;
    }
    if( fclose(qbw_fp) == EOF ) {			/* close input file   */
        fprintf ( stderr, "couldn't close file '%s'; %s\n",
                qbw_fp_file_name, strerror(errno) );
        exit (EXIT_FAILURE);
    }
    if( fclose(img_fp) == EOF ) {			/* close input file   */
        fprintf ( stderr, "couldn't close file '%s'; %s\n",
                img_fp_file_name, strerror(errno) );
        exit (EXIT_FAILURE);
    }
    free (image);
    image	= NULL;
	return 0;
}

