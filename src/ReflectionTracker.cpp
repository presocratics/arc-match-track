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
using namespace std;
using namespace cv;

int verbosity;
bool displayRegions;
bool displayWindows;


//RETURNS THE TOP-LEFT CORNER OF THE REFLECTION OF sourceTemplate ON image
Rect findBestMatchLocation( double slope, Mat image, Rect source_rect, 
        double* nsigma, Mat mask )
{
    Mat image_copy = image.clone();
    
    // Create template.
    Mat image_template_copy = image.clone();
    Mat sourceTemplate = image_template_copy( source_rect );
    flip( sourceTemplate, sourceTemplate, 0 );

    // Creates results matrix where the top left corner of the 
    // template is slid across each pixel of the source
    int result_cols = image.cols-sourceTemplate.cols+1;
    int result_rows = image.rows-sourceTemplate.rows+1;
    Mat result;
    result.create( result_cols,result_rows, CV_32FC1 );

    // Mask image to match only in selected ROI.
    if( !mask.empty() )
    {
        Mat tmp;
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


    Mat search_image;
    search_image = image_copy;

    //matchTemplate( masked_scene, sourceTemplate, result, match_method );
    matchTemplate( search_image, sourceTemplate, result, match_method );

    if( verbosity>=VERY_VERBOSE ) 
        cout << "Ran matchTemplate and normalized\n";
    double minVal, maxVal; 
    Point minLoc, maxLoc, matchLoc;
    minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );
     
    if( match_method == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED )
    {
        matchLoc = minLoc;	
    }
    else 
    {
        matchLoc = maxLoc;
    }
    Scalar mean, stddev;
    meanStdDev( result, mean, stddev, Mat() );
    *nsigma = ( maxVal-mean[0] )/ stddev[0];
    // matchLoc is the location of the top left corner of the reflection
    // that matchTemplate found


    if( verbosity==VERY_VERBOSE ) 
        cout << "Max value from results matrix: " << result.at<float>(matchLoc.x,matchLoc.y) << endl;

    bool debugResults = false;
    if( debugResults ) 
    {
        int surroundingPixel = 5;
        cout << "TL: " << result.at<float>(matchLoc.x-surroundingPixel,matchLoc.y-surroundingPixel)
             << " T: " << result.at<float>(matchLoc.x,matchLoc.y-surroundingPixel)
             << " TR: " << result.at<float>(matchLoc.x+surroundingPixel,matchLoc.y-surroundingPixel)
             << endl;

        cout << "BL: " << result.at<float>(matchLoc.x-surroundingPixel,matchLoc.y+surroundingPixel)
             << " B: " << result.at<float>(matchLoc.x,matchLoc.y+surroundingPixel) 
             << " BR: " <<result.at<float>(matchLoc.x+surroundingPixel,matchLoc.y+surroundingPixel)
             << endl;
        Scalar reflectionColor (232,49,190);
        rectangle( result, matchLoc, Point( matchLoc.x+sourceTemplate.cols, matchLoc.y+sourceTemplate.rows ), reflectionColor , 2, 8, 0 );
        imshow( "Source", result);//not right	
        namedWindow( "Results", CV_WINDOW_AUTOSIZE );
        imshow( "Results", result );
        moveWindow( "Results", 700, 0 );
        waitKey( 0 );
    }
    return Rect( matchLoc, source_rect.size() );
}


/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  rematch
 *  Description:  
 * =====================================================================================
 */
    bool
rematch ( Mat frame, Size patchSize, ARC_Pair& pair, double slope )
{
    if( !frame.data )
    {
        cout << "Image couldn't be loaded." << endl;
        exit( EXIT_FAILURE );
    }
    Mat sourceCopy;
    cvtColor( frame, sourceCopy, CV_RGB2GRAY, 1 );
    Rect frame_rect( Point(0, 0), frame.size() );
    Rect a, b;
    double nsigma;
    // Get Rect A
    a = Rect( pair.roi.source-( .5*Point( patchSize ) ), patchSize ) & frame_rect;
    if( a.width==0 || a.height==0 ) return false;
    // Get Rect B and nsigma
    b = findBestMatchLocation( slope, sourceCopy, a, &nsigma, Mat() );
    // Create ARC_Pair
    pair.set_reflection( frame, b, patchSize );
    /*
    Scalar reflectionColor(0,0,0);
    Scalar pointColor (0,255,0);
    circle(sourceCopy,pair.roi.source,4,reflectionColor,-1,8,0);
    circle(sourceCopy,pair.roi.reflection,4,pointColor,-1,8,0);
    line(sourceCopy,pair.roi.source,pair.roi.reflection,reflectionColor,1,CV_AA,0);
    namedWindow("M", CV_WINDOW_AUTOSIZE );
    imshow( "M", sourceCopy );
    waitKey( 0 );
    */
    return true;
}		/* -----  end of function rematch  ----- */

// GIVEN AN IMAGE, SLOPE INFORMATION, AND A PATCHSIZE, PUTS A SEQUENCE OF
// REAL OBJECTS AND THEIR REFLECTED REGIONS IN outvector AS ARC_Pair's
int getReflections( Mat frame, Size patchSize, int numOfFeatures, double slope,
        list<ARC_Pair>& outlist )
{
	Mat sourceCopy;
    if( numOfFeatures>((frame.rows*frame.cols)/(4*patchSize.width*patchSize.height))-4 ) 
        cerr<<"Large number of features requested for given patchSize, goodFeaturesToTrack might crash\n";
	if( !frame.data ) 
    {
        cout << "Image couldn't be loaded\n";
        exit( EXIT_FAILURE );
    }

	frame.copyTo( sourceCopy );
	cvtColor( sourceCopy, sourceCopy, CV_RGB2GRAY, 1 );

    if( verbosity>=VERBOSE ) 
        cout << "Image loaded and converted to grayscale\n";	

	vector<Point> points; 

    //vector<Rect> originalMatches;

	Mat mask = Mat::ones(frame.size(),CV_8UC1)*255;

    // Goes through any ARC_Pairs already in the outvector and creates a mask to
    // prevent rematching those regions
    Size shift( 20, 20 );
    for( list<ARC_Pair>::iterator it=outlist.begin();
            it!=outlist.end(); ++it )
    {
		Rect blockedRegionSource( it->roi.source-0.5*Point(shift), shift );
		Rect blockedRegionReflection( it->roi.reflection-0.5*Point(shift), shift );
		rectangle( mask, blockedRegionSource, 0, CV_FILLED );
		rectangle( mask, blockedRegionReflection, 0, CV_FILLED );
	}

    /*
	list<ARC_Pair> tempoutlist;
	tempoutlist = outlist;
	outlist.clear();
    */

    // Adds points from goodFeaturesToTrack one by one, and creates a mask for
    // each point to prevent clustering
    // TODO: We may be able to get rid of the anti-clustering code.
    //goodFeaturesToTrack( sourceCopy, points, numOfFeatures, 0.01, patchSize.width+10, Mat(), 3, 0, 0.04);
    goodFeaturesToTrack( sourceCopy, points, numOfFeatures, 0.1, 20, mask, 3, 0, 0.04);
    /*
    for( int i=0; i<numOfFeatures; ++i )
    {
		vector<Point> tempPoint;
		goodFeaturesToTrack( sourceCopy, tempPoint, 1, .01, patchSize.width+10, Mat(), 3, 0, 0.04 );
		
		points.push_back( tempPoint[0] );	
		Rect blockedRegionSource( tempPoint[0]-Point(patchSize), 2*Point(patchSize) );
		rectangle( sourceCopy, blockedRegionSource, 0, CV_FILLED );
	}

    verbosity=VERBOSE;
    if( verbosity>=VERBOSE )
    {
        cout << "Ran goodFeaturesToTrack" << endl;
        cout << "Features To Track" << endl;
        if( verbosity>=VERY_VERBOSE )
        {
            for( size_t i=0; i<points.size(); ++i )
            {
                cout << "x: " << points[i].x-20 << " y: " << points[i].y-20 << endl;
            }
        }
    }
    verbosity=NOT_VERBOSE;
    */

    Rect frame_rect( Point(0, 0), frame.size() );
    for( vector<Point>::iterator it=points.begin();
            it!=points.end(); ++it )
    {
        Mat sourceCopy2 = frame.clone();
        Rect a, b;
        double nsigma;
        // Get Rect A
        a = Rect( *it-( .5*Point( patchSize ) ), patchSize ) & frame_rect;
        if( a.width==0 || a.height==0 ) continue;
        // Get Rect B and nsigma
		b = findBestMatchLocation( slope, sourceCopy2, a, &nsigma, Mat() );
        // Create ARC_Pair
        bool err;
        ARC_Pair pair( *it, b, nsigma, sourceCopy2, &err );
        // Add to list
        if( !err ) outlist.push_back( pair );
    }
	
    /*
	if( verbosity>=VERY_VERBOSE ) 
        cout << "Top left corners of templates\n";

	if( verbosity>=VERBOSE )
    {
		cout << "Verified Points to Match:" << endl;
		for( size_t i=0; i<points.size(); i++ )
        {
			cout << i <<" at " << points[i] << endl;
		}
	}
    */

	//runSymmetryTest( sourceCopy2, patchSize, slope, &points, &reflections, &originalMatches, &outvector );
//	identifyRealObjects( &outvector );
	
    /*
	if( displayWindows )
    {
		cout << reflections.size() << endl;
		for( size_t i=0; i<reflections.size(); i++ )
        {
			Scalar reflectionColor(0,0,0);
			Point reflectionTLCorner(reflections[i].x,reflections[i].y);
			cout << "Drew one rectangle\n";
			Scalar pointColor (0,255,0);
 			if( !displayRegions )
            {
				Point centerReflection(reflections[i].x+=patchSize/2,reflections[i].y+=patchSize/2);
       	   		Point centerSource(originalMatches[i].x+patchSize/2,originalMatches[i].y+patchSize/2);
       	     	circle(sourceCopy2,centerReflection,4,reflectionColor,-1,8,0);
				circle(sourceCopy2,centerSource,4,pointColor,-1,8,0);
       		    line(sourceCopy2,centerSource,centerReflection,reflectionColor,1,8,0);
  	     	}
 	     	else
            {
				Scalar originalColor(0,0,255);
				Point sourceTL (originalMatches[i].x,originalMatches[i].y);
				rectangle(sourceCopy2,originalMatches[i],originalColor,2,8,0);
				rectangle(sourceCopy2, reflections[i],reflectionColor ,2,8,0);
        	    line(sourceCopy2,sourceTL,reflectionTLCorner,reflectionColor,1,8,0);
    	    }
		}
	}

	for( list<ARC_Pair>::iterator it=outlist.begin();
            it!=outlist.end(); ++it )
    {
		tempoutlist.push_back( *it );
	}
	outlist.clear();
	outlist=tempoutlist;
    */
	return outlist.size();
}

/*
// DISPLAYS THE RESULTS OF getReflections()
// If getReflections was already run and outvector is full, for patchSize enter 0
// If you only have an image, enter the desired patchSize and an empty outvector of ARC_Pair's
void displayReflectionMatches( Mat image, Size patchSize, double slope, double theta, list<ARC_Pair> *outlist )
{
    getReflections( image, patchSize, 15, slope, *outlist );

    outlist->remove_if( outside_theta(theta) );
    //outlist->remove_if( outside_slope(slope) );

    outlist->remove_if( below_threshold(3.5) );
    outlist->remove_if( overlap() );
    //outlist->remove_if( outside_slope(slope) );

    Mat draw = image.clone();
    Scalar originalColor(0,0,255);
    Scalar reflectionColor(0,0,0);
    for( list<ARC_Pair>::iterator it=outlist->begin();
            it!=outlist->end(); ++it )
    {
        cout << *it << endl;
        rectangle( draw, it->roi.source, originalColor, 1, 8, 0 );
        rectangle( draw, it->roi.reflection, reflectionColor, 1, 8, 0 );
        Point sourceTLCorner = it->roi.source.tl();
        Point reflectionTLCorner = it->roi.reflection.tl();
        line( draw, sourceTLCorner, reflectionTLCorner, reflectionColor, 1, 8, 0 );
	}
	namedWindow( "Reflections", CV_WINDOW_AUTOSIZE );
    imshow( "Reflections", draw );
    waitKey(0);
}
*/
/*

// GIVEN slope INFORMATION,A source MAT AND A tmplte RECT, IT RETURNS A RECT
// OF THE REFLECTION.
Rect findOneReflection( double slope, Mat source, Rect tmplte )
{
    double nsigma;
	Point TLCorner = tmplte.tl();
	Mat templateMat = source.clone();
	templateMat = templateMat(tmplte);
	Point matchLoc = findBestMatchLocation( slope, source, templateMat, TLCorner, &nsigma );
	Rect rect( matchLoc, Size( templateMat.cols, templateMat.cols ) );
	return rect;
}

// GIVEN A source MAT, patchSize, slope INFORMATION,AND A BOOLEAN FLAG, IT
// TRIES TO RETURN A NEW GOOD FEATURE AND IT'S REFLECTION.
ARC_Pair getOneReflectionPair( Mat source, int patchSize, double slope, 
        bool *regionFound )
{
	vector<ARC_Pair> outvector;
	ARC_Pair empty;	
	getReflections( source, patchSize, 5, slope, outvector );
	if( outvector.size()==0 )
    {
		*regionFound = false;
		return empty;	 
	}
	else
    {
		*regionFound = true;
		return outvector[0];
	}
}
*/

/*
// GIVEN AN IMAGE, A PATCHSIZE FOR THE INITIAL TEMPLATES AND A SIZE FOR THE
// SECONDARY, SMALLER, NESTED TEMPLATES, PUTS A SEQUENCE OF REAL OBJECTS AND
// THEIR REFLECTIONS IN AN OUTVECTOR OF ARC_PAIR'S. 
int getReflectionsPYR( Mat &image, Size outerPatchSize, Size innerPatchSize, 
        double slope, double theta, list<ARC_Pair> &outlist )
{
	Size patchSize = innerPatchSize;
    list<ARC_Pair> initial_list;
	list<ARC_Pair> final_list;
    // Gets an initial list of regions and reflections that can then be used
    // to match much smaller templates.
    getReflections( image, outerPatchSize, 15, slope, initial_list );
	//cout << "Initial templates found\n";
	//cout << "" << endl;
	Mat imageClone = image.clone();
    
    // Filtering
    //initial_list.remove_if( below_threshold( mean[0] + N*std[0] ) ) ;
    initial_list.remove_if( outside_theta(theta) );
    initial_list.remove_if( below_threshold( 3 ) ) ;
    initial_list.remove_if( overlap() );

    Scalar red (0,0,255);
    Scalar black(0,0,0);
	//Goes through every region found in the original image
    list<ARC_Pair> sublist;
    for( list<ARC_Pair>::iterator it=initial_list.begin();
            it!=initial_list.end(); ++it )
    {
		vector<Point> features;

        // Finds good points to track within one of the regions while
        // checking for proximity to boundaries and converting to the
        // original coordinate system.
        Mat gft_mask, gft_masked_scene;
        gft_mask = Mat::zeros( image.size(), CV_8UC1 );
        rectangle( gft_mask, it->roi.source, 255, CV_FILLED );
        imageClone.copyTo( gft_masked_scene, gft_mask );
        //Mat original = imageClone( it->roi.source );
        Mat original = imageClone;
		cvtColor( original, original, CV_RGB2GRAY, 1 );
		goodFeaturesToTrack( original, features, 3, .01, patchSize.width+3, gft_mask, 3, false, 0.04 );
		//Creates a mask from the original larger reflected region, from which smaller reflection matches will be found
		Mat mask, masked_scene;
		mask = Mat::zeros( image.size(), CV_8UC1 );
		rectangle( mask, it->roi.reflection, 255, CV_FILLED );
        
        Rect frame_rect( it->roi.source.tl(), it->roi.source.size() );
        for( vector<Point>::iterator subit=features.begin();
            subit!=features.end(); ++subit )
        {
            Rect a, b;
            double nsigma;
            // Get Rect A
            a = Rect( *subit-( .5*Point( patchSize ) ), patchSize ) & frame_rect;
            if( a.width==0 || a.height==0 ) continue;
            // Get Rect B and nsigma
            b = findBestMatchLocation( slope, imageClone, a, &nsigma, mask, false );
            // Create ARC_Pair
            ARC_Pair pair( a, b, nsigma );
            // Add to list
            sublist.push_back( pair );
        }
        //sublist.remove_if( below_threshold( 25.262 ) );
        //sublist.remove_if( overlap() );
        //sublist.remove_if( outside_theta(theta) );
        sublist.remove_if( below_threshold(22) );
	}
    //namedWindow( "MARTIN", CV_WINDOW_AUTOSIZE );
    //imshow( "MARTIN", image );
    //waitKey( 15 );
	//cout << "Reflections found\n";
	//return outvector.size();
    outlist = sublist;
    return 1;
}
*/
