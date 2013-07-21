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

//CREATES MAT TEMPLATES OF patchSize WIDTH AND HEIGHT, AND STORES THEM IN
//templates AND outvector
void createTemplatesFromVector( Mat image, Size patchSize, vector<Point> *points, 
        list<ARC_Pair> *outlist )
{
	int pointsLength = points->size();			
	for( int i=0; i<pointsLength; i++ )
    {
        //Figures out the top left corners from the goodfeaturestotrack points
        (*points)[i]-= Point( patchSize )*.5;
        if( verbosity==VERY_VERBOSE ) 
            cout<<" x: "<<(*points)[i].x<<" y: "<<(*points)[i].y<<endl;

        // TODO: Why is that necessary?
        // Only uses points that are within a patchSize distance away from
        // the borders of the source image.
        if((*points)[i].x<0 || (*points)[i].y<0 
                || image.cols-(*points)[i].x-patchSize.width<0 
                || image.rows-(*points)[i].y-patchSize.height<0 )
        { 
			points->erase(points->begin()+i);
			if( verbosity==VERY_VERBOSE ) 
                cout<<"failed\n";
			i--;
            pointsLength--;
		}
		else
        {	
            Rect rect( (*points)[i], patchSize );
            if( verbosity>=VERY_VERBOSE ) cout << "Created template rect\n";
			ARC_Pair temp_pair;
            temp_pair.roi.source = rect;
            outlist->push_back( temp_pair );
		}
	}
}

//RETURNS THE TOP-LEFT CORNER OF THE REFLECTION OF sourceTemplate ON image
Point findBestMatchLocation( double slope, Mat image, Rect source_rect, double* nsigma )
{
    Mat image_copy = image.clone();
    Point2f TLCornerTemplate = source_rect.tl();
    Mat sourceTemplate = image_copy( source_rect );
    // flips template to get the reflection used for matching
    flip( sourceTemplate, sourceTemplate, 0 );
    // Creates results matrix where the top left corner of the 
    // template is slid across each pixel of the source
    int result_cols = image.cols-sourceTemplate.cols+1;
    int result_rows = image.rows-sourceTemplate.rows+1;
    Mat result;
    result.create( result_cols,result_rows, CV_32FC1 );

    //0:CV_TM_SQDIFF
    //1:CV_TM_SQDIFF_NORMED
    //2:CV_TM_CORR
    //3:CV_TM_CCOR_NORMED
    //4:CV_TM_CCOEFF
    //5:CV_TM_CCOEFF_NORMED <----Most succesful at finding reflections
    
    int match_method = CV_TM_CCOEFF_NORMED; // 4 seemed good for stddev thresholding.

    // Creates a search region around the template given slope information
    // TODO: When slope is near horizontal, several times the mask 
    // isn't created properly - FIX 
    Mat mask, masked_scene;
    mask = Mat::zeros(image.size(),CV_8UC1);
    int xBot = (-TLCornerTemplate.y/slope)+TLCornerTemplate.x;
    int xTop = ((480-TLCornerTemplate.y)/slope)+TLCornerTemplate.x;
    int xStripSide= ((TLCornerTemplate.y-sourceTemplate.rows-TLCornerTemplate.y)/slope)+TLCornerTemplate.x;
    int difference = abs(TLCornerTemplate.x-xStripSide);
    int TLx = xTop;
    int TLy=480;
    int TRx = xTop+difference+sourceTemplate.cols;
    int TRy=480;
    Point searchRegion[1][4];
    int searchMargin=0;//Change this to increase width of searchRegion
    searchRegion[0][0] = Point (TLx-searchMargin,TLy);
    searchRegion[0][1] = Point (xBot-searchMargin,0);
    searchRegion[0][2] = Point (xBot+difference+sourceTemplate.cols+searchMargin,0);
    searchRegion[0][3] = Point (TRx+searchMargin,TRy);

    const Point* ppt[1] = {searchRegion[0]};
    int npt[] = {4};
    fillPoly(mask,ppt,npt,1,Scalar(255,255,255));

    if( searchRegion[0][1].x>640 )
    {
        Mat mask2 = Mat::zeros( Size( searchRegion[0][2].x,480 ), CV_8UC1 );
        fillPoly( mask2, ppt, npt, 1, Scalar( 255,255,255 ) );
        mask = mask2( Rect( 0, 0, 640, 480 ) );
    }

    if( searchRegion[0][2].x<0 )
    {
        Point shift( -searchRegion[0][1].x, 0 );
        searchRegion[0][0]+=shift;
        searchRegion[0][1]+=shift;
        searchRegion[0][2]+=shift;
        searchRegion[0][3]+=shift;
        Mat mask2 = Mat::zeros( Size( 640-searchRegion[0][1].x, 480 ), CV_8UC1 );
        fillPoly( mask2, ppt, npt, 1, Scalar( 255, 255, 255 ) );
        mask = ( mask2( Rect( -searchRegion[0][1].x, 0, 640, 480 ) ) );
    }	

    image.copyTo( masked_scene, mask );
    matchTemplate( masked_scene, sourceTemplate, result, match_method );

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
    return matchLoc;
}


//FINDS THE REFLECTIONS OF THE IMAGES IN templates ON image AND STORES THEM IN reflections AND outvector
void findReflections( Mat image, Size patchSize, double slope, list<ARC_Pair> *outlist )
{
    for( list<ARC_Pair>::iterator it=outlist->begin();
            it!=outlist->end(); ++it )
    {
        Mat copy = image.clone();
        double nsigma;

		Point matchLoc = findBestMatchLocation( slope, image, it->roi.source, &nsigma );
		
        Rect rect( matchLoc, patchSize );
		it->roi.reflection = rect;
		it->nsigma = nsigma;
	}
}

/*  
//CHECKS TO SEE IF WHEN THE MATCHED REFLECTION IS RUN THROUGH MATCHTEMPLATE, IT GIVES THE LOCATION OF THE ORIGINAL SOURCE TEMPLATE
void runSymmetryTest(Mat frame, int patchSize, double slope, 
        vector<Point> *points, vector<Rect> *reflections, 
        vector<Rect> *originalMatches, vector<ARC_Pair> *outvector)
{
	if( verbosity>=VERBOSE ) 
    {
        cout << "\n\n" <<endl;
        cout << "Running symmetry test\n";
    }
	for( size_t i=0; i<reflections->size(); i++ )
    {
        double nsigma;
		Mat temp = frame.clone();	
		Mat refl = temp( (*reflections)[i] );
		
        // Runs the reflections found through matchTemplate to get another
        // vector of matched originals
        Point tlCorner = (*reflections)[i].tl();
		Point matchLoc = findBestMatchLocation( slope, frame, refl, tlCorner, &nsigma );

		Rect org_rfl( matchLoc.x,matchLoc.y, patchSize,patchSize );
		originalMatches->push_back( org_rfl );
	
	}
	
	int symmetrySize = reflections->size();
	for( int i=0; i<symmetrySize; i++ )
    {
        // If the originals found through MatchTemplate are more than a
        // patchSize distance from the actual original templates, the matches
        // are discarded Also if the matches are right on top of each other,
        // they're discarded too
		int xdifference = abs((*points)[i].x-(*originalMatches)[i].x);
		int ydifference = abs((*points)[i].y-(*originalMatches)[i].y);
		int reflectionYdifference = abs( (*points)[i].y-(*reflections)[i].y );
		if( xdifference>patchSize || ydifference>patchSize ||  reflectionYdifference<patchSize )
        {
			points->erase( points->begin()+i );
			reflections->erase( reflections->begin()+i );
			originalMatches->erase( originalMatches->begin()+i );
			outvector->erase( outvector->begin()+i );
			i--; 
            symmetrySize--;
		}
		if( verbosity==VERY_VERBOSE ) 
            cout << "Outvector array size after test: " << (*reflections).size() << endl;
	}
}

// VERIFIES THAT THE SOURCES AND REFLECTIONS IN THE ARC_Pair's ARE CORRECT
// TODO: After identifying all the real objects, then make sure there are no overlapping regions
void identifyRealObjects(vector<ARC_Pair> *outvector)
{
    // If the y coordinate is lower, that is the real object, otherwise it is
    // the reflection.
    for( size_t i=0; i<outvector->size(); i++ )
    {
        if( (*outvector)[i].roi.source.y>(*outvector)[i].roi.reflection.y )
        {
            Rect temp = (*outvector)[i].roi.reflection;
            (*outvector)[i].roi.reflection = (*outvector)[i].roi.source;
            (*outvector)[i].roi.source = temp;
        }
		cout << "Outvector source at: " <<(*outvector)[i].roi.source.tl() 
             << " and reflection at: " << (*outvector)[i].roi.reflection.tl() << endl;
	}
}
*/

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

    /*
	Mat mask = Mat::ones(frame.size(),CV_8UC1)*255;

    // Goes through any ARC_Pairs already in the outvector and creates a mask to
    // prevent rematching those regions
    Point shift30( 30, 30 );
    Size shift60( 60, 60 );
    for( list<ARC_Pair>::iterator it=outlist.begin();
            it!=outlist.end(); ++it )
    {
		Rect blockedRegionSource( it->roi.source.tl()-shift30, it->roi.source.size() + shift60 );
		Rect blockedRegionReflection( it->roi.reflection.tl()-shift30, it->roi.reflection.size() + shift60 );
		rectangle( mask, blockedRegionSource, 0, CV_FILLED );
		rectangle( mask, blockedRegionReflection, 0, CV_FILLED );
	}

	list<ARC_Pair> tempoutlist;
	tempoutlist = outlist;
	outlist.clear();
    */

    // Adds points from goodFeaturesToTrack one by one, and creates a mask for
    // each point to prevent clustering
    // TODO: We may be able to get rid of the anti-clustering code.
    goodFeaturesToTrack( sourceCopy, points, numOfFeatures, 0.01, 10, Mat(), 3, 0, 0.04);
    /*
    for( list<ARC_Pair>::iterator it=outlist.begin();
            it!=outlist.end(); ++it )
    {
		vector<Point> tempPoint;
		goodFeaturesToTrack( sourceCopy, tempPoint, 1, .01, patchSize.width+10, mask, 3, 0, 0.04 );
		
		points.push_back( tempPoint[0] );	
		Rect blockedRegionSource( tempPoint[0]-Point(patchSize), 2*Point(patchSize) );
		rectangle( mask, blockedRegionSource, 0, CV_FILLED );
	}
    */

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

	Mat sourceCopy2 = frame.clone();
    //TODO Why median blur?
	//medianBlur( frame, frame, 3 );
    // TODO: redefine purpose
	createTemplatesFromVector( frame, patchSize, &points, &outlist );
	
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

	findReflections( sourceCopy2, patchSize, slope, &outlist );
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

// ===  FUNCTION  ======================================================================
//         Name:  compare_arc_pair
//  Description:  
// =====================================================================================

struct overlap {
    //overlap( double t ): threshold(t){}
    bool operator() (const ARC_Pair& value ) { return( (value.roi.source&value.roi.reflection).area()>0 ); }
};
struct below_threshold {
    below_threshold( double t ): threshold(t){}
    bool operator() (const ARC_Pair& value ) { return( value.nsigma<threshold ); }
    private:
    double threshold;
};
struct outside_slope {
    outside_slope( double m ): slope(m){}
    bool operator() (const ARC_Pair& value ) 
    { 
        Point del = value.roi.source.tl()-value.roi.reflection.tl();
        double match_slope = (del.x==0) ? 100 : del.y/del.x;
        cout << "Match slope: " << match_slope << endl;
        double ratio = match_slope/slope;
        return( ratio>1.5 || ratio<0.5 );
    }
    private:
    double slope;
};
// DISPLAYS THE RESULTS OF getReflections()
// If getReflections was already run and outvector is full, for patchSize enter 0
// If you only have an image, enter the desired patchSize and an empty outvector of ARC_Pair's
void displayReflectionMatches( Mat image, Size patchSize, double slope, list<ARC_Pair> *outlist )
{
    getReflections( image, patchSize, 15, slope, *outlist );
    outlist->remove_if( below_threshold(7.0) );
    outlist->remove_if( overlap() );
    outlist->remove_if( outside_slope(slope) );

    Mat draw = image.clone();
    Scalar originalColor(0,0,255);
    Scalar reflectionColor(0,0,0);
    for( list<ARC_Pair>::iterator it=outlist->begin();
            it!=outlist->end(); ++it )
    {
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

// GIVEN AN IMAGE, A PATCHSIZE FOR THE INITIAL TEMPLATES AND A SIZE FOR THE
// SECONDARY, SMALLER, NESTED TEMPLATES, PUTS A SEQUENCE OF REAL OBJECTS AND
// THEIR REFLECTIONS IN AN OUTVECTOR OF ARC_PAIR'S. 
/*
int getReflectionsPYR(Mat &image, int outerPatchSize, int innerPatchSize, 
        double slope, vector<ARC_Pair> &outvector)
{
	int patchSize = innerPatchSize;
    vector<ARC_Pair> initialvector;
	vector<ARC_Pair> finalvector;
    // Gets an initial list of regions and reflections that can then be used
    // to match much smaller templates.
    getReflections( image, outerPatchSize, 15, slope, initialvector );
	cout << "Initial templates found\n";
	cout << "" << endl;
	Mat imageClone = image.clone();

	//Goes through every region found in the original image
	for( size_t m=0; m<initialvector.size(); m++ )
    {
		vector<ARC_Pair> subvector;
		vector<Point> features;
		vector<Point> shiftedFeatures;
		vector<Mat> templates;
		vector<Rect> reflections;
		ARC_Pair pair = initialvector[m];

		//Draws the initial regions and their reflections
		Scalar red (0,0,255);
		Scalar black(0,0,0);
		rectangle( image,pair.roi.source, red, 2, 8, 0 );
		rectangle( image,pair.roi.reflection, black, 2, 8, 0 );
		Point originalCorner = pair.roi.source.tl();
		Point reflectionCorner = pair.roi.reflection.tl();
		line( image, originalCorner, reflectionCorner, black, 2, 8, 0 );
		
		//Finds good points to track within one of the regions while checking for proximity to boundaries and converting to the original coordinate system
		Mat original = imageClone( pair.roi.source );
		cvtColor( original, original, CV_RGB2GRAY, 1 );
		goodFeaturesToTrack( original, features, 3, .01, patchSize+3, noArray(), 3, false, 0.04 );
		for( size_t i=0; i<features.size(); i++ )
        {
			Point shift = pair.roi.source.tl();
			Point shiftedPoint(features[i]+shift);
			if( shiftedPoint.x>pair.roi.source.x+patchSize/2 
                    && shiftedPoint.y>pair.roi.source.y+patchSize/2 
                    && shiftedPoint.x<pair.roi.source.x+pair.roi.source.width-patchSize/2 
                    && shiftedPoint.y<pair.roi.source.y+pair.roi.source.height-patchSize/2) 
            {
                shiftedFeatures.push_back(shiftedPoint);
            }
		}

		createTemplatesFromVector( imageClone, patchSize, &shiftedFeatures, &templates, &subvector );
		//Creates a mask from the original larger reflected region, from which smaller reflection matches will be found
		Mat mask, masked_scene;
		mask = Mat::zeros( image.size(), CV_8UC1 );
		rectangle( mask, pair.roi.reflection, 255, CV_FILLED );
		imageClone.copyTo( masked_scene, mask );

		findReflections( masked_scene, patchSize, slope, &shiftedFeatures, &templates, &reflections, &subvector );
		identifyRealObjects(&subvector);
	
		//Pushes all matches found back to outvector
		for( size_t j=0; j<subvector.size(); j++ )
        {
			outvector.push_back(subvector[j]);	
		}	
	}
	cout << "Reflections found\n";
	return outvector.size();
}
*/
