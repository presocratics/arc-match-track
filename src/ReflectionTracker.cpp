//Track the reflections of templates across water
//Author: Simon Peter 
//speter3@illinois.edu

//To modify the the vertical strip around the template used as a mask, change 'searchMargin' in findBestMatchLocation()
//To modify the number of features goodFeaturesToTrack should find, change 'numOfFeatures' in getReflections()
/*
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

//CREATES MAT TEMPLATES OF patchSize WIDTH AND HEIGHT, AND STORES THEM IN templates AND outvector
void createTemplatesFromVector(Mat image, int patchSize, vector<Point> *points, vector<Mat> *templates, vector<ARC_Pair> *outvector){

	int pointsLength = points->size();			
	for( int i=0; i<pointsLength; i++ )
    {
		(*points)[i].x-=patchSize/2; (*points)[i].y-=patchSize/2;//Figures out the top left corners from the goodfeaturestotrack points
        if( verbosity==VERY_VERBOSE ) 
            cout<<" x: "<<(*points)[i].x<<" y: "<<(*points)[i].y<<endl;
		//Only uses points that are within a patchSize distance away from the borders of the source image
		if((*points)[i].x<0 || (*points)[i].y<0 || image.cols-(*points)[i].x-patchSize < 0 || image.rows-(*points)[i].y-patchSize<0 )
        { 
			(*points).erase((*points).begin()+i);
			if(verbosity==VERY_VERBOSE) 
                cout<<"failed\n";
			i--;
            pointsLength--;
		}
		else
        {	
            Rect rect ((*points)[i].x,(*points)[i].y,patchSize,patchSize);
            if( verbosity>=VERY_VERBOSE ) cout << "Created template rect\n";
			ARC_Pair temp_pair;
			temp_pair.iter_count = 0;
			temp_pair.no_match = 0;
			temp_pair.direction.match = DOWN;
			temp_pair.direction.track = DOWN;
			temp_pair.slope=-INFINITY;
            temp_pair.roi.source = rect;
            (*outvector).push_back( temp_pair );

            Mat temp = image.clone();
            temp = temp( rect );
            (*templates).push_back( temp );
            if( verbosity>=VERY_VERBOSE ) 
                cout<<"Added template rect to output vector\n";
		}
	}
}

//RETURNS THE TOP-LEFT CORNER OF THE REFLECTION OF sourceTemplate ON image
Point findBestMatchLocation(double slope, Mat image,  Mat sourceTemplate, Point TLCornerTemplate){

		flip(sourceTemplate,sourceTemplate,0);//flips template to get the reflection used for matching
		//Creates results matrix where the top left corner of the template is slid across each pixel of the source
		int result_cols = image.cols-sourceTemplate.cols+1;
		int result_rows = image.rows-sourceTemplate.rows+1;
		Mat result;
		result.create(result_cols,result_rows, CV_32FC1);

		//0:CV_TM_SQDIFF
		//1:CV_TM_SQDIFF_NORMED
		//2:CV_TM_CORR
		//3:CV_TM_CCOR_NORMED
		//4:CV_TM_CCOEFF
		//5:CV_TM_CCOEFF_NORMED <----Most succesful at finding reflections
		
		int match_method = 5;

		//Creates a search region around the template given slope information 
		Mat mask; Mat masked_scene;
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

		if(searchRegion[0][1].x>640){
			Mat mask2 = Mat::zeros(Size(searchRegion[0][2].x,480),CV_8UC1);
			fillPoly(mask2,ppt,npt,1,Scalar(255,255,255));
			mask = mask2(Rect(0,0,640,480));
		
		}

		if(searchRegion[0][2].x<0){
			Point shift (-searchRegion[0][1].x,0);
			searchRegion[0][0]+=shift;
			searchRegion[0][1]+=shift;
			searchRegion[0][2]+=shift;
			searchRegion[0][3]+=shift;
			Mat mask2 = Mat::zeros(Size(640-searchRegion[0][1].x,480),CV_8UC1);
			fillPoly(mask2,ppt,npt,1,Scalar(255,255,255));
			mask = (mask2(Rect(-searchRegion[0][1].x,0,640,480)));
		}	
*/
		/*
		//Specifies vertical search region with a mask 
		Mat mask; Mat masked_scene;
		mask = Mat::zeros( image.size(), CV_8UC1 );
        int searchMargin = 30;//extends this many pixels to the left and right of template width
		int leftSearchArea = TLCornerTemplate.x-searchMargin;
		Rect searchRegion (leftSearchArea,0,sourceTemplate.cols+2*searchMargin,image.rows);
		rectangle(mask,searchRegion,255,CV_FILLED);
		*/
	
/*
		image.copyTo(masked_scene,mask);
		matchTemplate(masked_scene, sourceTemplate, result, match_method);
		normalize(result,result,0,1,NORM_MINMAX,-1,Mat());

        if( verbosity>=VERY_VERBOSE ) 
            cout << "Ran matchTemplate and normalized\n";
		double minVal; double maxVal; 
		Point minLoc; Point maxLoc; Point matchLoc;
		minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, Mat());
		 
		if( match_method == 0 || match_method == 1 )
        {
			matchLoc = minLoc;	
		}
		else 
        {
			matchLoc = maxLoc;
		}
		//matchLoc is the location of the top left corner of the reflection that matchTemplate found
		return matchLoc;


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
}

//FINDS THE REFLECTIONS OF THE IMAGES IN templates ON image AND STORES THEM IN reflections AND outvector
void findReflections(Mat image, int patchSize, double slope, vector<Point> *points, vector<Mat> *templates, vector<Rect> *reflections, vector<ARC_Pair> *outvector){


	for( size_t i=0; i<(*points).size(); i++ )
    {
		Mat tmplte = (*templates)[i];

		Point TLCorner((*points)[i].x,(*points)[i].y);
		Point matchLoc = findBestMatchLocation(slope,image,tmplte,TLCorner);
		
		Rect rect(matchLoc.x, matchLoc.y, patchSize, patchSize);
		(*outvector)[i].roi.reflection = rect;

		(*reflections).push_back( rect ); 
	}

	
}

//CHECKS TO SEE IF WHEN THE MATCHED REFLECTION IS RUN THROUGH MATCHTEMPLATE, IT GIVES THE LOCATION OF THE ORIGINAL SOURCE TEMPLATE
void runSymmetryTest(Mat frame, int patchSize, double slope, vector<Point> *points, vector<Rect> *reflections, vector<Rect> *originalMatches, vector<ARC_Pair> *outvector){

	if( verbosity>=VERBOSE ) 
    {
        cout << "\n\n" <<endl;
        cout << "Running symmetry test\n";
    }
	for( size_t i=0; i<(*reflections).size(); i++)
    {
		Mat temp = frame.clone();	
		Mat refl = temp((*reflections)[i]);
		
		//Runs the reflections found through matchTemplate to get another vector of matched originals
		Point tlCorner ((*reflections)[i].x,(*reflections)[i].y);
		Point matchLoc = findBestMatchLocation(slope,frame,refl,tlCorner);

		Rect org_rfl( matchLoc.x,matchLoc.y, patchSize,patchSize );
		(*originalMatches).push_back( org_rfl );
	
	}
	
	int symmetrySize = (*reflections).size();
	for( int i=0; i<symmetrySize; i++ )
    {
		//If the originals found through MatchTemplate are more than a patchSize distance from the actual original templates, the matches are discarded
		//Also if the matches are right on top of each other, they're discarded too
		int xdifference = abs((*points)[i].x-(*originalMatches)[i].x);
		int ydifference = abs((*points)[i].y-(*originalMatches)[i].y);
		int reflectionYdifference = abs((*points)[i].y-(*reflections)[i].y);
		if( xdifference>patchSize || ydifference>patchSize ||  reflectionYdifference<patchSize )
        {
			(*points).erase((*points).begin()+i);
			(*reflections).erase((*reflections).begin()+i);
			(*originalMatches).erase((*originalMatches).begin()+i);
			(*outvector).erase((*outvector).begin()+i);
			i--; symmetrySize--;
		}
		if( verbosity==VERY_VERBOSE ) 
            cout << "Outvector array size after test: " << (*reflections).size() << endl;
	}
}

//VERIFIES THAT THE SOURCES AND REFLECTIONS IN THE ARC_Pair's ARE CORRECT
void identifyRealObjects(vector<ARC_Pair> *outvector){

	//If the y coordinate is lower, that is the real object, otherwise it is the reflection
	for( size_t i=0; i<(*outvector).size(); i++ )
    {
        if( (*outvector)[i].roi.source.y>(*outvector)[i].roi.reflection.y )
        {
            Rect temp = (*outvector)[i].roi.reflection;
            (*outvector)[i].roi.reflection = (*outvector)[i].roi.source;
            (*outvector)[i].roi.source = temp;
        }
		//cout<<"Outvector source at ("<<(*outvector)[i].roi.source.x<<","<<(*outvector)[i].roi.source.y<<") and reflection at ("<<(*outvector)[i].roi.reflection.x<<","			<<(*outvector)[i].roi.reflection.y<<")\n";
	}
}

//GIVEN AN IMAGE, SLOPE INFORMATION, AND A PATCHSIZE, PUTS A SEQUENCE OF REAL OBJECTS AND THEIR REFLECTED REGIONS IN outvector AS ARC_Pair's
int getReflections(Mat frame, int patchSize, int numOfFeatures, double slope, vector<ARC_Pair> &outvector){
	
    if(numOfFeatures>((frame.rows*frame.cols)/(pow(patchSize*2,2)))-4) cout<<"Large number of features requested for given patchSize, goodFeaturesToTrack might crash\n";
	if(!frame.data) cout << "Image couldn't be loaded\n";
	//namedWindow( "Source", CV_WINDOW_AUTOSIZE );

	Mat sourceCopy;
	frame.copyTo( sourceCopy );
	cvtColor( sourceCopy, sourceCopy, CV_RGB2GRAY, 1 );

    if( verbosity>=VERBOSE ) 
        cout << "Image loaded and converted to grayscale\n";	

	vector<Point> points; 
    vector<Mat> templates[2]; 
    vector<Rect> reflections; 
    vector<Rect> originalMatches;


	Mat mask = Mat::ones(frame.size(),CV_8UC1)*255;
//Goes through any ARC_Pairs already in the outvector and creates a mask to prevent rematching those regions
	for(unsigned i=0;i<outvector.size();i++){
		Rect blockedRegionSource(outvector[i].roi.source.x-30,outvector[i].roi.source.y-30,outvector[i].roi.source.width+60,outvector[i].roi.source.width+60);
		Rect blockedRegionReflection(outvector[i].roi.reflection.x-30,outvector[i].roi.reflection.y-30,outvector[i].roi.reflection.width+60,outvector[i].roi.reflection.width+60);
		rectangle(mask,blockedRegionSource,0,CV_FILLED);
		rectangle(mask,blockedRegionReflection,0,CV_FILLED);
	}

	vector<ARC_Pair> tempoutvector;
	tempoutvector = outvector;
	outvector.clear();
//Adds points from goodFeaturesToTrack one by one, and creates a mask for each point to prevent clustering
	for(int counter = outvector.size();counter<numOfFeatures;counter++){
		vector<Point> tempPoint;
		goodFeaturesToTrack(sourceCopy,tempPoint,1,.01,patchSize+10,mask,3,0,0.04);
		
		points.push_back(tempPoint[0]);	
		Rect blockedRegionSource (tempPoint[0].x-patchSize,tempPoint[0].y-patchSize,2*patchSize,2*patchSize);
		rectangle(mask,blockedRegionSource,0,CV_FILLED);
	}

    if( verbosity>=VERBOSE )
    {
        cout << "Ran goodFeaturesToTrack" << endl;
        cout << "Features To Track" << endl;
        if( verbosity>=VERY_VERBOSE )
        {
            for( size_t i=0; i<points.size(); ++i )
            {
                cout<<"x: "<<points[i].x-20<<" y: "<<points[i].y-20<<endl;
            }
        }
    }

	Mat sourceCopy2 = frame.clone();
	medianBlur(frame,frame,3);
	createTemplatesFromVector(frame,patchSize,&points,&templates[0],&outvector);
	
	if( verbosity>=VERY_VERBOSE ) 
        cout<<"Top left corners of templates\n";

	if( verbosity>=VERBOSE )
    {
		cout << "Verified Points to Match:" << endl;
		for( size_t i=0; i<points.size(); i++ )
        {
			cout << i <<" x: " << points[i].x << " y: " << points[i].y << endl;
		}
	}

	findReflections(sourceCopy2,patchSize,slope,&points,&templates[0],&reflections,&outvector);
	runSymmetryTest(sourceCopy2,patchSize,slope,&points,&reflections,&originalMatches,&outvector);
	identifyRealObjects(&outvector);
	
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

	for(unsigned int i=0;i<outvector.size();i++){
		tempoutvector.push_back(outvector[i]);
	}
	outvector.clear();
	outvector=tempoutvector;
	return outvector.size();
}

//DISPLAYS THE RESULTS OF getReflections()
//If getReflections was already run and outvector is full, for patchSize enter 0
//If you only have an image, enter the desired patchSize and an empty outvector of ARC_Pair's
void displayReflectionMatches(Mat image, int patchSize, double slope,vector<ARC_Pair> *outvector){
	
	//			time_t beginning;
	//			beginning=time(NULL);
	int outvectorSize;
	if(patchSize != 0){
		outvectorSize = getReflections(image,patchSize,15,slope,*outvector);
	}
	else {
		outvectorSize = (*outvector).size();
	}
	//cout<<"Outvector size: "<<outvectorSize<<endl;

    Mat draw = image.clone();
    Scalar originalColor(0,0,255);
    Scalar reflectionColor(0,0,0);
    for(int i=0;i<outvectorSize;i++){
        rectangle(draw,(*outvector)[i].roi.source,originalColor,2,8,0);
        rectangle(draw,(*outvector)[i].roi.reflection,reflectionColor,2,8,0);
        Point sourceTLCorner ((*outvector)[i].roi.source.x,(*outvector)[i].roi.source.y);
        Point reflectionTLCorner ((*outvector)[i].roi.reflection.x,(*outvector)[i].roi.reflection.y);
        line(draw,sourceTLCorner,reflectionTLCorner,reflectionColor,2,8,0);
	}
	//			time_t end;
	//			end = time(NULL);
	//			double timeElapsed = difftime(beginning,end);
	//			cout<<"Time Elapsed: "<<timeElapsed<<endl;
	namedWindow("Reflections",CV_WINDOW_AUTOSIZE);
    imshow("Reflections",draw);
    waitKey(0);

}

//GIVEN slope INFORMATION,A source MAT AND A tmplte RECT, IT RETURNS A RECT OF THE REFLECTION
Rect findOneReflection(double slope,Mat source, Rect tmplte){

	Point TLCorner(tmplte.x,tmplte.y);
	Mat templateMat = source.clone();
	templateMat = templateMat(tmplte);
	Point matchLoc = findBestMatchLocation(slope, source,templateMat,TLCorner);
	Rect rect (matchLoc.x,matchLoc.y,templateMat.cols,templateMat.cols);
	return rect;
}

//GIVEN A source MAT, patchSize, slope INFORMATION,AND A BOOLEAN FLAG, IT TRIES TO RETURN A NEW GOOD FEATURE AND IT'S REFLECTION  
ARC_Pair getOneReflectionPair(Mat source, int patchSize, double slope,bool *regionFound){

	vector<ARC_Pair> outvector;
	ARC_Pair empty;	
	getReflections(source,patchSize,5,slope,outvector);
	if(outvector.size()==0){
		*regionFound = false;
		return empty;	 
	}
	else{
		*regionFound = true;
		return outvector[0];
	}
}
*/
